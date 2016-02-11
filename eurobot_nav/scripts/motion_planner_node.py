#!/usr/bin/env python
# coding: utf-8

import rospy
import numpy as np
import tf2_ros
from tf.transformations import euler_from_quaternion
from visualization_msgs.msg import Marker, MarkerArray
# from geometry_msgs.msg import Twist
from std_msgs.msg import String
from threading import Lock
from geometry_msgs.msg import Twist, TwistStamped
from core_functions import cvt_global2local, cvt_local2global, wrap_angle, calculate_distance
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Point
from collision_avoidance import CollisionAvoidanceMainRobot, CollisionAvoidanceSecondaryRobot
from path_planner import PathPlanning
from polygon_conversions import list_from_polygon, list_from_polygon_array, polygon_list_from_numpy, get_collision_polygon



class MotionPlannerNode:
    def __init__(self):
        self.robot_name = rospy.get_param("robot_name")
        rospy.loginfo(self.robot_name)
        rospy.init_node("motion_planner", anonymous=True)
#       init motion planner params
        self.V_MAX = rospy.get_param("V_MAX")   # m/s
        self.num_points_in_path = rospy.get_param("num_points_in_path")
        self.R_DEC = rospy.get_param("R_DECELERATION")
        self.k = rospy.get_param("k")
        self.min_dist_to_goal_point = rospy.get_param("min_dist_to_goal_point")
        self.e_const = rospy.get_param("exp_const")
        self.velocity_vector = np.array(rospy.get_param("velocity_vector"))
        self.acceleration_vector = self.velocity_vector * 4 * self.velocity_vector[2]
        self.k_linear_wp = rospy.get_param("k_linear_wp")
        self.k_linear_vp = rospy.get_param("k_linear_wp")
        self.RATE = float(rospy.get_param("RATE"))
        self.XY_GOAL_TOLERANCE = rospy.get_param("XY_GOAL_TOLERANCE")
        self.YAW_GOAL_TOLERANCE = rospy.get_param("YAW_GOAL_TOLERANCE")
        self.robot_radius = rospy.get_param("robot_radius")
        self.oponent_robot_radius = rospy.get_param("oponent_robot_radius")
#       init world borders, playing elements, playing area
        self.world_border_bottom = np.array([[0.1, 0.], [0.1, self.robot_radius], [2.9, self.robot_radius], [2.9, 0.]])
        self.world_border_left = np.array([[0., 0.], [0., 2.], [self.robot_radius, 2.], [self.robot_radius, 0.]])
        self.world_border_top = np.array([[0.1, 1.9], [3., 1.9], [3., 2. - self.robot_radius],
                                          [0.1, 2. - self.robot_radius]])
        self.world_border_right = np.array([[3., 0.], [3., 2.], [3. - self.robot_radius, 2.],
                                            [3. - self.robot_radius, 0.]])
        self.scales = np.array([[0.45 - self.robot_radius, 2], [0.45 - self.robot_radius, 1.543 - self.robot_radius],
                                [2.55 + self.robot_radius, 1.543 - self.robot_radius],
                                [2.55 + self.robot_radius, 2], [0.45 - self.robot_radius, 2]])
#       set values to default
        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer)
        self.is_collision = False
        self.mutex = Lock()
        self.path = np.zeros((0, 3))
        self.obstacle_polygon = []
        self.way_points = None
        self.way_point_ind = 0
        self.buf_path = self.path
        self.global_goal = None
        self.prev_vel = np.array([0., 0., 0.])
        self.result_vel = np.array([0., 0., 0.])
        self.cmd_id = None
        self.cmd_type = None
        self.cmd_args = None
        self.t_prev = None
        self.goal = np.zeros((3, 0))
        self.current_cmd = None
        self.current_state = 'start'
        self.delta_dist = 0
        self.vel = np.zeros(3)
        self.theta_diff = None
        self.d_norm = None
        self.gamma = None
        self.coords = np.zeros((3, 0))
        self.start_collision_point = None
        self.dist_to_start_collision_point = None
        self.path = np.array([[]])
        self.point = np.array([0., 0., 0.])
        self.path_left = 9999  # big initial value
        self.distance_map_frame = None
        self.stop_id = 0
        self.timer = None
        self.buf_goal = None
        self.is_robot_stopped = False
        self.distances = np.array([100., 100., 100.])
        self.prev_time = rospy.Time.now().to_sec()
        self.p = 1
#       waiting for robot coords
        while not self.update_coords():
            rospy.sleep(1)
#       init rospy publishers
        self.collision_area_publisher = rospy.Publisher("collision_area", Marker, queue_size=10)
        self.command_publisher = rospy.Publisher("/%s/stm/command" % self.robot_name, String, queue_size=1)
        self.response_publisher = rospy.Publisher("response", String, queue_size=10)
        # self.point_publisher = rospy.Publisher("obstacle", MarkerArray, queue_size=10)
        self.twist_publisher = rospy.Publisher("cmd_vel", Twist, queue_size=1)
        self.path_publisher = rospy.Publisher('path', Path, queue_size=10)
        self.world_publisher = rospy.Publisher("world", MarkerArray, queue_size=1)
        self.twist_stamped_publihser = rospy.Publisher("velocity_vectors", TwistStamped, queue_size=1)
#       init rospy subscribers
        rospy.Subscriber("command", String, self.cmd_callback, queue_size=1)
        #rospy.Subscriber("obstacle", String, self.obstacle_callback, queue_size=1)
#       init collision avoidance
        if self.robot_name == "secondary_robot":
            self.collision_avoidance = CollisionAvoidanceSecondaryRobot()
        elif self.robot_name == "main_robot":
            self.collision_avoidance = CollisionAvoidanceMainRobot()
#       init path planner
        rospy.sleep(3)
        self.world = np.array([[self.robot_radius, self.robot_radius], [3. - self.robot_radius, self.robot_radius],
                               [3. - self.robot_radius, 2. - self.robot_radius],
                               [2.55 + self.robot_radius, 2. - self.robot_radius],
                               [2.55 + self.robot_radius, 1.543 - self.robot_radius],
                               [0.45 - self.robot_radius, 1.543 - self.robot_radius],
                               [0.45 - self.robot_radius, 2. - self.robot_radius],
                               [self.robot_radius, 2. - self.robot_radius],
                               [self.robot_radius, self.robot_radius]])
        self.publish_world(np.array([self.world]))
        self.path_planner = PathPlanning()

    def publish_world(self, world):
        markers = []
        i = 0
        for element in world:
            i += 1
            marker = Marker()
            marker.type = marker.LINE_STRIP
            marker.action = marker.ADD
            marker.ns = "world"
            marker.scale.x = 0.02
            marker.scale.y = 0.02
            marker.scale.z = 0.02
            marker.color.a = 1
            marker.color.g = 1
            marker.id = i
            marker.header.frame_id = "map"
            marker.header.stamp = rospy.Time.now()
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0
            marker.pose.position.x = 0.0
            marker.pose.position.y = 0.0
            marker.pose.position.z = 0.
            for point_ in element:
                point = Point()
                point.x = point_[0]
                point.y = point_[1]
                point.z = 0.
                marker.points.append(point)
            markers.append(marker)
            self.world_publisher.publish(markers)

    def get_polygon_from_point(self, point, radius):
        return np.array([[point[0] - radius, point[1] - radius],
                         [point[0] - radius, point[1] + radius],
                         [point[0] + radius, point[1] + radius],
                         [point[0] + radius, point[1] - radius]])

    def obstacle_callback(self, obstacle):
        obstacle_point = np.array(obstacle.data.split()).astype(float)
        self.obstacle_polygon = self.get_polygon_from_point(obstacle_point)
        self.collision_avoidance.set_collision_area(self.obstacle_polygon)


    def pub_path(self):
        path = Path()
        path.header.stamp = rospy.Time.now()
        path.header.frame_id = 'map'
        for i in self.path:
            pose = PoseStamped()
            pose.header.stamp = rospy.Time.now()
            pose.header.frame_id = 'map'
            pose.pose.position.x = i[0]
            pose.pose.position.y = i[1]
            path.poses.append(pose)
            self.path_publisher.publish(path)

    def constraint(self, prev_vel, target_vel, dt):
        k_ = np.linalg.norm((target_vel - prev_vel)/(self.acceleration_vector*dt))
        k = np.linalg.norm(target_vel / self.velocity_vector)
        if k > 1:
            target_vel /= k
        if k_ < 1:
            return target_vel
        else:
            max_acc = self.acceleration_vector*dt
            t_acc = np.linalg.norm(target_vel-prev_vel) / max_acc
            constraint_v = prev_vel + (target_vel - prev_vel)/t_acc
            return constraint_v

    def create_path_from_way_points(self):
        self.path = np.zeros((0, 3))
        for i in range(len(self.way_points) - 1):
            path = self.create_linear_path(self.way_points[i], self.way_points[i + 1])
            self.path = np.append(self.path, path, axis=0)

    def cmd_callback(self, data):

        """
        if new command comes
        should check if current command is finished - why?
        parse new command and write to self parsed args
        stop publishing in stm_command
        :param data: type of action and args. For move_arc args are goal's X, Y, THETA orientation
        :return:
        """

        self.mutex.acquire()
        rospy.loginfo("CMD CALLBACK")
        rospy.loginfo(data.data)
        rospy.loginfo(rospy.Time.now().to_sec())
        self.prev_vel = np.array([0., 0., 0.])
        if self.timer is not None:
            self.timer.shutdown()
        data_split = data.data.split()
        cmd_id = data_split[0]
        cmd_type = data_split[1]
        cmd_args = data_split[2:]
        self.prev_vel = np.zeros(3)
        self.prev_time = rospy.Time.now().to_sec()
        if cmd_type == "move_arc" or cmd_type == "move_line" or cmd_type == "trajectory_following" \
                or cmd_type == "move_forward_sensor":
            rospy.loginfo(rospy.Time.now().to_sec())
            rospy.loginfo('START')
            args = np.array(cmd_args).astype('float')
            goal = args[:3]
            goal[2] %= (2 * np.pi)
            self.goal = goal
            self.start_moving(goal, cmd_id, cmd_type)
            self.timer = rospy.Timer(rospy.Duration(1. / self.RATE), self.timer_callback)
        elif cmd_type == "move_to_point":
            args = np.array(cmd_args).astype('float')
            goal = args[:3]
            goal[2] %= (2 * np.pi)
            self.goal = goal
            self.way_points = self.path_planner.create_path(self.coords, self.goal, self.obstacle_polygon)
            # self.publish_world(np.array([world]))
            # self.collision_avoidance.set_collision_point([self.world[1, :]])
            self.way_points[:-1, 2] = self.coords[2]
            self.way_points[-1, 2] = self.goal[2]
            self.create_path_from_way_points()
            self.pub_path()
            goal = self.way_points[1, :]
            goal = self.way_points[1, :]
            self.start_moving(goal, cmd_id, cmd_type)
            self.timer = rospy.Timer(rospy.Duration(1. / self.RATE), self.timer_callback)
        elif cmd_type == "stop":
            self.terminate_moving()
        self.mutex.release()

    def start_moving(self, goal, cmd_id, cmd_type):
        rospy.loginfo("Setting a new goal:\t" + str(goal))
        rospy.loginfo("Current cmd:\t" + str(cmd_type))
        rospy.loginfo("=====================================")
        rospy.loginfo("")
        self.cmd_id = cmd_id
        self.current_cmd = cmd_type
        self.goal = goal
        self.buf_goal = self.goal.copy()
        self.update_coords()
        rospy.loginfo(rospy.Time.now().to_sec())
        self.current_state = "start"
        # rospy.loginfo(self.current_cmd)
        if self.current_cmd == "move_line":
            self.path = self.create_linear_path(self.coords, self.goal)
        elif self.current_cmd == "move_arc":
            self.path = self.create_linear_path(self.coords, self.goal)
            self.current_state = "move_arc"
        elif self.current_cmd == "move_to_point":
            self.current_state = "move_to_point"
            self.way_point_ind = 1
            self.path = self.create_linear_path(self.coords, self.way_points[self.way_point_ind, :])
        rospy.loginfo(rospy.Time.now().to_sec())
        rospy.loginfo('----------------------!!!!!------------')

    # noinspection PyUnusedLocal
    def timer_callback(self, event):
        self.calculate_current_status()
        self.move()

    def calculate_current_status(self):
        """
        Calculates X and Y distance from current location to goal
        Calculates difference in rotation between robot's and goal's orientations
        Calculates length of vector to follow and it's orientation
        Calculates remained path to go in units of angle and distance
        :return:
        """

        rospy.loginfo('---------------------------------------')
        rospy.loginfo('CURRENT STATUS')
        rospy.loginfo(self.current_state)
        if not self.update_coords():
            rospy.logwarn("NOT COORDS UPDATED")
        self.distance_map_frame, self.theta_diff = calculate_distance(self.coords, self.goal)
        self.gamma = np.arctan2(self.distance_map_frame[1], self.distance_map_frame[0])
        self.d_norm = np.linalg.norm(self.distance_map_frame)
        self.path_left = np.sqrt(self.d_norm ** 2 + self.theta_diff ** 2)

    def terminate_moving(self):
        self.timer.shutdown()
        self.set_speed(np.zeros(3))
        rospy.loginfo("Robot has stopped.")
        self.update_coords()
        rospy.loginfo("GOAL POINT %s", self.goal)
        rospy.loginfo("REACHED POINT %s", self.coords)
        answer = str(self.cmd_id) + " success"
        self.response_publisher.publish(answer)

    def create_linear_path(self, start_point, goal_point):
        x = np.linspace(start_point[0], goal_point[0], self.num_points_in_path)
        y = np.linspace(start_point[1], goal_point[1], self.num_points_in_path)
        theta = np.linspace(0, wrap_angle(goal_point[2] - start_point[2]), self.num_points_in_path)
        theta += start_point[2]
        return np.array([x, y, theta]).T

    @staticmethod
    def path_position_arg(path, point):
        path = path.copy()
        path[:, 2] = wrap_angle(path[:, 2])
        delta = path - point
        delta[2] = wrap_angle(delta[2])
        return np.argmin(np.linalg.norm(delta[:, :2], axis=1))

    def path_follower_regulator(self, point):
        p = self.path.copy()
        path_point = self.path_position_arg(p, point)
        if path_point == p.shape[0] - 1:
            delta_next_point = np.array([0., 0., 0.])
            t = 1
        else:
            delta_next_point = p[path_point + 1] - p[path_point]
            delta_next_point[2] = wrap_angle(delta_next_point[2])
            t = max(np.abs(delta_next_point / self.velocity_vector))
        rospy.loginfo("T %s", t)
        delta_path_point = p[path_point] - point
        delta_path_point[2] = wrap_angle(delta_path_point[2])
        ref_vel = delta_next_point / t
        omega_vel = self.k_linear_wp * delta_path_point[2]
        delta_path_point_dist = delta_path_point
        delta_dist = np.linalg.norm(delta_path_point_dist[:2], axis=0)
        vel = self.k_linear_vp * delta_dist
        theta = np.arctan2(delta_path_point[1], delta_path_point[0])
        self.result_vel[0] = vel * np.cos(theta)
        self.result_vel[1] = vel * np.sin(theta)
        self.result_vel[2] = omega_vel
        self.result_vel += ref_vel
        self.result_vel = cvt_global2local(np.array([self.result_vel.copy()[0], self.result_vel.copy()[1], 0]),
                                           np.array([0., 0., point[2]]))
        self.result_vel[2] = wrap_angle(omega_vel + ref_vel[2])
        curr_time = rospy.Time.now().to_sec()
        dt = curr_time - self.prev_time
        distance = max(self.d_norm, self.R_DEC * abs(self.theta_diff))
        deceleration_coefficient = self.get_deceleration_coefficient(distance)
        rospy.loginfo("Decelation coefficietn %s", deceleration_coefficient)
        self.result_vel = self.constraint(self.prev_vel.copy(), self.result_vel.copy(), dt)
        self.result_vel *= deceleration_coefficient
        self.prev_vel = self.result_vel.copy()
        self.prev_time = curr_time
        return self.result_vel

    def follow_path(self):
        self.update_coords()
        velocity = self.path_follower_regulator(self.coords)
        rospy.loginfo("Setting speed")
        rospy.loginfo(rospy.Time.now().to_sec())
        self.set_speed(velocity)

    @staticmethod
    def calculate_distance(coords1, coords2):
        distance_map_frame = coords2[:2] - coords1[:2]
        theta_diff = wrap_angle(coords2[2] - coords1[2])
        return distance_map_frame, theta_diff

    def set_speed(self, v_cmd):
        v_cmd *= self.p
        rospy.loginfo("Speed %s", v_cmd)
        cmd = str(self.cmd_id) + " 8 " + str(v_cmd[0]) + " " + str(v_cmd[1]) + " " + str(v_cmd[2])
        self.set_speed_simulation(v_cmd[0], v_cmd[1], v_cmd[2])
        self.command_publisher.publish(cmd)

    def get_deceleration_coefficient(self, distance):
        """
        Exponential function that is used to perform accurate approach to goal position
        :param distance: Euclidean distance from robot's location to goal location
        :return:
        """
        return np.e ** (-self.k / (distance + self.e_const))

    # noinspection PyPep8Naming
    def move_arc(self):
        """
        go to goal in one movement by arc path
        :return:
        """
        v = self.V_MAX
        if np.abs(self.theta_diff) < 1e-4:  # abs!!!!!!!!!!!!!
            w = 0
            rospy.loginfo("orientation is the same, start arcline move")
        else:
            R = 0.5 * self.d_norm / np.sin(self.theta_diff / 2)
            if R != 0:
                w = v / R  # must be depended on v such way so path becomes an arc
            else:
                self.terminate_moving()
        beta = wrap_angle(self.gamma - self.theta_diff / 2)
        distance = max(self.d_norm, self.R_DEC * abs(self.theta_diff))
        deceleration_coefficient = self.get_deceleration_coefficient(distance)
        rospy.loginfo("DEC coeff move_arc %s", deceleration_coefficient)

        vx = v * np.cos(beta)
        vy = v * np.sin(beta)
        vx, vy = cvt_local2global(np.array([vx, vy]), np.array([0., 0., -self.coords[2]]))
        v_cmd = np.array([vx, vy, w])
        curr_time = rospy.Time.now().to_sec()
        dt = curr_time - self.prev_time
        v_cmd = self.constraint(self.prev_vel, v_cmd, dt) * deceleration_coefficient
        self.prev_time = curr_time
        self.prev_vel = v_cmd
        self.set_speed(v_cmd)
        if self.path_left < self.XY_GOAL_TOLERANCE and self.path_left < self.YAW_GOAL_TOLERANCE:
            self.set_speed(np.zeros(3))
            self.is_robot_stopped = True

    def move(self):
        self.update_coords()
        rospy.loginfo(self.robot_name)
        rospy.loginfo(self.path[-1, :])
        delta_coords = self.coords - self.goal
        delta_coords[2] = wrap_angle(delta_coords[2])
        delta_coords[2] *= self.robot_radius
        self.delta_dist = np.linalg.norm(delta_coords[:2], axis=0)
        rospy.loginfo("COORDS %s", self.coords)
        rospy.loginfo(self.goal)
        rospy.loginfo(self.collision_avoidance)
        rospy.loginfo(type(self.collision_avoidance.get_collision_status(self.coords.copy(), self.goal.copy())))
        self.is_collision, self.p, obstacle_point = self.collision_avoidance.get_collision_status(self.coords.copy(), self.goal.copy())

        #obstacle_polygon = self.get_polygon_from_point(obstacle_point)
        # self.collision_avoidance.set_collision_area(obstacle_polygon)
        #self.is_collision = False
        rospy.loginfo(self.is_collision)
        rospy.loginfo(self.p)
        rospy.loginfo("DIST %s", self.delta_dist)
        if self.current_state == "stop":
            self.is_robot_stopped = False
            self.terminate_moving()
        elif self.is_collision:
            self.set_speed(np.zeros(3))
            self.prev_vel = np.zeros(3)
            self.goal = self.buf_goal.copy()
            self.prev_time = rospy.Time.now().to_sec()
            obstacle_polygon = self.get_polygon_from_point(obstacle_point, self.oponent_robot_radius + self.robot_radius)
            self.collision_avoidance.set_collision_area(obstacle_polygon)
            robot_polygon = self.get_polygon_from_point(self.coords.copy(), self.robot_radius)
            obstacle_polygon = get_collision_polygon(obstacle_polygon, robot_polygon)
            obstacle_polygon = list_from_polygon(obstacle_polygon)
            self.way_points = self.path_planner.create_path(self.coords.copy(), self.goal.copy(), obstacle_polygon)
            self.way_points[:-1, 2] = self.coords[2]
            self.way_points[-1, 2] = self.goal[2]
            self.create_path_from_way_points()
            self.pub_path()
            self.way_point_ind = 1
            self.goal = self.way_points[1, :]
            self.current_state = "move_to_point"
        elif self.current_state == "moving_backward":
            if self.is_robot_stopped:
                self.set_speed(np.zeros(3))
                self.goal = self.buf_goal.copy()
                rospy.loginfo("GOAL %s", self.goal)
                self.start_collision_point = self.coords
                self.path = self.create_linear_path(self.coords, self.goal)
                self.is_robot_stopped = False
                self.current_state = "following"
            elif self.delta_dist >= self.min_dist_to_goal_point:
                self.follow_path()
            else:
                self.move_arc()

        elif self.current_state == "avoid_obstacle_step1":
            if self.is_robot_stopped:
                self.set_speed(np.zeros(3))
                self.goal = self.buf_goal.copy()
                self.buf_goal = self.goal.copy()
                point = cvt_global2local(self.goal, self.start_collision_point)
                self.goal = cvt_local2global(point, self.coords)
                self.buf_goal = self.goal.copy()
                rospy.loginfo("GOAL %s", self.goal)
                self.path = self.create_linear_path(self.coords, self.goal)
                self.is_robot_stopped = False
                self.current_state = "avoid_obstacle_step2"
            elif self.delta_dist >= self.min_dist_to_goal_point:
                self.follow_path()
            else:
                self.move_arc()

        elif self.current_state == "avoid_obstacle_step2":
            if self.is_robot_stopped:
                self.set_speed(np.zeros(3))
                self.goal = self.global_goal.copy()
                self.path = self.create_linear_path(self.coords, self.goal)
                self.is_robot_stopped = False
                self.current_state = "following"
            elif self.delta_dist >= self.min_dist_to_goal_point:
                self.follow_path()
            else:
                self.move_arc()

        elif self.current_state == 'start':
            self.path = self.create_linear_path(self.coords, self.goal)
            self.prev_time = rospy.Time.now().to_sec()
            self.prev_vel = np.array([0., 0., 0.])
            self.follow_path()
            self.current_state = 'following'
        elif self.current_state == "following":
            if self.is_robot_stopped:
                self.terminate_moving()
                self.is_robot_stopped = False
            elif self.delta_dist >= self.min_dist_to_goal_point:
                self.follow_path()
            else:
                self.move_arc()

        elif self.current_state == "move_to_point":
            rospy.loginfo("GOAL POINT MOVE TO POINT %s", self.goal)
            rospy.loginfo(len(self.way_points))
            rospy.loginfo("WAY POINT IND %s", self.way_point_ind)
            # if 3 < 2:
            #     self.goal = self.way_points[-1]
            #     self.current_state = "following"
            #     self.path = self.create_linear_path(self.coords, self.goal)
            if self.is_robot_stopped:
                self. is_robot_stopped = False
                self.way_point_ind += 1
                if self.way_point_ind >= (len(self.way_points) - 1):
                    self.way_point_ind = 1
                    self.goal = self.way_points[-1]
                    self.current_state = "following"
                else:
                    self.goal = self.way_points[self.way_point_ind]
                self.path = self.create_linear_path(self.coords, self.goal)
            elif self.delta_dist >= self.min_dist_to_goal_point:
                self.follow_path()
            else:
                self.move_arc()

        elif self.current_state == "move_arc":
            if self.is_robot_stopped:
                self.terminate_moving()
                self.is_robot_stopped = False
            else:
                self.move_arc()

    def create_collision_path(self):
        self.buf_goal = self.goal.copy()
        goal = cvt_global2local(self.goal, self.coords)
        rotate_g = cvt_local2global(goal, np.array([0., 0., np.pi / 2]))
        rotate_g *= 0.6/np.linalg.norm(rotate_g)
        rotate_g = cvt_local2global(rotate_g, self.coords)
        self.goal = np.array([rotate_g[0], rotate_g[1], self.coords[2]])

    def set_speed_simulation(self, vx, vy, w):
        tw = Twist()
        twist_stamped = TwistStamped()
        tw.linear.x = vx
        tw.linear.y = vy
        tw.angular.z = w
        twist_stamped.header.stamp = rospy.Time.now()
        twist_stamped.header.frame_id = self.robot_name
        self.twist_publisher.publish(tw)
        twist_stamped.twist = tw
        self.twist_stamped_publihser.publish(twist_stamped)

    def update_coords(self):
        try:
            trans = self.tfBuffer.lookup_transform('map', self.robot_name, rospy.Time(0))
            q = [trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z,
                 trans.transform.rotation.w]
            angle = euler_from_quaternion(q)[2] % (2 * np.pi)
            self.coords = np.array([trans.transform.translation.x, trans.transform.translation.y, angle])
            return True
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as msg:
            rospy.logwarn(str(msg))
            return False


if __name__ == "__main__":
    planner = MotionPlannerNode()
    rospy.spin()
