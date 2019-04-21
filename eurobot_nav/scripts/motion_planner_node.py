#!/usr/bin/env python
# coding: utf-8

import rospy
import numpy as np
import tf2_ros
from tf.transformations import euler_from_quaternion
from visualization_msgs.msg import MarkerArray, Marker
# from geometry_msgs.msg import Twist
from std_msgs.msg import String
from threading import Lock
from geometry_msgs.msg import Twist
from core_functions import cvt_global2local, cvt_local2global, wrap_angle, calculate_distance, cvt_ros_scan2points
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from collision_avoidance import CollisionAvoidance


class MotionPlannerNode:
    def __init__(self):
        self.robot_name = rospy.get_param("robot_name")
        rospy.init_node("motion_planner", anonymous=True)
#       init motion planner params
        self.V_MAX = rospy.get_param("V_MAX")   # m/s
        self.num_points_in_path = rospy.get_param("num_points_in_path")
        self.R_DEC = rospy.get_param("R_DECELERATION")
        self.k = rospy.get_param("k")
        self.min_dist_to_goal_point = rospy.get_param("min_dist_to_goal_point")
        self.e_const = rospy.get_param("exp_const")
        self.velocity_vector = np.array(rospy.get_param("velocity_vector"))
        self.acceleration_vector = self.velocity_vector * 2 * self.velocity_vector[2]
        self.k_linear_wp = rospy.get_param("k_linear_wp")
        self.k_linear_vp = rospy.get_param("k_linear_wp")
        self.RATE = rospy.get_param("RATE")
        self.XY_GOAL_TOLERANCE = rospy.get_param("XY_GOAL_TOLERANCE")
        self.YAW_GOAL_TOLERANCE = rospy.get_param("YAW_GOAL_TOLERANCE")
        self.r = rospy.get_param("robot_radius")
#       set values to default
        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer)
        self.is_collision = False
        self.mutex = Lock()
        self.path = np.array([])
        self.buf_path = self.path
        self.global_goal = None
        self.prev_vel = np.array([0., 0., 0.])
        self.result_vel = np.array([0., 0., 0.])
        self.cmd_id = None
        self.cmd_type = None
        self.cmd_args = None
        self.t_prev = None
        self.goal = None
        self.current_cmd = None
        self.current_state = 'start'
        self.delta_dist = 0
        self.vel = np.zeros(3)
        self.theta_diff = None
        self.d_norm = None
        self.gamma = None
        self.coords = None
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
        self.command_publisher = rospy.Publisher("/%s/stm/command"%self.robot_name, String, queue_size=1)
        self.response_publisher = rospy.Publisher("response", String, queue_size=10)
        self.point_publisher = rospy.Publisher("obstacle", MarkerArray, queue_size=10)
        self.twist_publisher = rospy.Publisher("cmd_vel", Twist, queue_size=1)
        self.path_publisher = rospy.Publisher('path', Path, queue_size=10)
#       init rospy subscribers
        rospy.Subscriber("command", String, self.cmd_callback, queue_size=1)
#       init collision avoidance
        self.collision_avoidance = CollisionAvoidance()


    # noinspection PyTypeChecker
    def set_collision_point(self, positions):
        marker = []
        for i, position in enumerate(positions):
            point = Marker()
            point.header.stamp = rospy.Time.now()
            point.header.frame_id = "map"
            point.id = i
            point.type = 1
            point.ns = "obstacle"
            #
            #rospy.loginfo(position)
            point.pose.position.x = position[0]
            point.pose.position.y = position[1]
            point.pose.position.z = 0
            point.pose.orientation.w = 1
            point.scale.x = 0.1
            point.scale.y = 0.1
            point.scale.z = 0.1
            point.color.a = 1
            point.color.r = 1
            point.lifetime = rospy.Duration(0.7)
            marker.append(point)
        self.point_publisher.publish(marker)

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
        # when new cmd arrives shutdown running timer
        if self.timer is not None:
            self.timer.shutdown()
        # parse name,type
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
            rospy.loginfo(rospy.Time.now().to_sec())
            rospy.loginfo("TIME")
            rospy.loginfo(rospy.Time.now().to_sec())
            self.timer = rospy.Timer(rospy.Duration(1.0 / self.RATE), self.timer_callback)
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
        self.update_coords()
        rospy.loginfo(rospy.Time.now().to_sec())
        self.current_state = "start"
        # rospy.loginfo(self.current_cmd)
        if self.current_cmd == "move_line":
            self.create_linear_path()
        elif self.current_cmd == "move_arc":
            self.current_state = "move_arc"
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

    def create_linear_path(self):
        x = np.linspace(self.coords[0], self.goal[0], self.num_points_in_path)
        y = np.linspace(self.coords[1], self.goal[1], self.num_points_in_path)
        theta = np.linspace(0, wrap_angle(self.goal[2] - self.coords[2]), self.num_points_in_path)
        theta += self.coords[2]
        self.path = np.array([x, y, theta]).T
        rospy.loginfo("PATH")
        rospy.loginfo(rospy.Time.now().to_sec())

    @staticmethod
    def path_position_arg(path, point):
        path = path.copy()
        path[:, 2] = wrap_angle(path[:, 2])
        delta = path - point
        delta[2] = wrap_angle(delta[2])
        return np.argmin(np.linalg.norm(delta[:, :2], axis=1))

    def path_follower_regulator(self, point, r=0):
        p = self.path.copy()
        path_point = self.path_position_arg(p, point)
        if path_point == p.shape[0] - 1:
            delta_next_point = np.array([0., 0., 0.])
            t = 1
        else:
            delta_next_point = p[path_point + 1] - p[path_point]
            delta_next_point[2] = wrap_angle(delta_next_point[2])
            t = max(np.abs(delta_next_point / self.velocity_vector))
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
        delta_coords = self.coords - self.path[-1, :]
        delta_coords[2] = wrap_angle(delta_coords[2])
        delta_coords[2] *= self.r
        self.delta_dist = np.linalg.norm(delta_coords[:2], axis=0)
        #self.create_linear_path()
        self.is_collision, self.p = self.collision_avoidance.get_collision_status(self.coords.copy(), self.goal.copy())
        rospy.loginfo(self.is_collision)
        rospy.loginfo(self.p)
        rospy.loginfo("DIST %s", self.delta_dist)
        if self.current_state == "stop":
            self.is_robot_stopped = False
            self.terminate_moving()

        elif self.is_collision:
            self.set_speed(np.zeros(3))
            self.prev_vel = np.zeros(3)
            self.prev_time = rospy.Time.now().to_sec()
            self.create_linear_path()

        elif self.current_state == "moving_backward":
            if self.is_robot_stopped:
                self.set_speed(np.zeros(3))
                self.goal = self.buf_goal.copy()
                rospy.loginfo("GOAL %s", self.goal)
                self.start_collision_point = self.coords
                self.create_linear_path()
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
                self.create_linear_path()
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
                self.create_linear_path()
                self.is_robot_stopped = False
                self.current_state = "following"
            elif self.delta_dist >= self.min_dist_to_goal_point:
                self.follow_path()
            else:
                self.move_arc()

        elif self.current_state == 'start':
            self.create_linear_path()
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
        elif self.current_state == "move_arc":
            if self.is_robot_stopped:       
                self.terminate_moving()
                self.is_robot_stopped = False
            else:
               self. move_arc()

    def create_collision_path(self):
        self.buf_goal = self.goal.copy()
        goal = cvt_global2local(self.goal, self.coords)
        rotate_g = cvt_local2global(goal, np.array([0., 0., np.pi / 2]))
        rotate_g *= 0.6/np.linalg.norm(rotate_g)
        rotate_g = cvt_local2global(rotate_g, self.coords)
        self.goal = np.array([rotate_g[0], rotate_g[1], self.coords[2]])

    def set_speed_simulation(self, vx, vy, w):
        tw = Twist()
        tw.linear.x = vx
        tw.linear.y = vy
        tw.angular.z = w
        self.twist_publisher.publish(tw)

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

