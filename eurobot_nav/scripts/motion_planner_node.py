#!/usr/bin/env python
# coding: utf-8

# simple go to goal in different methods: odom movement

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
from geometry_msgs.msg import PoseStamped, Point
from sensor_msgs.msg import LaserScan


LIDAR_DELTA_ANGLE = (np.pi / 180) / 4
LIDAR_START_ANGLE = -(np.pi / 2 + np.pi / 4)
class MotionPlannerNode:
    def __init__(self):
        self.robot_name = rospy.get_param("robot_name")
        rospy.init_node("motion_planner", anonymous=True)
        rospy.Subscriber("command", String, self.cmd_callback, queue_size=1)
        rospy.Subscriber("/navigation/path", Path, self.callback_path)
        self.collision_area_publisher = rospy.Publisher("collision_area", Marker, queue_size=10)
        rospy.Subscriber("/secondary_robot/scan", LaserScan, self.scan_callback, queue_size=1)
        rospy.Subscriber("/obstacle_point", MarkerArray, self.obstacle_callback, queue_size=1)
        #rospy.Subscriber("/%s/stm/proximity_status"%self.robot_name, String, self.distance_callback, queue_size=10)
        self.command_publisher = rospy.Publisher("/%s/stm/command"%self.robot_name, String, queue_size=1)
        self.response_publisher = rospy.Publisher("response", String, queue_size=10)
        self.point_publisher = rospy.Publisher("obstacle", MarkerArray, queue_size=10)
        self.twist_publisher = rospy.Publisher("cmd_vel", Twist, queue_size=1)
        self.path_publisher = rospy.Publisher('path', Path, queue_size=10)
        self.V_MAX = rospy.get_param("V_MAX")   # m/s
        self.W_MAX = rospy.get_param("W_MAX")
        self.AV_MAX = rospy.get_param("AV_MAX")  # m / s / s
        self.AW_MAX = rospy.get_param("AW_MAX")  # 1 / s / s
        self.R_DEC = rospy.get_param("R_DECELERATION")
        self.min_dist_to_obstacle = rospy.get_param("min_dist_to_obstacle")
        self.k = rospy.get_param("k")
        self.min_dist_to_goal_point = rospy.get_param("min_dist_to_goal_point")
        self.e_const = rospy.get_param("exp_const")
        self.velocity_vector = np.array(rospy.get_param("velocity_vector"))
        self.k_linear_wp = rospy.get_param("k_linear_wp")
        self.k_linear_vp = rospy.get_param("k_linear_wp")
        self.RATE = rospy.get_param("RATE")
        self.prev_time = rospy.Time.now().to_sec()
        self.XY_GOAL_TOLERANCE = rospy.get_param("XY_GOAL_TOLERANCE")
        self.YAW_GOAL_TOLERANCE = rospy.get_param("YAW_GOAL_TOLERANCE")
        self.r = rospy.get_param("robot_radius")
        self.num_points_in_path = rospy.get_param("num_points_in_path")
        self.acceleration_vector = self.velocity_vector * 2 * self.velocity_vector[2]
        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer)
        self.mutex = Lock()
        self.path = np.array([])
        self.buf_path = self.path
        self.prev_vel = np.array([0., 0., 0.])
        self.result_vel = np.array([0., 0., 0.])
        self.cmd_id = None
        self.cmd_type = None
        self.cmd_args = None
        self.t_prev = None
        self.goal = None
        self.current_cmd = None
        self.current_state = 'start'
        self.collision_area = None
        self.delta_dist = 0
        self.vel = np.zeros(3)
        self.theta_diff = None
        self.d_norm = None
        self.gamma = None
        self.coords = None
        self.path = np.array([[]])
        self.point = np.array([0., 0., 0.])
        self.path_left = 9999  # big initial value
        self.distance_map_frame = None
        self.stop_id = 0
        self.obstacle_points = np.array([100, 100])
        self.dist_to_obstacle = 100
        self.timer = None
        self.buf_goal= None
        self.is_robot_stopped = None
        #self.sensors_coords_vector = np.array([, [-1, 0], [np.sin(np.pi/6), np.cos(np.pi/6)]])
        self.sensors_coords_vector = np.array([[-1, 0], [np.sin(np.pi/6), np.cos(np.pi/6)]  ,[np.sin(np.pi/6), -np.cos(np.pi/6)]])
        self.sensor_ind = 0
        self.sensor_angles = wrap_angle(np.array([np.pi/6, np.pi*(2/3), np.pi*(4/3)]))
        self.distances = np.array([100., 100., 100.])
        self.min_sin = 0.3
        #self.robot_name = "secondary_robot"
        self.max_dist = 0.5
        self.p = 1


    def get_landmarks(self, scan):
        """Returns filtrated lidar data"""
        ranges = np.array(scan.ranges)
        ind = self.filter_scan(scan)
        final_ind = np.where((np.arange(ranges.shape[0]) * ind) > 0)[0]
        angles = (LIDAR_DELTA_ANGLE * final_ind + LIDAR_START_ANGLE) % (2 * np.pi)
        distances = ranges[final_ind]
        return angles, distances

    def scan_callback(self, scan):
        self.scan = scan
        self.update_coords()
        self.scan_stamp = scan.header.stamp
        angles, distances = self.get_landmarks(self.scan)
        x = distances * np.cos(angles)
        y = distances * np.sin(angles)
        if x.shape[0] > 0:
            landmarks = (np.array([x, y])).T
            rospy.loginfo("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!1")
            rospy.loginfo(landmarks)
            landmarks = cvt_local2global(landmarks, self.coords)
            self.obstacle_point = landmarks[np.argmin(np.linalg.norm(self.coords[:2] - landmarks))]
            self.set_collision_point(self.obstacle_point)
        else:
            self.obstacle_point = np.array([100, 100])

    def filter_scan(self, scan):
        ranges = np.array(scan.ranges)
        intensities = np.array(scan.intensities)
        cloud = cvt_ros_scan2points(scan)
        index0 = ranges < self.max_dist
        #index1 = self.alpha_filter(cloud, self.min_sin)
        index = index0 #* index1
        return np.where(index, ranges, 0)

    def set_collision_area(self, points):
        marker = Marker()
        marker.type = marker.LINE_STRIP
        marker.action = marker.ADD
        marker.scale.x = 0.05
        marker.scale.y = 0.05
        marker.scale.z = 0.05
        marker.color.a = 1
        marker.color.g = 1
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.1
        for i, position in enumerate(points):
            point = Point()
            point.x = position[0]
            point.y = position[1]
            point.z = 0.2
            marker.points.append(point)
        self.collision_area_publisher.publish(marker)

    def is_inside_collision_area(self, point):
        if ((point[0] < self.collision_area[0,0] and point[0] > self.collision_area[2, 0]) \
                or (point[0] > self.collision_area[0,0] and point[0] < self.collision_area[2, 0]))\
                and ((point[1] < self.collision_area[0,1] and self.point[1] > self.collision_area[2, 1]) \
                or (point[1] > self.collision_area[0,1] and point[1] < self.collision_area[2, 1])):
            return True
        else:
            return False

    def get_collision_area(self):
        goal = cvt_global2local(self.goal, self.coords)
        dist_to_goal = np.linalg.norm(self.goal[:2] - self.coords[:2])
        points = np.array([[0, -0.2], [dist_to_goal, -0.2], [dist_to_goal, 0.2], [0, 0.2], [0, -0.2]])
        # points = cvt_local2global(points, self.coords)
        self.collision_area = cvt_local2global(points, np.array(
            [self.coords[0], self.coords[1], self.coords[2] + wrap_angle(np.arctan2(goal[1], goal[0]))]))
        # rospy.loginfo(points)
        self.set_collision_area(self.collision_area)

    @staticmethod
    def alpha_filter(cloud, min_sin_angle):
        x, y = cloud.T
        x0, y0 = 0, 0
        x1, y1 = np.roll(x, 1), np.roll(y, 1)
        cos_angle = ((x - x0) * (x - x1) + (y - y0) * (y - y1)) / (np.sqrt((x - x0) * (x - x0) + (y - y0) * (y - y0))
                                                                   * np.sqrt((x - x1) * (x - x1) + (y - y1) * (y - y1)))
        sin_angle = np.sqrt(1 - cos_angle * cos_angle)
        index = sin_angle >= min_sin_angle
        return index
    # noinspection PyTypeChecker
    def set_collision_point(self, positions):
        marker = []
        #for i, position in enumerate(positions):
        point = Marker()
        point.header.stamp = rospy.Time.now()
        point.header.frame_id = "map"
        point.id = 1
        point.type = 1
        point.ns = "obstacle"
        rospy.loginfo(positions)
        point.pose.position.x = positions[0]
        point.pose.position.y = positions[1]
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

    def cvt_distances2points(self):
        self.update_coords()
        point_local = ((self.sensors_coords_vector.T) * (self.distances + self.r)).T
        # self.obstacle_points = cvt_local2global(point_local, self.coords)
        self.set_collision_point(cvt_local2global(point_local, self.coords))

    def distance_callback(self, data):
        self.distances = np.array(data.data.split()).astype(float)/100
        #rospy.loginfo(self.distances)
        #self.cvt_distances2points()

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

    def obstacle_callback(self, data):
        self.update_coords()
        self.obstacle_points = np.array([data.markers[0].pose.position.x, data.markers[0].pose.position.y])
        self.dist_to_obstacle = np.linalg.norm(self.coords[:2] - self.obstacle_points)

    def constraint(self, prev_vel, target_vel, dt):
        k_ = np.linalg.norm((target_vel - prev_vel)/(self.acceleration_vector*dt))
        k = np.linalg.norm(target_vel / self.velocity_vector)
        if k > 1:
            target_vel /= k
        #rospy.loginfo(str(target_vel-prev_vel))
        #rospy.loginfo(str(k))
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
        self.prev_vel = np.array([0., 0., 0.])
        rospy.loginfo("")
        rospy.loginfo("=====================================")
        rospy.loginfo("NEW CMD:\t" + str(data.data))

        # when new cmd arrives shutdown running timer
        if self.timer is not None:
            self.timer.shutdown()

        # parse name,type
        data_split = data.data.split()
        cmd_id = data_split[0]
        cmd_type = data_split[1]
        cmd_args = data_split[2:]
        rospy.loginfo(cmd_type)
        if cmd_type == "move_arc" or cmd_type == "move_line" or cmd_type == "trajectory_following" \
                or cmd_type == "move_forward_sensor":
            rospy.loginfo('START')
            args = np.array(cmd_args).astype('float')
            goal = args[:3]
            goal[2] %= (2 * np.pi)
            self.goal = goal
            self.start_moving(goal, cmd_id, cmd_type)
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
        if cmd_type == "trajectory_following":
            self.current_state = cmd_type
        else:
            self.current_state = "start"
            # rospy.loginfo(self.current_cmd)
            if self.current_cmd == "move_line":
                self.create_linear_path()
            elif self.current_cmd == "move_forward_sensor":
                self.create_linear_path_forward_sensor()
            elif self.current_cmd == "move_arc":
                self.current_state = "move_arc"
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

        # rospy.loginfo('---------------------------------------')
        #rospy.loginfo('CURRENT STATUS')
        #rospy.loginfo(self.current_state)
        while not self.update_coords():
            rospy.sleep(0.05)

        self.distance_map_frame, self.theta_diff = calculate_distance(self.coords, self.goal)
        self.gamma = np.arctan2(self.distance_map_frame[1], self.distance_map_frame[0])
        self.d_norm = np.linalg.norm(self.distance_map_frame)
        #rospy.loginfo("d_norm %.3f", self.d_norm)
        #rospy.loginfo("theta_diff %.3f" % self.theta_diff)

        # path_done = np.sqrt(self.d_init**2 + self.alpha_init**2) - np.sqrt(d**2 + alpha**2)
        self.path_left = np.sqrt(self.d_norm ** 2 + self.theta_diff ** 2)

    def terminate_moving(self):
        self.timer.shutdown()
        self.set_speed(np.zeros(3))
        self.set_speed_simulation(0, 0, 0)
        rospy.loginfo("Robot has stopped.")
        rospy.loginfo(str(self.dist_to_obstacle))
        rospy.sleep(1.0 / 40)
        answer = str(self.cmd_id) + " success"
        self.response_publisher.publish(answer)

    def create_linear_path(self):
        x = np.linspace(self.coords[0], self.goal[0], self.num_points_in_path)
        y = np.linspace(self.coords[1], self.goal[1], self.num_points_in_path)
        theta = np.linspace(0, wrap_angle(self.goal[2] - self.coords[2]), self.num_points_in_path)
        theta += self.coords[2]
        self.path = np.array([x, y, theta]).T

    def create_linear_path_forward_sensor(self):
        self.update_coords()
        goal = cvt_global2local(self.goal, self.coords)
        rospy.loginfo(str(goal))
        x = np.linspace(0., goal[0], self.num_points_in_path)
        y = np.linspace(0., goal[1], self.num_points_in_path)
        angles = np.arctan2(self.sensors_coords_vector[:, 1], self.sensors_coords_vector[:, 0])
        trajectory_angle = np.arctan2(goal[1], goal[0])
        rospy.loginfo("ANGLES %s", angles)
        rospy.loginfo("TRAJECTORY ANGLE %s", trajectory_angle)
        rospy.loginfo(np.linalg.norm(self.sensors_coords_vector - goal[:2], axis=1))
        self.sensor_ind = np.argmin(np.linalg.norm(self.sensors_coords_vector - goal[:2], axis=1))
        rospy.loginfo(self.sensor_ind)
        rospy.loginfo(self.sensors_coords_vector[self.sensor_ind])
        angle = angles[self.sensor_ind]
        angle += trajectory_angle
        rospy.loginfo("angle %s", angle)
        theta = np.ones(self.num_points_in_path) * wrap_angle(angle)
        theta[-1] = goal[2]
        self.path = cvt_local2global(np.array([x, y, theta]).T, self.coords)
        rospy.loginfo(self.path)

    def path_position_arg(self, path, point, r=0):
        path = path.copy()
        path[:, 2] = wrap_angle(path[:, 2])
        delta = path - point
        delta[2] = wrap_angle(delta[2])
        return np.argmin(np.linalg.norm(delta[:, :2], axis=1))

    def path_follower_regulator(self, point, r=0):
        # nearest path point to robot pose
        p = self.path.copy()
        #rospy.loginfo("!!!!!!!!!!!!!!!!!!!!!!!!! %s", self.delta_dist)
        path_point = self.path_position_arg(p, point, r)
        # delta coords to next path point
        if path_point == p.shape[0] - 1:
            delta_next_point = np.array([0., 0., 0.])
            t = 1
        else:
            delta_next_point = p[path_point + 1] - p[path_point]
            delta_next_point[2] = wrap_angle(delta_next_point[2])
            # t = max(np.abs(delta_next_point[0] / self.V_MAX), np.abs(delta_next_point[1] / self.V_MAX),
            #         np.abs(delta_next_point[2] / self.W_MAX))
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
        # try:
            # self.cvt_distances2points()
        # except:
        #     pass
        velocity = self.path_follower_regulator(self.coords)
        # self.prev_vel = self.result_vel
        self.set_speed(velocity)
        self.set_speed_simulation(velocity[0], velocity[1], velocity[2])

    @staticmethod
    def calculate_distance(coords1, coords2):
        distance_map_frame = coords2[:2] - coords1[:2]
        theta_diff = wrap_angle(coords2[2] - coords1[2])
        return distance_map_frame, theta_diff

    # TODO local to global
    # noinspection PyPep8Naming
    @staticmethod
    def rotation_transform(vec, angle):
        # counterclockwise rotation
        M = np.array([[np.cos(angle), -np.sin(angle)],
                      [np.sin(angle), np.cos(angle)]])
        ans = vec.copy()
        ans[:2] = np.matmul(M, ans[:2].reshape((2, 1))).reshape(2)
        return ans

    def set_speed(self, v_cmd):
        # vx, vy, w = v_cmd

        # TODO
        # vx, vy = self.rotation_transform(np.array([vx, vy]), -self.coords[2])
        v_cmd *= self.p
        cmd = str(self.cmd_id) +  " 8 " + str(v_cmd[0]) + " " + str(v_cmd[1]) + " " + str(v_cmd[2])
        #rospy.loginfo("Sending cmd: " + cmd)
        self.command_publisher.publish(cmd)

    # def get_optimal_velocity(self):
    #     v = max(self.V_MIN, min(self.Kp * self.d_norm, self.V_MAX))
    #
    #     return v

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
        try:
            self.cvt_distances2points()
        except:
            pass
        rospy.loginfo("NEW ARC MOVEMENT")
        rospy.loginfo("Goal:\t" + str(self.goal))

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
        # Deceleration when we are near the goal point
        distance = max(self.d_norm, self.R_DEC * abs(self.theta_diff))
        deceleration_coefficient = self.get_deceleration_coefficient(distance)
        rospy.loginfo("DEC COEFF + %.4f", deceleration_coefficient)
        vx = v * np.cos(beta)
        vy = v * np.sin(beta)
        vx, vy = self.rotation_transform(np.array([vx, vy]), -self.coords[2])
        v_cmd = np.array([vx, vy, w])
        curr_time = rospy.Time.now().to_sec()
        dt = curr_time - self.prev_time
        v_cmd = self.constraint(self.prev_vel, v_cmd, dt) * deceleration_coefficient
        self.prev_time = curr_time
        self.prev_vel = v_cmd
        #v_cmd = self.acceleration_constraint(v_cmd)
        self.set_speed(v_cmd)
        self.set_speed_simulation(v_cmd[0], v_cmd[1], v_cmd[2])
        if self.path_left < self.XY_GOAL_TOLERANCE and self.path_left < self.YAW_GOAL_TOLERANCE:
            self.is_robot_stopped = True

    def move(self):
       #rospy.loginfo('==================NEW LINE MOVEMENT=====================')
        #rospy.loginfo('goal is' + str(self.goal))

        if self.current_state == "stop":
            self.terminate_moving()
        elif self.get_collision_status() and self.current_state != "collision_avoidance":
            self.current_state = "collision_avoidance"
            rospy.loginfo("COLLISION")
            #self.terminate_moving()
            self.set_speed(np.zeros(3))
            self.result_vel[2] = 0
            self.result_vel /= 2
            #self.dist_to_obstacle = 0.5
            self.set_speed_simulation(0., 0., 0.)
        elif self.current_state == 'start':
            self.update_coords()
            rospy.loginfo("Create_path")
            # self.create_linear_path()
            self.pub_path()
            self.prev_time = rospy.Time.now().to_sec()
            self.prev_vel = np.array([0., 0., 0.])
            self.follow_path()
            delta_coords = self.coords - self.path[-1, :1]
            delta_coords[2] = wrap_angle(delta_coords[2])
            delta_coords[2] *= self.r
            self.delta_dist = np.linalg.norm(delta_coords, axis=0)
            self.current_state = 'following'
        elif self.current_state == 'trajectory_following':
            self.update_coords()
            self.path = self.buf_path.copy()
            self.pub_path()
            self.follow_path()
            rospy.loginfo("START_TRAJECTORY_FOLLOWING")
            delta_coords = self.coords - self.path[-1, :]
            delta_coords[2] = wrap_angle(delta_coords[2])
            delta_coords[2] *= self.r
            self.delta_dist = np.linalg.norm(delta_coords, axis=0)
            self.goal = np.array([self.path[-1, 0], self.path[-1, 1], self.path[-1, 2]])
            self.current_state = 'following'
        elif self.current_state == 'following' and self.delta_dist >= self.min_dist_to_goal_point:
            self.update_coords()
            self.follow_path()
            #rospy.loginfo("PREV_VELF")
            #rospy.loginfo(str(self.prev_vel))
            delta_coords = self.coords - self.path[-1, :]
            delta_coords[2] = wrap_angle(delta_coords[2])
            delta_coords[2] *= self.r
            self.delta_dist = np.linalg.norm(delta_coords[:2], axis=0)
            #rospy.loginfo("DELTA DIST %.4f", self.delta_dist)
        elif self.current_state == "following" and self.is_robot_stopped:
            self.is_robot_stopped = False
            self.current_state = "Stop"
            self.terminate_moving()
        elif self.current_state == 'following' and self.delta_dist < self.min_dist_to_goal_point:
            self.move_arc()
        elif self.current_state == 'move_arc':
            self.move_arc()
        elif self.current_state == "collision_avoidance" and self.get_collision_status():
            self.update_coords()
            rospy.loginfo("COLLISION")
            rospy.loginfo(str(self.result_vel))
            self.set_speed((-1) * self.result_vel)
            self.set_speed_simulation(-1*self.result_vel[0], -1*self.result_vel[1], -1*self.result_vel[2])
        elif self.current_state == "collision_avoidance" and not self.get_collision_status():
            self.current_state = "safe_distance"
            #self.create_collision_path()
            self.prev_vel = np.zeros(3)
            self.prev_time = rospy.Time.now().to_sec()
            self.set_speed(np.zeros(3))
            self.set_speed_simulation(0., 0., 0.)
        elif self.current_state == "safe_distance":
            self.update_coords()
            self.current_state = "following_collision_point"
            self.create_collision_path()
            self.prev_vel = np.zeros(3)
            self.prev_time = rospy.Time.now().to_sec()
            self.set_speed(np.zeros(3))
            self.set_speed_simulation(0., 0., 0.)
            rospy.loginfo(str(self.dist_to_obstacle))
            self.update_coords()
            self.follow_path()
            #rospy.loginfo("PREV_VELF")
            #rospy.loginfo(str(self.prev_vel))
            delta_coords = self.coords - self.path[-1, :]
            delta_coords[2] = wrap_angle(delta_coords[2])
            delta_coords[2] *= self.r
            self.delta_dist = np.linalg.norm(delta_coords[:2], axis=0)
        elif self.current_state == "following_collision_point" and self.delta_dist > self.min_dist_to_goal_point:
            self.update_coords()
            self.follow_path()
            #rospy.loginfo("PREV_VELF")
            #rospy.loginfo(str(self.prev_vel))
            delta_coords = self.coords - self.path[-1, :]
            delta_coords[2] = wrap_angle(delta_coords[2])
            delta_coords[2] *= self.r
            self.delta_dist = np.linalg.norm(delta_coords[:2], axis=0)
        elif self.current_state == "following_collision_point" and self.is_robot_stopped:
            self.set_speed(np.zeros(3))
            self.is_robot_stopped = False
            self.update_coords()
            self.goal = self.buf_goal.copy()
            self.create_linear_path()
            self.current_state = "following"
        elif self.current_state == "following_collision_point" and self.delta_dist <= self.min_dist_to_goal_point:
            self.move_arc()
        elif self.current_state == "safe_distance":
            rospy.loginfo(str(self.dist_to_obstacle))
            self.current_state = "following"

    def get_collision_status(self):
        rospy.loginfo("!!!!!!!COLLISION!!!!!!!")
        rospy.loginfo(self.distances)
        rospy.loginfo(self.sensor_ind)
        self.p = 1
        if self.is_inside_collision_area(self.obstacle_point):
            if (np.linalg.norm(self.coords[:2] - self.obstacle_point) < 0.5):
                rospy.loginfo("TRUE COLLISION")
                return True
            else:
                self.p = np.linalg.norm(self.coords[:2] - self.obstacle_point) / 0.7
                self.p *= 0.5
                if self.p > 1:
                    self.p = 1
        return False

    def create_collision_path(self):
        self.buf_goal = self.goal.copy()
        goal = cvt_global2local(self.goal, self.coords)
        rotate_g = cvt_local2global(goal, np.array([0., 0., np.pi / 2]))
        rotate_g *= 1/np.linalg.norm(rotate_g)
        rotate_g = cvt_local2global(rotate_g, self.coords)
        self.goal = np.array([rotate_g[0], rotate_g[1], self.coords[2]])
        self.create_linear_path()
        self.set_collision_point(rotate_g)
        self.pub_path()

    def set_speed_simulation(self, vx, vy, w):
        tw = Twist()
        tw.linear.x = vx
        tw.linear.y = vy
        tw.angular.z = w
        self.twist_publisher.publish(tw)

    def update_coords(self):
        try:
            trans = self.tfBuffer.lookup_transform('map', self.robot_name, rospy.Time())
            q = [trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z,
                 trans.transform.rotation.w]
            angle = euler_from_quaternion(q)[2] % (2 * np.pi)
            self.coords = np.array([trans.transform.translation.x, trans.transform.translation.y, angle])
            self.get_collision_area()
            rospy.loginfo("Robot coords:\t" + str(self.coords))
            return True
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as msg:
            rospy.logwarn(str(msg))
            return False

    def callback_path(self, data):
        self.buf_path = np.zeros((len(data.poses), 3))
        for i in range(len(data.poses)):
            self.point[0] = data.poses[i].pose.position.x
            self.point[1] = data.poses[i].pose.position.y
            q = [data.poses[i].pose.orientation.x,
                 data.poses[i].pose.orientation.y,
                 data.poses[i].pose.orientation.z,
                 data.poses[i].pose.orientation.w]
            self.point[2] = euler_from_quaternion(q)[2] % (2 * np.pi)
            self.buf_path[i] = self.point
        self.update_coords()


if __name__ == "__main__":
    planner = MotionPlannerNode()
    rospy.spin()
