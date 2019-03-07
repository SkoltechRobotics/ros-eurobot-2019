#!/usr/bin/env python
# coding: utf-8

# simple go to goal in different methods: odom movement

import rospy
import numpy as np
import tf2_ros
import time
from tf.transformations import euler_from_quaternion
# from geometry_msgs.msg import Twist
from std_msgs.msg import String
from threading import Lock
from geometry_msgs.msg import Twist
from core_functions import *
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped


# from std_msgs.msg import Int32MultiArray
# from core_functions import cvt_local2global
# from core_functions import wrap_angle


def wrap_angle(angle):
    """
    Wraps the given angle to the range [-pi, +pi].
    :param angle: The angle (in rad) to wrap (can be unbounded).
    :return: The wrapped angle (guaranteed to in [-pi, +pi]).
    """
    return (angle + np.pi) % (np.pi * 2) - np.pi


class MotionPlannerNode:
    def __init__(self):
        rospy.init_node("motion_planner", anonymous=True)
        rospy.Subscriber("command", String, self.cmd_callback, queue_size=1)
        rospy.Subscriber("/navigation/path", Path, self.callback_path)
        self.command_publisher = rospy.Publisher("/secondary_robot/stm_command", String, queue_size=1)
        self.response_publisher = rospy.Publisher("response", String, queue_size=10)
        self.twist_publisher = rospy.Publisher("cmd_vel", Twist, queue_size=1)
        self.path_publisher = rospy.Publisher('path', Path, queue_size=10)
        self.V_MAX = rospy.get_param("V_MAX")   # m/s
        self.W_MAX = rospy.get_param("W_MAX")
        self.AV_MAX = rospy.get_param("AV_MAX")  # m / s / s
        self.AW_MAX = rospy.get_param("AW_MAX")  # 1 / s / s
        self.R_DEC = rospy.get_param("R_DECELERATION")
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
        self.robot_name = rospy.get_param("robot_name")
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
        # in future current state is for example, follow path and stopping by rangefinders, stopping by localization
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
        self.cmd_stop_robot_id = None
        self.vx_prev = 0
        self.vy_prev = 0
        self.w_prev = 0
        self.t_prev = time.time()
        self.stop_id = 0
        self.timer = None


    # noinspection PyTypeChecker
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
        rospy.loginfo(str(target_vel-prev_vel))
        rospy.loginfo(str(k))
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
        if cmd_type == "move_arc" or cmd_type == "move_line" or cmd_type == "trajectory_following":
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
        if cmd_type == "trajectory_following":
            self.current_state = cmd_type
        else:
            self.current_state = "start"
            rospy.loginfo('----------------------!!!!!------------')

    # noinspection PyUnusedLocal
    def timer_callback(self, event):

        self.calculate_current_status()

        if self.current_cmd == "move_arc":
            self.move_arc()
        elif self.current_cmd == "move_line":
            self.move_line()
        elif self.current_cmd =='trajectory_following':
            self.current_cmd = 'trajectory_following'
            self.move_line()

    def calculate_current_status(self):
        """
        Calculates X and Y distance from current location to goal
        Calculates difference in rotation between robot's and goal's orientations
        Calculates length of vector to follow and it's orientation
        Calculates remained path to go in units of angle and distance
        :return:
        """

        # rospy.loginfo('---------------------------------------')
        rospy.loginfo('CURRENT STATUS')
        rospy.loginfo(self.current_state)
        while not self.update_coords():
            rospy.sleep(0.05)

        self.distance_map_frame, self.theta_diff = self.calculate_distance(self.coords, self.goal)
        self.gamma = np.arctan2(self.distance_map_frame[1], self.distance_map_frame[0])
        self.d_norm = np.linalg.norm(self.distance_map_frame)
        rospy.loginfo("d_norm %.3f", self.d_norm)
        rospy.loginfo("theta_diff %.3f" % self.theta_diff)

        # path_done = np.sqrt(self.d_init**2 + self.alpha_init**2) - np.sqrt(d**2 + alpha**2)
        self.path_left = np.sqrt(self.d_norm ** 2 + self.theta_diff ** 2)

    def terminate_moving(self):
        self.timer.shutdown()
        self.set_speed(np.zeros(3))
        self.set_speed_simulation(0, 0, 0)
        rospy.loginfo("Robot has stopped.")
        rospy.sleep(1.0 / 40)
        answer = str(self.cmd_id) + " success"
        self.response_publisher.publish(answer)

    def create_linear_path(self):
        x = np.linspace(self.coords[0], self.goal[0], self.num_points_in_path)
        y = np.linspace(self.coords[1], self.goal[1], self.num_points_in_path)
        theta = np.linspace(0, wrap_angle(self.goal[2] - self.coords[2]), self.num_points_in_path)
        theta += self.coords[2]
        self.path = np.array([x, y, theta]).T

    def path_position_arg(self, path, point, r=0):
        path = path.copy()
        path[:, 2] = wrap_angle(path[:, 2])
        delta = path - point
        # delta[2] = wrap_angle(delta[2])
        return np.argmin(np.linalg.norm(delta[:, :2], axis=1))

    def path_follower_regulator(self, point, r=0):
        # nearest path point to robot pose
        p = self.path.copy()
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
        self.vx_prev, self.vy_prev, self.w_prev = self.result_vel.copy()
        self.prev_time = curr_time
        return self.result_vel

    def follow_path(self):
        self.update_coords()
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

        cmd = str(self.cmd_id ) + " 8 " + str(v_cmd[0]) + " " + str(v_cmd[1]) + " " + str(v_cmd[2])
          
        rospy.loginfo("Sending cmd: " + cmd)
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
        return np.e ** (-1 / (self.k * distance + self.e_const))

    # noinspection PyPep8Naming
    def move_arc(self):
        """
        go to goal in one movement by arc path
        :return:
        """

        rospy.loginfo("NEW ARC MOVEMENT")
        rospy.loginfo("Goal:\t" + str(self.goal))

        v = self.V_MAX
        if abs(self.theta_diff) < 1e-4:  # abs!!!!!!!!!!!!!
            w = 0
            rospy.loginfo("orientation is the same, start arcline move")
        else:
            R = 0.5 * self.d_norm / np.sin(self.theta_diff / 2)
            w = v / R  # must be depended on v such way so path becomes an arc

        if abs(w) > self.W_MAX:
            k = abs(w) / self.W_MAX
            v /= k
            w /= k

        beta = wrap_angle(self.gamma - self.theta_diff / 2)

        # Deceleration when we are near the goal point
        distance = max(self.d_norm, self.R_DEC * abs(self.theta_diff))
        deceleration_coefficient = self.get_deceleration_coefficient(distance)
        v *= deceleration_coefficient
        w *= deceleration_coefficient

        vx = v * np.cos(beta)
        vy = v * np.sin(beta)
        vx, vy = self.rotation_transform(np.array([vx, vy]), -self.coords[2])
        v_cmd = np.array([vx, vy, w])
        v_cmd = self.acceleration_constraint(v_cmd)
        self.set_speed(v_cmd)
        self.set_speed_simulation(v_cmd[0], v_cmd[1], v_cmd[2])
        if self.path_left < self.XY_GOAL_TOLERANCE and self.path_left < self.YAW_GOAL_TOLERANCE:
            self.terminate_moving()

    def move_line(self):
        rospy.loginfo('==================NEW LINE MOVEMENT=====================')
        rospy.loginfo('goal is' + str(self.goal))

        if self.current_state == "stop":
            self.terminate_moving()
        elif self.current_state == 'start':
            self.update_coords()
            self.create_linear_path()
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
            rospy.loginfo("PREV_VELF")
            rospy.loginfo(str(self.prev_vel))
            delta_coords = self.coords - self.path[-1, :]
            delta_coords[2] = wrap_angle(delta_coords[2])
            delta_coords[2] *= self.r
            self.delta_dist = np.linalg.norm(delta_coords, axis=0)
            rospy.loginfo("DELTA DIST %.4f", self.delta_dist)
        elif self.current_state == 'following' and self.delta_dist < self.min_dist_to_goal_point:
            self.current_state = 'move_arc'
        elif self.current_state == 'move_arc':
            self.move_arc()

    def acceleration_constraint(self, vel_cmd):
        vx, vy, w = vel_cmd
        t = time.time()
        if np.abs(w - self.w_prev) > self.AW_MAX * (t - self.t_prev):
            w_new = self.w_prev + self.AW_MAX * (t - self.t_prev) * np.sign(w - self.w_prev)
            if abs(w) > 5 * self.AW_MAX * (t - self.t_prev):
                vx *= w_new / w
                vy *= w_new / w
            w = w_new

        if np.abs(vx - self.vx_prev) > self.AV_MAX * (t - self.t_prev):
            vx = self.vx_prev + self.AV_MAX * (t - self.t_prev) * np.sign(vx - self.vx_prev)

        if np.abs(vy - self.vy_prev) > self.AV_MAX * (t - self.t_prev):
            vy = self.vy_prev + self.AV_MAX * (t - self.t_prev) * np.sign(vy - self.vy_prev)

        if abs(vx) > self.V_MAX:
            k = abs(vx) / self.V_MAX
            w /= k
            vx /= k
            vy /= k

        if abs(vy) > self.V_MAX:
            k = abs(vy) / self.V_MAX
            w /= k
            vx /= k
            vy /= k

        if abs(w) > self.W_MAX:
            k = abs(w) / self.W_MAX
            vx /= k
            vy /= k
            w /= k

        self.vy_prev = vy
        self.vx_prev = vx
        self.w_prev = w
        self.t_prev = t
        return np.array([vx, vy, w])

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
            rospy.loginfo("Robot coords:\t" + str(self.coords))
            return True
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as msg:
            rospy.logwarn(str(msg))
            rospy.logwarn("SMT WROND QQ")
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
