#!/usr/bin/env python
# coding: utf-8

# simple go to goal in different methods: odom movement

import rospy
import numpy as np
import tf2_ros
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from threading import Lock
from std_msgs.msg import Int32MultiArray
from core_functions import cvt_local2global
from core_functions import wrap_angle


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

        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer)

        self.robot_name = "secondary_robot"

        self.mutex = Lock()

        self.cmd_id = None
        self.cmd_type = None
        self.cmd_args = None
        self.t_prev = None
        self.goal = None
        self.current_cmd = None
        self.current_state = None
        # in future current state is for example, follow path and stopping by rangefinders, stopping by localization

        self.RATE = 10

        self.XY_GOAL_TOLERANCE = 0.1
        self.YAW_GOAL_TOLERANCE = 0.1

        self.vel = np.zeros(3)

        self.path_left = 9999  # big initial value
        self.distance_map_frame = None

        self.cmd_stop_robot_id = None
        self.stop_id = 0

        self.V_MAX = 0.2  # m/s
        self.V_MIN = 0.15
        self.W_MAX = 0.3

        self.Kp = 5

        # self.pub_twist = rospy.Publisher("cmd_vel", Twist, queue_size=1)
        self.pub_cmd = rospy.Publisher("/secondary_robot/stm_command", String, queue_size=1)

        rospy.Subscriber("move_command", String, self.cmd_callback, queue_size=1)

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

        if cmd_type == "move_arc" or cmd_type == "move_line":
            args = np.array(cmd_args).astype('float')
            goal = args[:3]
            goal[2] %= (2 * np.pi)
            self.start_moving(goal, cmd_id, cmd_type)
            self.timer = rospy.Timer(rospy.Duration(1.0 / self.RATE), self.timer_callback)

        elif cmd_type == "stop":
            self.terminate_moving()

        self.mutex.release()

    def start_moving(self, goal, cmd_id, cmd_type):
        rospy.loginfo("Setting a new goal:\t" + str(goal))
        rospy.loginfo("Current cmd:\t" + str(cmd_type))
        self.cmd_id = cmd_id
        self.current_cmd = cmd_type
        self.goal = goal
        if self.current_cmd == "move_line":
            self.current_state = "move_rotate"

    def timer_callback(self):

        self.calculate_current_status()



        # if self.current_cmd == "move_line" and self.current_state == "stop":
        #     self.terminate_following()

        if self.current_cmd == "move_arc":
            self.move_arc()
        elif self.current_cmd == "move_line":
            self.move_line()

    def calculate_current_status(self):
        """
        Calculates X and Y distance from current location to goal
        Calculates difference in rotation between robot's and goal's orientations
        Calculates length of vector to follow and it's orientation
        Calculates remained path to go in units of angle and distance
        :return:
        """

        rospy.loginfo('---------------------------------------')
        rospy.loginfo('CALCULATING CURRENT STATUS')

        while not self.update_coords():
            rospy.sleep(0.05)

        self.distance_map_frame, self.theta_diff = self.calculate_distance(self.coords, self.goal)
        self.gamma = np.arctan2(self.distance_map_frame[1], self.distance_map_frame[0])
        self.d_norm = np.linalg.norm(self.distance_map_frame)
        rospy.loginfo("euclidean distance left %.3f", self.d_norm)

        # path_done = np.sqrt(self.d_init**2 + self.alpha_init**2) - np.sqrt(d**2 + alpha**2)
        self.path_left = np.sqrt(self.d_norm ** 2 + self.theta_diff ** 2)

    def terminate_moving(self):
        self.timer.shutdown()
        self.set_speed(np.zeros(3))

    def calculate_distance(self, coords1, coords2):
        distance_map_frame = coords2[:2] - coords1[:2]
        theta_diff = wrap_angle(coords2[2] - coords1[2])
        return distance_map_frame, theta_diff

    # TODO local to global
    @staticmethod
    def rotation_transform(vec, angle):
        # counterclockwise rotation
        M = np.array([[np.cos(angle), -np.sin(angle)],
                      [np.sin(angle), np.cos(angle)]])
        ans = vec.copy()
        ans[:2] = np.matmul(M, ans[:2].reshape((2, 1))).reshape(2)
        return ans

    def set_speed(self, v_cmd):
        vx, vy, w = v_cmd

        # TODO
        # vx, vy = self.rotation_transform(np.array([vx, vy]), -self.coords[2])

        rospy.loginfo("v_cmd:\t" + str(v_cmd))
        cmd = " 8 " + str(v_cmd[0]) + " " + str(v_cmd[1]) + " " + str(v_cmd[2])
        rospy.loginfo("Sending cmd: " + cmd)
        self.pub_cmd.publish(cmd)

    def get_optimal_velocity(self):
        v = max(self.V_MIN, min(self.Kp * self.d_norm, self.V_MAX))
        return v

    def move_arc(self):
        """
        go to goal in one movement by arc path

        :return:
        """

        rospy.loginfo("-------NEW ARC MOVEMENT-------")
        rospy.loginfo("Goal:\t" + str(self.goal))



        v = self.get_optimal_velocity()

        if self.theta_diff < 1e-4:
            w = 0
        else:
            R = 0.5 * self.d_norm / np.sin(self.theta_diff / 2)
            w = v / R  # must be depended on v such way so path becomes an arc

        if abs(w) > self.W_MAX:
            k = abs(w) / self.W_MAX
            v /= k
            w /= k

        beta = wrap_angle(self.gamma - self.theta_diff / 2)
        vx = v * np.cos(beta)
        vy = v * np.sin(beta)
        vx, vy = self.rotation_transform(np.array([vx, vy]), -self.coords[2])
        v_cmd = np.array([vx, vy, w])
        self.set_speed(v_cmd)

        if self.path_left < self.XY_GOAL_TOLERANCE and self.path_left < self.YAW_GOAL_TOLERANCE:
            self.terminate_moving()

    def move_line(self):
        rospy.loginfo(' ------- NEW LINE MOVEMENT ------- ')
        rospy.loginfo('goal is' + str(self.goal))

        if self.current_state == "move_rotate" and abs(self.theta_diff) < self.YAW_GOAL_TOLERANCE:
            self.current_state = "move_translate"

        if self.current_state == "move_translate" and self.d_norm < self.XY_GOAL_TOLERANCE:
            self.current_state = "stop"

        if self.current_state == "move_rotate":
            self.rotate_odom()
        elif self.current_state == "move_translate":
            self.translate_odom()
        elif self.current_state == "stop":
            self.terminate_moving()


        # while abs(self.theta_diff) > self.YAW_GOAL_TOLERANCE:
        #     self.rotate_odom()
        #
        # rospy.loginfo(' ROTATING FINISHED %.4f', self.theta_diff)
        # self.terminate_following()
        #
        # while self.d_norm > self.XY_GOAL_TOLERANCE:
        #     self.translate_odom()
        #
        # rospy.loginfo('TRANSLATION FINISHED WITH TOLERANCE %.4f', self.d_norm)
        # self.terminate_following()

    def rotate_odom(self):
        rospy.loginfo("-1- step - NEW ROTATIONAL MOVEMENT")
        rospy.loginfo('current orientation' + str(self.coords[2]))
        rospy.loginfo('ROT_ODOM: abs theta_diff wrapped %.4f', abs(self.theta_diff))

        sign_w = self.theta_diff / abs(self.theta_diff)

        # TODO
        # w = self.get_optimal_velocity()

        w = sign_w * self.W_MAX
        v_cmd = np.array([0, 0, w])
        self.set_speed(v_cmd)

    def translate_odom(self):
        rospy.loginfo("-2- step - NEW TRANSLATE MOVE")

        vx = self.d_norm * np.cos(self.gamma)
        vy = self.d_norm * np.sin(self.gamma)
        w = 0

        if abs(vx) > self.V_MAX:
            k = abs(vx) / self.V_MAX
            vx /= k
            vy /= k

        if abs(vy) > self.V_MAX:
            k = abs(vy) / self.V_MAX
            vx /= k
            vy /= k

        vx, vy = self.rotation_transform(np.array([vx, vy]), -self.coords[2])
        v_cmd = np.array([vx, vy, w])
        self.set_speed(v_cmd)

    def update_coords(self):
        try:
            trans = self.tfBuffer.lookup_transform('map', self.robot_name, rospy.Time())
            q = [trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z,
                 trans.transform.rotation.w]
            angle = euler_from_quaternion(q)[2] % (2 * np.pi)
            self.coords = np.array([trans.transform.translation.x, trans.transform.translation.y, angle])
            rospy.loginfo("Robot coords:\t" + str(self.coords))
            return True
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.loginfo("MotionPlanner failed to lookup tf2.")
            return False


if __name__ == "__main__":
    planner = MotionPlannerNode()
    rospy.spin()

    # TODO
    # @staticmethod
    # def vel(path_done, path_left, V_MIN, V_MAX, k, Kp):
# rospy.loginfo('VEL FUNC')
#     if path_done < path_left:
#     #rospy.loginfo('path done', path_done)
#         v = min(V_MAX * np.e**(-1 / (path_done / k + 0.1)) + V_MIN, V_MAX)
#
#
#         # use linear ACC
#         #v = min(V_MAX, Kp*path_done + V_MIN)
#     rospy.loginfo('acc vel %.4f', v)
#     else:
#         # expo DCL
#         v = min(V_MAX * np.e**(-1 / (path_left / k + 0.1)) + V_MIN, V_MAX)
#     rospy.loginfo('dcl vel %.4f', v)
#     return v
