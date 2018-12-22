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

class MotionPlannerNode:
    def __init__(self):
        rospy.init_node("motion_planner", anonymous=True)

        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer)

        self.robot_name="secondary_robot"

        self.mutex = Lock()

        self.cmd_id = None
        self.cmd_type = None
        self.cmd_args = None
        self.t_prev = None
        self.goal = None
        self.mode = None

        self.RATE = 10

        self.XY_GOAL_TOLERANCE = 0.1
        self.YAW_GOAL_TOLERANCE = 0.1

        self.vel = np.zeros(3)

        self.path_left = 9999 # big initial value

        self.cmd_stop_robot_id = None
        self.stop_id = 0

        self.V_MAX = 0.2 # m/s
        self.V_MIN = 0.15
        self.W_MAX = 0.3

        self.Kp = 5


        #self.pub_twist = rospy.Publisher("cmd_vel", Twist, queue_size=1)
        self.pub_cmd = rospy.Publisher("/secondary_robot/stm_command", String, queue_size=1)

        rospy.Subscriber("move_command", String, self.cmd_callback, queue_size=1)

        # start the main timer that will follow given goal points



    def cmd_callback(self, data):

        """
        if new command comes
        should check if current command is finished - why?
        parse new command and write to self parsed args
        stop publishing in stm_command

        :param data: type of action and args. For move_arc args are goal's X, Y, THETA orienation
        :return:
        """

        self.mutex.acquire()

        rospy.loginfo("=====================================")
        rospy.loginfo("NEW CMD:\t" + str(data.data))

        # when new cmd arrives shutdown running timer
        if self.timer is not None:
            self.timer.shutdown()

        # parse name,type
        data_splitted = data.data.split()
        cmd_id = data_splitted[0]
        cmd_type = data_splitted[1]
        cmd_args = data_splitted[2:]

        if cmd_type == "move_arc" or cmd_type == "move_line":
            args = np.array(cmd_args).astype('float')
            goal = args[:3]
            goal[2] %= (2 * np.pi)
            self.set_goal(goal, cmd_id, cmd_type)
            self.timer = rospy.Timer(rospy.Duration(1.0 / self.RATE), self.mode)


        elif cmd_type == "stop" or self.path_left < self.XY_GOAL_TOLERANCE and self.path_left < self.YAW_GOAL_TOLERANCE:
            self.cmd_id = cmd_id
            self.mode = cmd_type
            self.terminate_following()

        self.mutex.release()



    def set_goal(self, goal, cmd_id, cmd_type):
        rospy.loginfo("Setting a new goal:\t" + str(goal))
        rospy.loginfo("Mode:\t" + str(cmd_type))
        self.cmd_id = cmd_id
        self.mode = cmd_type
        self.goal = goal


    def terminate_following(self):
        self.set_speed(np.zeros(3))


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
        rospy.loginfo("v_cmd:\t" + str(v_cmd))
        cmd = " 8 " + str(v_cmd[0]) + " " + str(v_cmd[1]) + " " + str(v_cmd[2])
        rospy.loginfo("Sending cmd: " + cmd)
        self.pub_cmd.publish(cmd)


    def distance(self, coords1, coords2):
        ans = coords2 - coords1
        ans[2] = self.wrap_angle(ans[2])
        return ans


    def wrap_angle(self,angle):
        """
        Wraps the given angle to the range [-pi, +pi].
        :param angle: The angle (in rad) to wrap (can be unbounded).
        :return: The wrapped angle (guaranteed to in [-pi, +pi]).
        """
        return (angle + np.pi) % (np.pi * 2) - np.pi



    def arc_move(self):
        """
        go to goal in one movement by arc path

        :return:
        """

        rospy.loginfo("-------NEW ARC MOVEMENT-------")
        rospy.loginfo("Goal:\t" + str(self.goal))

        while not self.update_coords():
            rospy.sleep(0.05)

        distance_map_frame = self.distance(self.coords, self.goal)

        x_diff = distance_map_frame[0]
        y_diff = distance_map_frame[1]
        alpha = distance_map_frame[2] # theta_diff

        gamma = np.arctan2(y_diff, x_diff)

        d_norm = np.linalg.norm(distance_map_frame[:2])
        #sqrtD = np.sqrt(x_diff**2 + y_diff**2) # rho,
        rospy.loginfo("euclidian distance left %.3f", d_norm)

        #path_done = np.sqrt(self.d_init**2 + self.alpha_init**2) - np.sqrt(d**2 + alpha**2)
        self.path_left = np.sqrt(d_norm**2 + alpha**2) # in units of angle and distance

        v = max(self.V_MIN, min(self.Kp * d_norm, self.V_MAX))

        if alpha < 1e-4:
            w = 0
        else:
            R = 0.5 * d_norm / np.sin(alpha/2)
            w = v / R  # must be depended on v such way so path becomes an arc

        beta = self.wrap_angle(gamma - alpha/2)

        vx = v * np.cos(beta)
        vy = v * np.sin(beta)
        vx, vy = self.rotation_transform(np.array([vx, vy]), -self.coords[2])

        v_cmd = np.array([vx, vy, w])
        self.set_speed(v_cmd)


    def line_move(self):
        pass

    def update_coords(self):
        try:
            trans = self.tfBuffer.lookup_transform('map', self.robot_name, rospy.Time())
            q = [trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w]
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
