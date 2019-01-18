#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import numpy as np
import tf2_ros
import tf_conversions
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from core_functions import *

WHEEL_RADIUS = 0.05
WHEEL_L = 0.662
TRANSFER_NUMBER = 10

SET_NOISE = 0.1
GET_NOISE = 0.1

INTEGRATE_RATE = 100
ODOMETRY_RATE = 20


# noinspection PyTypeChecker,PyUnusedLocal
class SimMoveNode(object):
    def __init__(self):
        # ROS node
        rospy.init_node('simulate_node', anonymous=True)
        rospy.Subscriber("cmd_vel", Twist, self.set_twist)
        self.pub_odom = rospy.Publisher("odom", Odometry, queue_size=1)

        # Robot params
        self.base_frame = "secondary_robot"
        self.odom_frame = "secondary_robot_odom"

        # TF broadcaster
        self.br = tf2_ros.TransformBroadcaster()

        # Odometry point
        self.sim_true_point = np.array([0, 0, 0])
        self.odom_point = np.array([0, 0, 0])
        self.last_odom_integration_time = rospy.get_time()
        self.last_integration_time = rospy.get_time()
        self.sim_true_wheel_speeds = np.array([0, 0, 0])

        self.integrate_rate = rospy.Rate(INTEGRATE_RATE)
        rospy.Timer(rospy.Duration(1. / ODOMETRY_RATE), self.pub_odom_coords_timer_callback)

    def set_twist(self, twist):
        vx = twist.linear.x
        vy = twist.linear.y
        w = twist.angular.z
        self.set_wheel_speeds(np.array([vx, vy, w]))

    @staticmethod
    def robot_speed2wheel_speed(v, w):
        v_left = (2 * v - w * WHEEL_L) / (2. * WHEEL_RADIUS)
        v_right = - (2 * v + w * WHEEL_L) / (2. * WHEEL_RADIUS)
        return np.array([v_left, v_right])

    @staticmethod
    def wheel_speed2robot_speed(v_left, v_right):
        """
        Converts wheel speed [v_left, v_right] to [v, w].
        """
        return np.array([(-v_right + v_left) * WHEEL_RADIUS / 2., 0,
                         (-v_right - v_left) * WHEEL_RADIUS / WHEEL_L])

    def set_wheel_speeds(self, speeds):
        #set_noise = np.random.normal(0, SET_NOISE, (3, ))
        #speeds = speeds + set_noise
        #speeds = speeds - np.clip(speeds, -SET_NOISE, SET_NOISE)
        self.sim_true_wheel_speeds = speeds

    def get_wheel_speeds(self):
        speeds = self.sim_true_wheel_speeds
        #get_noise = np.random.normal(0, GET_NOISE, (3, ))
        #speeds = get_noise + speeds
        #speeds = speeds - np.clip(speeds, -GET_NOISE, GET_NOISE)
        #speeds = np.array(speeds, dtype=np.int32)
        return speeds

    def publish_tf(self, point, child_frame, parent_frame):
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = 'map'
        t.child_frame_id = 'secondary_robot'
        t.transform = cvt_point2ros_transform(point)
        self.br.sendTransform(t)

    def publish_odom(self, point, vel):
        self.publish_tf(point, self.base_frame, self.odom_frame)
        odom = Odometry()
        odom.header.frame_id = "secondary_robot_odom"
        odom.header.stamp = rospy.Time.now()
        odom.child_frame_id = "secondary_robot"
        odom.twist.twist.linear.x = vel[0]
        odom.twist.twist.linear.y = vel[1]
        odom.twist.twist.angular.z = vel[2]
        odom.pose.pose = cvt_point2ros_pose(point)
        odom.pose.pose.position.z = 0.5
        self.pub_odom.publish(odom)

    def pub_odom_coords_timer_callback(self, event):
        # time between iterations
        now = rospy.get_time()
        dt = now - self.last_odom_integration_time
        self.last_odom_integration_time = now

        # transform to robot velocity
        vels = self.get_wheel_speeds()
        robot_vel = vels
        dpoint = robot_vel * dt

        # integrate
        self.odom_point = cvt_local2global(dpoint, self.odom_point)
        self.publish_odom(self.odom_point, robot_vel)

    def integrate(self):
        while not rospy.is_shutdown():
            # time between iterations
            now = rospy.get_time()
            dt = now - self.last_integration_time
            self.last_integration_time = now

            vel = self.sim_true_wheel_speeds
            dpoint = vel * dt

            # integrate
            self.sim_true_point = cvt_local2global(dpoint, self.sim_true_point)
            self.integrate_rate.sleep()


if __name__ == '__main__':
    try:
        node = SimMoveNode()
        node.integrate()
    except rospy.ROSInterruptException:
        pass
