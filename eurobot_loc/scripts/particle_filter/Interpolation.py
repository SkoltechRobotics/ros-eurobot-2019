#!/usr/bin/env python
import rospy
import numpy as np
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import LaserScan
import tf2_ros
import tf_conversions
from np_Particle import cvt_local2global, cvt_global2local, ParticleFilter


class new_odom(object):
    def __init__(self):
        self.tf_buffer = tf2_ros.Buffer()
        self.pf = ParticleFilter()
        self.listener_tf = tf2_ros.TransformListener(self.tf_buffer)
        self.rate = rospy.Rate(20)
        self.scan_time = rospy.Time.now()
        self.scan_time = self.scan_time.secs + self.scan_time.nsecs * 1e-9
        self.odom_time_buff = []
        self.odom_time_after_scan = self.scan_time
        self.odom_time_before_scan = self.scan_time
        self.odom_time_buff.append(self.odom_time_before_scan)
        self.odom_time_buff.append(self.odom_time_after_scan)
        self.buff_odom = []
        self.buff_tf = []
        self.time_tf_stamp = [self.odom_time_before_scan, self.odom_time_after_scan]
        self.buff_coords = []
        self.scan_stamp = rospy.Time.now()
        self.lidar_point = np.array([rospy.get_param("lidar_x"), rospy.get_param("lidar_y"),
                                     rospy.get_param("lidar_a")])
        # f, robot_odom_point = self.get_odom()
        # rospy.loginfo("sdn")
        rospy.sleep(1)

        robot_odom_point = self.pf.start_coords
        #rospy.loginfo("Fdgw")
        self.lidar_odom_point = cvt_local2global(self.lidar_point, robot_odom_point)
        self.prev_lidar_odom_point = self.lidar_odom_point.copy()
        self.lidar_odom_time = rospy.Time.now()
        self.prev_lidar_odom_time = rospy.Time.now()
        self.scan = rospy.Time.now()
        self.robot_name = "secondary_robot"
        rospy.Subscriber("/secondary_robot/scan", LaserScan, self.scan_callback, queue_size=1)
        rospy.Subscriber("/tf", TransformStamped, self.callback_frame, queue_size=1)

    def scan_callback(self, scan):
        self.scan = np.array([np.array(scan.ranges), scan.intensities]).T

    def print_data(self):
        rospy.loginfo(self.prev_lidar_odom_point)
        rospy.loginfo(self.lidar_odom_point)
        rospy.Timer(rospy.Duration(1. / 20), self.print_data)


    def callback_frame(self, data):
        #rospy.loginfo(data)
        try:
            if data.transforms[0].header.frame_id == "secondary_robot_odom":
                self.prev_lidar_odom_time = self.lidar_odom_time
                self.lidar_odom_time = data.transforms[0].header.stamp
                #rospy.loginfo(str(self.time_tf_stamp[-1].secs + self.time_tf_stamp[-1].nsecs * 1e-9))
                #rospy.loginfo("fsdhrjek")
                odom = self.tf_buffer.lookup_transform("secondary_robot_odom", "secondary_robot", self.lidar_odom_time)
                #rospy.loginfo(str(odom))
                robot_odom_point = self.tf_to_coords(odom)
                self.prev_lidar_odom_point = self.lidar_odom_point
                self.lidar_odom_point = cvt_local2global(self.lidar_odom_point, robot_odom_point)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            pass

    def get_times(self):
        return self.prev_lidar_odom_point, self.lidar_odom_point, self.prev_lidar_odom_time, self.lidar_odom_time

    # def callback_frame(self, data):
    #     try:
    #         if data.transforms[0].header.frame_id == "secondary_robot_odom":
    #             self.time_tf_stamp.append(data.transforms[0].header.stamp)
    #             self.odom_time_buff.append(self.time_tf_stamp[-1].secs + self.time_tf_stamp[-1].nsecs * 1e-9)
    #             #rospy.loginfo(str(self.time_tf_stamp[-1].secs + self.time_tf_stamp[-1].nsecs * 1e-9))
    #             self.buff_odom.append(self.tf_buffer.lookup_transform("secondary_robot_odom", "secondary_robot",
    #                                                                   self.time_tf_stamp[-1]))
    #             self.buff_coords.append(self.tf_to_coords(self.buff_odom[-1]))
    #     except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
    #         pass

    # def callback_frame(self, data):
    #     try:
    #         if data.transforms[0].header.frame_id == "secondary_robot_odom":
    #             #rospy.loginfo(str(self.time_tf_stamp[-1].secs + self.time_tf_stamp[-1].nsecs * 1e-9))
    #             lidar_odom_time = data.transforms[0].header.stamp
    #             odom = self.tf_buffer.lookup_transform("secondary_robot_odom", "secondary_robot", lidar_odom_time)
    #             robot_odom_point = self.tf_to_coords(odom)
    #     except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
    #         pass

    def get_odom(self):
        try:
            t = self.tf_buffer.lookup_transform('secondary_robot_odom', 'secondary_robot', rospy.Time(0))
            q = [t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w]
            yaw = tf_conversions.transformations.euler_from_quaternion(q)[2]
            return True, np.array([t.transform.translation.x, t.transform.translation.y, yaw])
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logwarn("Transform for PF with error")
            return False, np.array([0, 0, 0])

    def stamp2secs(self, time):
        self.odom_time_buff.append(time.secs + time.nsecs * 1e-9)

    def tf_to_coords(self, odom):
        q = [odom.transform.rotation.x, odom.transform.rotation.y,
             odom.transform.rotation.z, odom.transform.rotation.w]
        yaw = tf_conversions.transformations.euler_from_quaternion(q)[2]
        x = odom.transform.translation.x
        y = odom.transform.translation.y
        return np.array([x, y, yaw])

    def interpolate(self):
        delta_x, delta_y, delta_a = cvt_global2local(self.buff_coords[-1], self.buff_coords[-2])
        delta = np.array([delta_x, delta_y, delta_a])/(self.odom_time_buff[-1] - self.odom_time_buff[-2])
        x, y, a = self.buff_coords[-1] + delta * (self.scan_time - self.odom_time_buff[-1])
        return self.scan_stamp, cvt_local2global(np.array([x, y, a]), self.buff_coords[-2])

if __name__ == "__main__":
    try:
        rospy.init_node("tf_listener_test", anonymous=True)
        gt = new_odom()
        rospy.spin()
    except rospy.ROSException:
        pass
