#!/usr/bin/env python
import rospy
import geometry_msgs.msg
import tf_conversions
import tf2_ros


class Odometry():
    def __init__(self, stm_protocol, rate):
        self.robot_name = rospy.get_param("robot_name")
        self.lidar_x = rospy.get_param("lidar_x")
        self.lidar_y = rospy.get_param("lidar_y")
        self.lidar_a = rospy.get_param("lidar_a")
        
        self.tf2_broad = tf2_ros.TransformBroadcaster()
        self.stm_protocol = stm_protocol
        rospy.Timer(rospy.Duration(1. / rate), self.odom_callback)
        
    
    def odom_callback(self, event):
        successfully, values = self.stm_protocol.send(0x0F, args=None)
        if successfully:
            self.send_odometry(values)


    def send_odometry(self, values):
        rospy.loginfo("odom: %.3f %.3f %.3f" % tuple(values))
        t = geometry_msgs.msg.TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = self.robot_name + "_odom"
        t.child_frame_id = self.robot_name
        t.transform.translation.x = values[0]
        t.transform.translation.y = values[1]
        t.transform.translation.z = 0.0
        q = tf_conversions.transformations.quaternion_from_euler(0, 0, values[2])
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        
        self.tf2_broad.sendTransform(t)
        t = geometry_msgs.msg.TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = self.robot_name
        t.child_frame_id = self.robot_name + "_laser"
        t.transform.translation.x = self.lidar_x
        t.transform.translation.y = self.lidar_y
        t.transform.translation.z = .4
        q = tf_conversions.transformations.quaternion_from_euler(0, 0, self.lidar_a)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
 
        self.tf2_broad.sendTransform(t)
