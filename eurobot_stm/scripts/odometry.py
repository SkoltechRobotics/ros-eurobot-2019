#!/usr/bin/env python
import rospy
import geometry_msgs.msg
import tf_conversions
import tf2_ros
import STM_protocol


class Odometry():
    def __init__(self, rate=40, stm_protocol):
        self.tf2_broad = tf2_ros.TransformBroadcaster()
        self.stm_protocol = stm_protocol
        
        rospy.Timer(rospy.Duration(1. / rate), self.odom_callback)
        
    
    def odom_callback(self, event):
        self.stm_protocol.send(0x0F, args=None)
        if successfully:
            self.send_odometry(values)


    def send_odometry(self, values):
        rospy.loginfo("odom: %.3f %.3f %.3f" % tuple(values))
        t = geometry_msgs.msg.TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "secondary_robot_odom"
        t.child_frame_id = "secondary_robot"
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
        t.header.frame_id = "secondary_robot"
        t.child_frame_id = "secondary_robot_laser"
        t.transform.translation.x = 0#self.laser_coords[0]
        t.transform.translation.y = 0#self.laser_coords[1]
        t.transform.translation.z = .4
        q = tf_conversions.transformations.quaternion_from_euler(0, 0, 0)#self.laser_angle)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
 
        self.tf2_broad.sendTransform(t)
            
