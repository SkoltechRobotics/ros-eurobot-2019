#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from threading import Lock
from std_msgs.msg import Int32MultiArray
import geometry_msgs.msg
import tf_conversions
import tf2_ros
import numpy as np
import datetime
import time
import struct
import serial
import itertools
from STM_protocol import STMprotocol

ODOM_RATE = 40 # Hz

class STMnode(STMprotocol):
    def __init__(self, serial_port, baudrate=115200):
        # Init ROS
        rospy.init_node('stm_node', anonymous=True)
        # ROS subscribers
        rospy.Subscriber("stm_command", String, self.stm_command_callback)
        
        #rospy.Publisher("odom", anonymous=True)
        self.tf2_broad = tf2_ros.TransformBroadcaster()
        
        super(STMnode, self).__init__(serial_port, baudrate)
        self.mutex = Lock()
	
	    self.laser_coords = (rospy.get_param('lidar_x') / 1000.0, rospy.get_param('lidar_y') / 1000.0, 0.41)
        self.laser_angle = rospy.get_param('lidar_a')
    
        rospy.Timer(rospy.Duration(1. / ODOM_RATE), self.odom_callback)
    
    def stm_command_callback(self, data):
        cmd, args = self.parse_data(data)
        successfully, values = self.send(cmd, args)
        
            
    def parse_data(self, data):
        data_splitted = data.data.split()
        cmd = int(data_splitted[0])
        args_dict = {'c': str, 'H': int, 'f': float}
        args = [args_dict[t](s) for t, s in itertools.izip(self.pack_format[cmd][1:], data_splitted[1:])]
        return cmd, args
    
    def send(self, cmd, args):
        self.mutex.acquire()
        successfully, values = self.send_command(cmd, args)
        self.mutex.release()
        rospy.loginfo('Got response args: '+ str(values))
        return successfully, values

    def odom_callback(self, event):
        successfully, values = self.send(0x0F, args=None)
        if successfully:
            self.send_odometry(values)
        
    def send_odometry(self, values):
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
    
if __name__ == '__main__':
    serial_port = "/dev/ttyUSB0"
    stm = STMnode(serial_port)
    rospy.spin()
