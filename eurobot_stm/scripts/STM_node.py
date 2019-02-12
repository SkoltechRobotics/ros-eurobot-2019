#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from threading import Lock

import serial
import itertools
from STM_protocol import STMprotocol
import odometry

ODOM_RATE = 40 # Hz

class STM():
    def __init__(self, serial_port, baudrate=115200):
        # Init ROS
        rospy.init_node('stm_node', anonymous=True)
        # ROS subscribers
<<<<<<< HEAD
        rospy.Subscriber("stm_command", String, self.stm_command_callback)
        
        #rospy.Publisher("odom", anonymous=True)
        self.tf2_broad = tf2_ros.TransformBroadcaster()
        
        super(STMnode, self).__init__(serial_port, baudrate)
        self.mutex = Lock()
	
	    self.laser_coords = (rospy.get_param('lidar_x') / 1000.0, rospy.get_param('lidar_y') / 1000.0, 0.41)
=======
        rospy.Subscriber("stm_command", String, self.stm_command_callback)  

	self.stm_protocol = STMprotocol(serial_port, baudrate)
	self.odometry = Odometry(self.stm_protocol, ODOM_RATE)

	self.laser_coords = (rospy.get_param('lidar_x') / 1000.0, rospy.get_param('lidar_y') / 1000.0, 0.41)
>>>>>>> origin/stm
        self.laser_angle = rospy.get_param('lidar_a')
    
    def stm_command_callback(self, data):
        cmd, args = self.parse_data(data)
        successfully, values = self.stm_protocol.send(cmd, args)

    def parse_data(self, data):
        data_splitted = data.data.split()
        cmd = int(data_splitted[0])
        args_dict = {'c': str, 'H': int, 'f': float}
        args = [args_dict[t](s) for t, s in itertools.izip(self.pack_format[cmd][1:], data_splitted[1:])]
        return cmd, args

if __name__ == '__main__':
    serial_port = "/dev/ttyUSB0"
    stm = STM(serial_port)
    rospy.spin()
