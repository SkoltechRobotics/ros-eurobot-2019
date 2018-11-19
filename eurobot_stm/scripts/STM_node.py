#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from threading import Lock
from geometry_msgs.msg import Twist, TransformStamped
from std_msgs.msg import Int32MultiArray
import numpy as np
import time
import struct
import serial
from STM_protocol import STMprotocol

class STMnode(STMprotocol):
    def __init__(self, serial_port, baudrate=115200):
        # Init ROS
        rospy.init_node('stm_command_node', anonymous=True)
        # ROS subscribers
        rospy.Subscriber("stm_command", String, self.stm_command_callback)
        
	super(STMnode, self).__init__(serial_port, baudrate)
        self.mutex = Lock()
    
    def stm_command_callback(self, data):
        cmd, args = self.parse_data(data)
        successfully = self.send(cmd, args)
    
    def parse_data(self, data):
        data_splitted = data.data.split()
        cmd = int(data_splitted[0])
        args = [float(num) for num in data_splitted[1:]]
        return cmd, args
    
    def send(self, cmd, args):
        self.mutex.acquire()
        successfully, values = self.send_command(cmd, args)
        self.mutex.release()
        rospy.loginfo('Got response args: ' + str(values))
        return successfully
    
    def square_test(self):
        self.pure_send_command(8, (0.2,0,0))
        time.sleep(3)
        self.pure_send_command(8, (0,0.2,0))
        time.sleep(3)
        self.pure_send_command(8, (-0.2,0,0))
        time.sleep(3)
        self.pure_send_command(8, (0,-0.2,0))
        time.sleep(3)
        self.pure_send_command(8, (0,0,0))

    def triangle(self):
	#self.pure_send_command(8, (0,0,3.14))
	#time.sleep(1)
	self.pure_send_command(8, (0,0.8,0))
	time.sleep(3)
	self.pure_send_command(8, (0.8,0,0))
	time.sleep(3)
	self.pure_send_command(8,(-0.8,-0.8,0))
	time.sleep(3)
	self.pure_send_command(8, (0,0,0))

if __name__ == '__main__':
    serial_port = "/dev/ttyUSB0"
    stm = STMnode(serial_port)
    rospy.spin()
