#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from threading import Lock
from std_msgs.msg import Int32MultiArray
import numpy as np
import datetime
import time
import struct
import serial
import itertools
from STMprotocol import STMprotocol

class STMnode(STMprotocol):
    def __init__(self, serial_port, baudrate=115200):
        # Init ROS
        rospy.init_node('stm_node', anonymous=True)
        # ROS subscribers
        rospy.Subscriber("stm_command", String, self.stm_command_callback)
        rospy.Subscriber("t")
        
        super(STMnode, self).__init__(serial_port, baudrate)
        self.mutex = Lock()
    
    def stm_command_callback(self, data):
        cmd, args = self.parse_data(data)
        successfully = self.send(cmd, args)
    
    def parse_data(self, data):
        data_splitted = data.data.split()
        cmd = int(data_splitted[0])
        args_dict = {'c': int, 'H': int, 'f': float}
        args = [action_args_dict[t](s) for t, s in itertools.izip(pack_format[cmd][1:], data_splitted[1:])]
        return cmd, args
    
    def send(self, cmd, args):
        self.mutex.acquire()
        successfully, values = self.send_command(cmd, args)
        self.mutex.release()
        rospy.loginfo('Got response args: '+ str(values))
        return successfully

if __name__ == '__main__':
    serial_port = "/dev/ttyUSB0"
    stm = STMnode(serial_port)
    rospy.spin()
