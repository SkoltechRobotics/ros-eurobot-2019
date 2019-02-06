#!/usr/bin/env python
import rospy
from std_msgs.msg import String

import itertools

from STM_protocol import STMprotocol
from odometry import Odometry
from manipulator import Manipulator

ODOM_RATE = rospy.get_param("ODOM_RATE")

class STM():
    def __init__(self, serial_port, baudrate=115200):
        rospy.init_node('stm_node', anonymous=True)
        rospy.Subscriber("stm_command", String, self.stm_command_callback)
        self.response = rospy.Publisher("stm_response", String, queue_size=50)

        self.stm_protocol = STMprotocol(serial_port, baudrate)
        self.odometry = Odometry(self.stm_protocol, ODOM_RATE)

    
    def stm_command_callback(self, data):
        id, cmd, args = self.parse_data(data)
        successfully, values = self.stm_protocol.send(cmd, args)
        response = str(id) + " " + str(values)
        self.response.publish(response)

    def parse_data(self, data):
        data_splitted = data.data.split()
        cmd = int(data_splitted[0])
        args_dict = {'c': str, 'H': int, 'f': float}
        args = [args_dict[t](s) for t, s in itertools.izip(self.stm_protocol.pack_format[cmd][1:], data_splitted[1:])]
        return cmd, args

if __name__ == '__main__':
    serial_port = "/dev/ttyUSB0"
    stm = STM(serial_port)
    rospy.spin()
