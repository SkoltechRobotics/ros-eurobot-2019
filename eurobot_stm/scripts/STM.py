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
        rospy.Subscriber("stm/command", String, self.stm_command_callback)
        self.response = rospy.Publisher("stm/response", String, queue_size=50)


        self.stm_protocol = STMprotocol(serial_port, baudrate)
        self.odometry = Odometry(self.stm_protocol, ODOM_RATE)
        self.stm_status = STMstatus(self.stm_protocol)

    def stm_command_callback(self, data):
        id, cmd, args = self.parse_data(data)
        successfully, values = self.stm_protocol.send(cmd, args)
        for val in values:
            st = "".join(str(val))
        response = str(id) + " " + st
        print ("RESPONSE=", values)
        self.response.publish(response)

    def parse_data(self, data):
        data_splitted = data.data.split()
        print (data_splitted)
        id = data_splitted[0]
        try:
            cmd = int(data_splitted[1])
        except ValueError as e:
            cmd = int(data_splitted[1], 16)
        args_dict = {'c': str, 'B': int, 'f': float}
        args = [args_dict[t](s) for t, s in itertools.izip(self.stm_protocol.pack_format[cmd][1:], data_splitted[2:])]
        return id, cmd, args

class STMstatus(object):
    def __init__(self, stm_protocol):
        self.start_status_publisher = rospy.Publisher("stm/start_status", String, queue_size=50)
        self.side_status_publisher = rospy.Publisher("stm/side_status", String, queue_size=50)

    def update_status(self, event):
        successfully, values = self.stm_protocol.send(0x3, args=None)
        if successfully:
            
            self.start_status_publisher.publish(values)

        successfully, values = self.stm_protocol.send(0x4, args=None)
        if successfully:
            self.side_status_publisher.publish(values)

if __name__ == '__main__':
    # TODO::search for ports

    serial_port = "/dev/ttyUSB0"
    stm = STM(serial_port)
    rospy.spin()