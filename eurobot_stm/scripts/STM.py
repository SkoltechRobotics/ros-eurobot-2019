#!/usr/bin/env python
import rospy
from std_msgs.msg import String

import itertools
import datetime

from STM_protocol import STMprotocol
from odometry import Odometry
from manipulator import Manipulator

ODOM_RATE = rospy.get_param("ODOM_RATE")
STATUS_RATE = rospy.get_param("STATUS_RATE")


class STM(object):
    def __init__(self, serial_port, baudrate=115200):
        rospy.init_node('stm_node', anonymous=True)
        rospy.Subscriber("stm/command", String, self.stm_command_callback)
        self.response = rospy.Publisher("stm/response", String, queue_size=50)

        self.stm_protocol = STMprotocol(serial_port, baudrate)
        self.odometry = Odometry(self.stm_protocol, ODOM_RATE)
        self.stm_status = STMstatus(self.stm_protocol, STATUS_RATE)

    def stm_command_callback(self, data):
        id, cmd, args = self.parse_data(data)
        successfully, values = self.stm_protocol.send(cmd, args)
        st = ""
        if values is not None:
            for val in values:
                st += str(val)
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
    def __init__(self, stm_protocol, rate):
        self.stm_protocol = stm_protocol
        
        self.start_status_publisher = rospy.Publisher("stm/start_status", String, queue_size=1)
        self.side_status_publisher = rospy.Publisher("stm/side_status", String, queue_size=1)
        self.strategy_status_publisher = rospy.Publisher("stm/strategy_status", String, queue_size=1)
        self.proximity_status_publisher = rospy.Publisher("stm/proximity_status", String, queue_size=1)
        self.limit_switch_status_publisher = rospy.Publisher("stm/limit_switch_status", String, queue_size=1)

        self.start_flag = False
        self.start_status_counter = 0

        rospy.Timer(rospy.Duration(1. / rate), self.update_status)

    def update_status(self, event):
        try:
            if not self.start_flag:
                successfully, values = self.stm_protocol.send(0x3, args=None)
            
                message = ""
                for val in values:
                    message += str(val)
                if message == "1":
                    self.start_status_counter += 1
                if self.start_status_counter == 5:
                    self.start_flag = True
                if successfully and self.start_flag:
                    self.start_status_publisher.publish("1")
                elif successfully:
                    self.start_status_publisher.publish("0")

                successfully, values = self.stm_protocol.send(0x4, args=None)

                message = ""
                for val in values:
                    message += str(val)
                if successfully:
                    self.side_status_publisher.publish(message)

                # Strategy
                successfully, values = self.stm_protocol.send(0x5, args=None)

                message = ""
                for val in values:
                    message += str(val)
                if successfully:
                    self.strategy_status_publisher.publish(message)

            else:
                successfully, values = self.stm_protocol.send(0x70, args=None)

                message = ""
                for val in values:
                    message += str(val) + " "
                if successfully:
                    self.proximity_status_publisher.publish(message)


                successfully, values = self.stm_protocol.send(0x19, args=None)

                message = ""
                for val in values:
                    message += str(val) + " "
                if successfully:
                    self.limit_switch_status_publisher.publish(message)

        except Exception as exc:
            rospy.loginfo('Exception:\t' + str(exc))
            rospy.loginfo('At time:\t' + str(datetime.datetime.now()))
            rospy.loginfo('Failed parse values from stm response')
            rospy.loginfo('--------------------------')


if __name__ == '__main__':
    serial_port = "/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0"
    stm = STM(serial_port)
    rospy.spin()
