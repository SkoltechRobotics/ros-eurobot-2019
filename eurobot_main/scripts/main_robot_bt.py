#!/usr/bin/env python
import rospy
import behavior_tree as bt
import bt_ros
import numpy as np
from std_msgs.msg import String


class MainRobotBT:
    def __init__(self):

        # publishers
        self.move_command_publisher = rospy.Publisher('move_command', String, queue_size=10)
        self.stm_command_publisher = rospy.Publisher('stm_command', String, queue_size=10)  # FIXME
        self.response_publisher = rospy.Publisher("response", String, queue_size=10)
        self.response_publisher = rospy.Publisher("manipulator/response", String, queue_size=10)

        rospy.Subscriber("stm_response", String, self.response_callback)

        self.last_response_id = None
        self.last_response_args = None

        self.id_command = 1
        rospy.sleep(2)

        self.timer = None
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.RATE), self.timer_callback)


    def timer_callback(self, event):
        """
        in timer publish 0x03 till response in 1
        then fire command to execute BT
        :param event:
        :return:
        """
        cmd = 0x03
        message = 'is_cord_removed' + str(self.id_command) + str(cmd)
        self.stm_command_publisher.publish(message)  # FIXME
        rospy.sleep(0.1)
        if self.last_response_id == ("is_cord_removed-" + str(self.id_command)):
            if self.last_response_args == 1:
                self.id_command += 1
                self.execute_BT()
                self.timer.shutdown()
                rospy.loginfo("TN -- Robot has completely stopped")
                rospy.sleep(1.0 / 40)


    def response_callback(self, data):
        response = data.data.split()
        print ("RESPONSE=", response)
        self.last_response_id = response[0]
        self.last_response_args = response[1]
