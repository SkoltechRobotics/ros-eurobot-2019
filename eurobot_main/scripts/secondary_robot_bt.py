#!/usr/bin/env python

import numpy as np

import rospy
from std_msgs.msg import String

import bt_ros
import behavior_tree as bt
import secondary_strategy
from bt_controller import SideStatus, BTController

from core_functions import *

class SecondaryRobotBT(object):
    def __init__(self):
        self.move_publisher = rospy.Publisher("navigation/command", String, queue_size=10)
        self.move_client = bt_ros.ActionClient(self.move_publisher)
        rospy.Subscriber("navigation/response", String, self.move_client.response_callback)

        self.manipulator_publisher = rospy.Publisher("manipulator/command", String, queue_size=10)
        self.manipulator_client = bt_ros.ActionClient(self.manipulator_publisher)
        rospy.Subscriber("manipulator/response", String, self.manipulator_client.response_callback)

        self.purple_strategy = secondary_strategy.VovanStrategy(SideStatus.PURPLE)
        self.yellow_strategy = secondary_strategy.VovanStrategy(SideStatus.YELLOW)

        self.side_status = None
        self.strategy = None

        self.bt = None
        self.bt_timer = None




    def start(self):
        self.bt = bt.Root(self.strategy.tree,
                          action_clients={"move_client": self.move_client, "manipulator_client": self.manipulator_client})

        self.bt_timer = rospy.Timer(rospy.Duration(0.1), self.timer_callback)

    def change_side(self, side):
        if side == SideStatus.PURPLE:
            self.strategy = self.purple_strategy
        elif side == SideStatus.YELLOW:
            self.strategy = self.yellow_strategy
        else:
            self.strategy = None
        self.side_status = side

    def timer_callback(self, event):
        status = self.bt.tick()
        if status != bt.Status.RUNNING:
            self.bt_timer.shutdown()
        print("============== BT LOG ================")
        self.bt.log(0)


if __name__ == '__main__':
    try:
        rospy.init_node("secondary_robot_BT")
        secondary_robot_bt = SecondaryRobotBT()
        bt_controller = BTController(secondary_robot_bt)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
