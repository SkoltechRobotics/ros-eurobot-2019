#!/usr/bin/env python

import rospy
import numpy as np
import behavior_tree as bt
import bt_ros
import tf2_ros
from bt_controller import SideStatus, BTController
from std_msgs.msg import String
from main_strategies import OptimalStrategy, GreedyStrategy, BlindStrategy


class MainRobotBT(object):
    # noinspection PyTypeChecker
    def __init__(self):
        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer)

        self.move_publisher = rospy.Publisher("navigation/command", String, queue_size=100)
        self.manipulator_publisher = rospy.Publisher("manipulator/command", String, queue_size=100)
        self.stm_publisher = rospy.Publisher("stm/command", String, queue_size=100)

        self.move_client = bt_ros.ActionClient(self.move_publisher)
        self.manipulator_client = bt_ros.ActionClient(self.manipulator_publisher)
        self.stm_client = bt_ros.ActionClient(self.stm_publisher)

        self.strategy = None
        # self.purple_strategy = OptimalStrategy(SideStatus.PURPLE)
        # self.yellow_strategy = OptimalStrategy(SideStatus.YELLOW)

        self.purple_strategy = BlindStrategy(SideStatus.PURPLE)
        self.yellow_strategy = BlindStrategy(SideStatus.YELLOW)

        self.side_status = None

        self.bt = None
        self.bt_timer = None

        rospy.Subscriber("navigation/response", String, self.move_client.response_callback)
        rospy.Subscriber("manipulator/response", String, self.manipulator_client.response_callback)

    def start(self):

        self.bt = bt.Root(self.strategy.tree,
                          action_clients={"move_client": self.move_client,
                                          "manipulator_client": self.manipulator_client,
                                          "stm_client": self.stm_client})

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
        rospy.init_node("main_robot_BT")
        main_robot_bt = MainRobotBT()
        bt_controller = BTController(main_robot_bt)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
