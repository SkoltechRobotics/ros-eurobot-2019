#!/usr/bin/env python
import rospy
import behavior_tree as bt
import bt_ros
from std_msgs.msg import String
from bt_controller import SideStatus, BTController
from core_functions import *


class ScoreController(object):
    def __init__(self, collected_pucks, robot_name):
        self.collected_pucks = collected_pucks
        self.pucks = ["REDIUM", "GREENIUM", "BLUNIUM", "GOLDENIUM", "UNDEFINED"]
        self.places = ["RED", "GREEN", "BLUE", "ACC", "SCALES", "CELLS", "MIDDLE"]
        self.bonuses = ["UNLOCK_GOLDENIUM_BONUS", "GRAB_GOLDENIUM_BONUS"]
        self.robot_name = robot_name
        if self.robot_name == "main_robot":
            self.score_publisher = rospy.Publisher("/main_robot/score", String, queue_size=100)
        elif self.robot_name == "secondary_robot":
            self.score_publisher = rospy.Publisher("/secondary_robot/score", String, queue_size=100)

    def reward(self, bonus):
        assert bonus in self.bonuses
        self.score_publisher.publish(bonus)

    def add(self, puck):
        assert puck in self.pucks
        self.collected_pucks.set(np.append(self.collected_pucks.get(), puck))  # Action
        rospy.loginfo("you added " + puck)
        rospy.loginfo("Now inside: " + str(self.collected_pucks.get()))
        rospy.loginfo(" ")

    def unload(self, place):  # side="top"
        assert place in self.places

        if self.collected_pucks.get().size == 0:
            rospy.loginfo("you tried unloading a puck, but there are no pucks left ")
        else:
            lifo_puck = self.collected_pucks.get()[-1]
            self.collected_pucks.set(self.collected_pucks.get()[:-1])

            rospy.loginfo('Unloaded lifo: ' + str(lifo_puck) + " on " + place)
            rospy.loginfo('Pucks to unload: ' + str(self.collected_pucks.get().size) + " " + str(self.collected_pucks.get()))
            self.score_publisher.publish(lifo_puck + "_ON_" + place)

