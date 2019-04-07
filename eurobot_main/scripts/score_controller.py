#!/usr/bin/env python
import rospy
import behavior_tree as bt
import bt_ros
from std_msgs.msg import String
from bt_controller import SideStatus, BTController
from core_functions import *


class ScoreController(object):
    def __init__(self, collected_pucks):
        self.collected_pucks = collected_pucks
        self.pucks = ["REDIUM", "GREENIUM", "BLUNIUM", "GOLDENIUM"]
        self.places = ["RED", "GREEN", "BLUE", "ACC", "SCALES"]
        self.bonuses = ["UNLOCK_GOLDENIUM_BONUS", "GRAB_GOLDENIUM_BONUS"]
        self.score_publisher = rospy.Publisher("/score", String, queue_size=100)


    def reward(self, bonus):
        assert bonus in self.bonuses
        self.score_publisher.publish(bonus)

    def add(self, puck):
        assert puck in self.pucks

        if len(self.collected_pucks.get()) == 0:
            self.collected_pucks.set(puck)
            # self.collected_pucks.get().append(puck)
            rospy.loginfo("you added " + puck)
        else:
            self.collected_pucks.set(self.collected_pucks.get() + " " + puck)

        rospy.loginfo("inside " + str(self.collected_pucks.get()))
        rospy.loginfo(" ")

    def unload(self, place):  # side="top"
        assert place in self.places
        lifo_puck = None

        try:
            if len(self.collected_pucks.get()) == 0:
                rospy.loginfo("you tried unloading a puck, but there are no pucks left ")

            elif len(self.collected_pucks.get().split()) == 1:
                lifo_puck = self.collected_pucks.get()
                rospy.loginfo('Unloaded lifo: ' + str(lifo_puck) + " on " + place)
                self.score_publisher.publish(lifo_puck + "_ON_" + place)
                self.collected_pucks.set(str())

            elif len(self.collected_pucks.get().split()) > 1:
                lifo_puck = self.collected_pucks.get().split()[-1]
                self.collected_pucks.set(self.collected_pucks.get().rsplit(' ', 1)[0])  # most right word
                rospy.loginfo('Unloaded lifo: ' + str(lifo_puck) + " on " + place)
                self.score_publisher.publish(lifo_puck + "_ON_" + place)

        except AttributeError as Error:  # FIXME
            rospy.loginfo("se lia vi, error")

        pucks_to_unload = str(self.collected_pucks.get().split())
        quantity = len(self.collected_pucks.get().split())
        rospy.loginfo('Pucks to unload: ' + str(quantity) + " " + pucks_to_unload)

    # if side == "top":
    # elif side == "bottom":
    #     fifo_puck = self.collected_pucks.get()[0]
    #     print(fifo_puck)
    #     self.score_publisher.publish(fifo_puck + "_ON_" + place)
    #
