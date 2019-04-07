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
        self.score_publisher = rospy.Publisher("score", String, queue_size=100)

        # self.is_puck_grabbed_flag = False

    def add(self, puck):
        assert puck in self.pucks

        print "adding ", puck
        if len(self.collected_pucks.get()) == 0:
            self.collected_pucks.set(puck)
        else:
            self.collected_pucks.set(self.collected_pucks.get() + " " + puck)
        print "inside: ", self.collected_pucks.get()
        print " "

    def unload(self, place):  # side="top"
        assert place in self.places
        lifo_puck = None

        try:
            if self.collected_pucks.get() == 0:
                rospy.loginfo("there are no pucks ERORR")
            elif len(self.collected_pucks.get().split()) == 1:
                lifo_puck = self.collected_pucks.get()
                rospy.loginfo('Unloaded lifo: ' + str(lifo_puck) + " on " + place)
                self.score_publisher.publish(lifo_puck + "_ON_" + place)
                self.collected_pucks.set(0)
            elif len(self.collected_pucks.get().split()) > 1:
                lifo_puck = self.collected_pucks.get().split()[-1]
                self.collected_pucks.set(self.collected_pucks.get().rsplit(' ', 1)[0])  # most right word
                rospy.loginfo('Unloaded lifo: ' + str(lifo_puck) + " on " + place)
                self.score_publisher.publish(lifo_puck + "_ON_" + place)

        except AttributeError as Error:  # FIXME
            rospy.loginfo("se lia vi, tuple object has no attribute split")
        # rospy.loginfo('Pucks to unload: '+ str(len(self.collected_pucks.get().split())))

    def reward(self, bonus):
        assert bonus in self.bonuses

        self.score_publisher.publish(bonus)
        print "YOU ACHIEVED A BONUS!!!!!"

    # if side == "top":
    # elif side == "bottom":
    #     fifo_puck = self.collected_pucks.get()[0]
    #     print(fifo_puck)
    #     self.score_publisher.publish(fifo_puck + "_ON_" + place)
    #