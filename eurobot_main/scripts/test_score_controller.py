#!/usr/bin/env python

import threading
from behavior_tree import BTVariable
from std_msgs.msg import String
import numpy as np


class ScoreController(object):
    def __init__(self, collected_pucks):
        self.collected_pucks = collected_pucks
        self.pucks = ["REDIUM", "GREENIUM", "BLUNIUM", "GOLDENIUM"]
        self.places = ["RED", "GREEN", "BLUE", "ACC", "SCALES"]
        self.bonuses = ["UNLOCK_GOLDENIUM_BONUS", "GRAB_GOLDENIUM_BONUS"]
        # self.score_publisher = rospy.Publisher("score", String, queue_size=100)
        self.collected = None
        # self.is_puck_grabbed_flag = False

    def reward(self, bonus):
        assert bonus in self.bonuses
        # self.score_publisher.publish(bonus)

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
        if len(self.collected_pucks.get().split()) > 1:
            lifo_puck = self.collected_pucks.get().split()[-1]
            self.collected_pucks.set(self.collected_pucks.get().rsplit(' ', 1)[0])
            print "unloading lifo ", lifo_puck, "in ", place

        elif len(self.collected_pucks.get().split()) == 1:
            lifo_puck = self.collected_pucks.get()
            print "unloading lifo ", lifo_puck, "in ", place
            self.collected_pucks.set([])

        # self.score_publisher.publish(lifo_puck + "_ON_" + place)
        print "aafter unloading pucks are: ", self.collected_pucks.get()
        print " "

    # if side == "top":
    # elif side == "bottom":
    #     fifo_puck = self.collected_pucks.get()[0]
    #     print(fifo_puck)
    #     self.score_publisher.publish(fifo_puck + "_ON_" + place)
    #

class Prediction:
    def __init__(self):

        self.puck_points = {
            "REDIUM_ON_RED": 6,
            "REDIUM_ON_OTHER": 1,  # FIXME

            "GREENIUM_ON_GREEN": 6,
            "GREENIUM_ON_OTHER": 1,  # FIXME

            "BLUNIUM_ON_BLUE": 6,
            "BLUNIUM_ON_OTHER": 1,  # FIXME

            "GOLDENIUM_ON_ANY": 6,

            "REDIUM_ON_ACC": 10,
            "GREENIUM_ON_ACC": 10,
            "BLUNIUM_ON_ACC": 10,

            "UNLOCK_GOLDENIUM_BONUS": 10,
            "GRAB_GOLDENIUM_BONUS": 20,

            "REDIUM_ON_SCALES": 4,
            "GREENIUM_ON_SCALES": 8,
            "BLUNIUM_ON_SCALES": 12,
            "GOLDENIUM_ON_SCALES": 24,
        }

    def get_points(self, key):
        return self.puck_points.get(key)


if __name__ == '__main__':
    collected_pucks = BTVariable(np.array([]))
    predict = Prediction()
    score_master = ScoreController(collected_pucks)
    score_master.add("BLUNIUM")
    score_master.add("REDIUM")
    score_master.add("GREENIUM")
    score_master.unload("ACC")
    score_master.add("GOLDENIUM")
    score_master.unload("SCALES")
    score_master.unload("ACC")
    score_master.unload("ACC")
    print " "
    print "inside main", len(collected_pucks.get())
    print predict.get_points("REDIUM_ON_SCALES")
