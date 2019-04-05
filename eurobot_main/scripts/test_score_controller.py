#!/usr/bin/env python

import threading
# from behavior_tree import BTVariable
from std_msgs.msg import String
import numpy as np

class BTVariable(object):
    def __init__(self, default_data=None):
        self.data = default_data
        self.mutex = threading.Lock()

    def set(self, data):
        with self.mutex:
            self.data = data

    def get(self):
        with self.mutex:
            data = self.data
        return data

    def my_append(self, data):
        with self.mutex:
            self.data.append(data)


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
        #print "adding ", puck
        if len(self.collected_pucks.get()) == 0:
            self.collected_pucks.set(puck)
        else:
            self.collected_pucks.set(self.collected_pucks.get() + " " + puck)

        #print "inside: ", self.collected_pucks.get()
        #print " "

    def unload(self, place):  # side="top"
        assert place in self.places
        if len(self.collected_pucks.get().split()) > 1:
            lifo_puck = self.collected_pucks.get().split()[-1]
            self.collected_pucks.set(self.collected_pucks.get().rsplit(' ', 1)[0])
            # print "unloading lifo ", lifo_puck, "in ", place

        elif len(self.collected_pucks.get().split()) == 1:
            lifo_puck = self.collected_pucks.get()
            # print "unloading lifo ", lifo_puck, "in ", place
            self.collected_pucks.set(None)

        # self.score_publisher.publish(lifo_puck + "_ON_" + place)
        # print "aafter unloading pucks are: ", self.collected_pucks.get()
        # print " "

    # if side == "top":
    # elif side == "bottom":
    #     fifo_puck = self.collected_pucks.get()[0]
    #     print(fifo_puck)
    #     self.score_publisher.publish(fifo_puck + "_ON_" + place)
    #

if __name__ == '__main__':
    collected_pucks = BTVariable(np.array([]))
    score_master = ScoreController(collected_pucks)
    score_master.add("BLUNIUM")
    score_master.add("REDIUM")
    score_master.add("GREENIUM")
    score_master.unload("ACC")
    score_master.add("GOLDENIUM")
    score_master.unload("SCALES")
    score_master.unload("ACC")
    score_master.unload("ACC")
    #print " "
    #print "inside main", list(collected_pucks.get())
