#!/usr/bin/env python

import threading
# from eurobot_main.scripts.behavior_tree import BTVariable
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


class ScoreController(object):
    def __init__(self, collected_pucks):
        self.collected_pucks = collected_pucks
        self.pucks = ["REDIUM", "GREENIUM", "BLUNIUM", "GOLDENIUM"]
        self.places = ["RED", "GREEN", "BLUE", "ACC", "SCALES"]
        self.bonuses = ["UNLOCK_GOLDENIUM_BONUS", "GRAB_GOLDENIUM_BONUS"]
        self.collected = None

    def reward(self, bonus):
        assert bonus in self.bonuses
        print "you get a ", bonus, predict.get_points(bonus)

    def add(self, puck):
        assert puck in self.pucks
        print "adding ", puck
        # self.collected_pucks.get().append(puck)
        self.collected_pucks.set(np.append(self.collected_pucks.get(), puck))  # Action

        print "inside: ", self.collected_pucks.get()
        print " "

    def unload(self, place):  # side="top"
        assert place in self.places

        if self.collected_pucks.get().size == 0:
            print "you tried unloading a puck, but there are no pucks left"
            print " "
        else:
            lifo_puck = self.collected_pucks.get()[-1]
            self.collected_pucks.set(self.collected_pucks.get()[:-1])
            print 'Unloaded lifo: ' + str(lifo_puck) + " on " + place
            print "you receive: ", predict.get_points(lifo_puck + "_ON_" + place)
            print " "

        print 'Pucks to unload: ' + str(self.collected_pucks.get().size) + " " + str(self.collected_pucks.get())
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
    score_master.unload("ACC")
    score_master.add("BLUNIUM")
    score_master.add("REDIUM")
    score_master.add("GREENIUM")
    score_master.unload("ACC")
    score_master.add("GOLDENIUM")
    score_master.unload("SCALES")
    score_master.unload("ACC")
    score_master.unload("ACC")
    score_master.unload("ACC")
    score_master.add("GOLDENIUM")
    score_master.reward("UNLOCK_GOLDENIUM_BONUS")
    print " "
    print "inside main"
    if collected_pucks.get().size == 0:
        print type(collected_pucks.get())
        print 'All pucks unloaded'
    else:
        print 'Pucks to unload: ' + str(collected_pucks.get())
