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

    def score_master(self, cmd, *args, **kwargs):
        """
        Examples:
        (add, "REDIUM")
        (unload, "SCALES")
        (unload, "RED", side="bottom")
        (reward, "OPEN_GOLDENIUM_BONUS")

        # TODO Add condition node before updating score
        # TODO if puck wasn't taken, DON't MAKE STEP DOWN

        :param cmd: add, unload, reward
        :param args: color of puck, where to unload
        :param kwargs: unload from top or from bottom (only for Main robot)
        :return:
        """

        if cmd == "add":
            bt.ActionNode(lambda: self.add(args))
            print("collected_pucks are: ", self.collected_pucks)

        elif cmd == "unload":
            bt.ActionNode(lambda: self.unload(args))
            print("aafter unloading pucks are: ", self.collected_pucks)

        elif cmd == "reward":
            bt.ActionNode(lambda: self.reward(args))

    def reward(self, bonus):
        assert bonus in self.bonuses
        self.score_publisher.publish(bonus)

    def add(self, puck):
        assert puck in self.pucks
        self.collected_pucks.set(self.collected_pucks.get().append(puck))

    def unload(self, place):  # side="top"
        assert place in self.places
        lifo_puck = self.collected_pucks.get()[-1]
        print(lifo_puck)
        self.score_publisher.publish(lifo_puck + "_ON_" + place)


    # if side == "top":
    # elif side == "bottom":
    #     fifo_puck = self.collected_pucks.get()[0]
    #     print(fifo_puck)
    #     self.score_publisher.publish(fifo_puck + "_ON_" + place)
    #

    # def is_puck_grabbed(self):
    #     if self.is_puck_grabbed_flag is True:
    #         return bt.Status.SUCCESS
    #     else:
    #         return bt.Status.FAILED

    # bt.FallbackWithMemoryNode([
    #     bt.SequenceWithMemoryNode([
    #         bt.ConditionNode(self.is_puck_grabbed),
    #         bt.ActionNode(lambda: self.score_master.add(args)),
    #     bt.ConditionNode(lambda: bt.Status.SUCCESS)
    #     ]),
    # ])
