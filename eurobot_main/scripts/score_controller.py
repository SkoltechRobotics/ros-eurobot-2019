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
        self.score_master = bt_ros.ScoreMaster(self.collected_pucks)
        # self.is_puck_grabbed_flag = False

    def score_controller(self, cmd, *args, **kwargs):
        """
        Examples:
        (add, "REDIUM")
        (unload, "SCALES")
        (unload, "RED", side="bottom")
        (reward, "OPEN_GOLDENIUM_BONUS")

        :param cmd: add, unload, reward
        :param args: color of puck, where to unload
        :param kwargs: unload from top or from bottom (only for Main robot)
        :return:
        """

        # TODO Add condition node before updating score

        # TODO if puck wasn't taken, DON't MAKE STEP DOWN

        if cmd == "add":
            bt.FallbackWithMemoryNode([
                bt.SequenceWithMemoryNode([
                    bt.ConditionNode(self.is_puck_grabbed),
                    bt.ActionNode(lambda: self.score_master.add(args)),
                bt.ConditionNode(lambda: bt.Status.SUCCESS)
                ]),
            ])
        elif cmd == "unload":
            bt.ActionNode(lambda: self.score_master.unload(args, kwargs))
        elif cmd == "reward":
            bt.ActionNode(lambda: self.score_master.reward(args))

    def is_puck_grabbed(self):
        if self.is_puck_grabbed_flag is True:
            return bt.Status.SUCCESS
        else:
            return bt.Status.FAILED
