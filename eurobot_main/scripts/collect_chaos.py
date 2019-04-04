#!/usr/bin/env python
import rospy
import behavior_tree as bt
import bt_ros
from std_msgs.msg import String
from bt_controller import SideStatus, BTController
from score_controller import ScoreController

import tf2_ros
from tf.transformations import euler_from_quaternion
from threading import Lock
from tactics_math import *
from core_functions import cvt_local2global
from visualization_msgs.msg import MarkerArray


class CollectChaosPucks(bt.FallbackNode):
    def __init__(self, move_client, manipulator_client):

        # Init parameters
        self.pucks = bt.BTVariable([])
        self.is_observed = bt.BTVariable(False)

        rospy.Subscriber("/pucks", MarkerArray, self.pucks_callback, queue_size=1)

        # Init useful child nodes
        # self.move_to_waypoint_node = bt_ros.ActionClientNode("move 0 0 0", move_client, name="move_to_waypoint")
        # self.move_to_waypoint_node = bt_ros.ActionClientNode("move 0 0 0", manipulator_client, name="move_to_waypoint")
        # self.choose_new_waypoint_latch = bt.Latch(bt.ActionNode(self.choose_new_waypoint))

        # Make BT
        super(CollectChaosPucks, self).__init__([
            bt.ConditionNode(self.is_chaos_collected_completely),
            bt.SequenceNode([
                bt.SequenceWithMemoryNode([
                    bt.FallbackNode([
                        bt.ActionNode(self.get_chaos_observation_from_camera)  # FIXME
                    ]),
                    bt.ConditionNode(self.is_chaos_observed), # return RUNNING when not observed

                    bt.FallbackWithMemoryNode([
                        bt.ConditionNode(self.is_not_first_puck),
                        bt.SequenceWithMemoryNode([
                            bt.ActionNode(self.calculate_pucks_configuration),
                            bt.ActionNode(self.calculate_landings),
                            bt_ros.MoveLineToPoint(prelanding, "move_client"),  # approach_nearest_prelanding
                            bt_ros.MoveLineToPoint(landings[0], "move_client"),
                        ]),

                    ]),

                    bt_ros.StartCollectGround("manipulator_client"),  # self.start_suck()

                    bt.FallbackWithMemoryNode([
                        bt.ConditionNode(self.is_safe_away_from_other_pucks_to_suck),
                        bt.SequenceNode([
                            bt_ros.MoveLineToPoint(drive_back_point, "move_client")  # self.drive_back()
                        ])
                    ]),

                    bt.ParallelNode([
                        bt_ros.CompleteCollectGround("manipulator_client"),

                        bt.FallbackWithMemoryNode([
                            bt.ConditionNode(self.is_last_puck),
                            bt.SequenceWithMemoryNode([
                                bt.ActionNode(self.calculate_pucks_configuration),
                                bt.ActionNode(self.calculate_landings)
                                bt_ros.MoveArcToPoint(prelanding, "move_client"),  # approach_nearest_prelanding
                                bt_ros.MoveLineToPoint(landings[0], "move_client"),  # approach_nearest_landing
                            ])
                        ])
                    ], threshold=2),  # FIXME
                ]),
                bt.ConditionNode(lambda: bt.Status.RUNNING),
            ])
        ])

    def pucks_callback(self, data):
        self.pucks.set(data)  # Action
        self.is_observed.set(True)

    def is_chaos_observed(self):  # Condition
        if self.is_observed.get():
            return bt.Status.SUCCESS
        else:
            return bt.Status.RUNNING

# add in MainRobotBT when collect_chaos ready
