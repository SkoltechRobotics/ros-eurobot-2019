#!/usr/bin/env python

import rospy
import numpy as np
import behavior_tree as bt
import bt_ros
import tf2_ros
from tf.transformations import euler_from_quaternion
from core_functions import *
from std_msgs.msg import String
from tactics_math import *
from score_controller import ScoreController
from visualization_msgs.msg import MarkerArray
# from collect_chaos import CollectChaos
from main_robot_bt import MainRobotBT
from bt_controller import SideStatus, BTController
from shapely.geometry import Point
from shapely.geometry.polygon import Polygon



# class PushBluniumStrategy(Strategy):
#     def __init__(self, side):
#         super(PushBluniumStrategy, self).__init__()
#
#         if side == SideStatus.PURPLE:
#             self.param = "purple_side"
#             self.side_sign = -1
#         elif side == SideStatus.YELLOW:
#             self.param = "yellow_side"
#             self.side_sign = 1
#
#         red_cell_puck = bt.SequenceWithMemoryNode([
#                             bt_ros.MoveLineToPoint(self.first_puck_landing, "move_client"),
#                             bt.FallbackWithMemoryNode([
#                                 bt.SequenceWithMemoryNode([
#                                     bt_ros.BlindStartCollectGround("manipulator_client"),
#                                     bt.ActionNode(lambda: self.score_master.add("REDIUM")),  # FIXME: color is undetermined without camera!
#                                     bt.ParallelWithMemoryNode([
#                                         bt_ros.CompleteCollectGround("manipulator_client"),
#                                         bt_ros.MoveLineToPoint(self.tactics.first_puck_landing_finish, "move_client"),
#                                     ], threshold=2),
#                                 ]),
#                                 bt.ParallelWithMemoryNode([
#                                     bt_ros.SetManipulatortoWall("manipulator_client"),
#                                     bt_ros.MoveLineToPoint(self.tactics.first_puck_landing_finish, "move_client"),
#                                 ], threshold=2),
#                             ]),
#                         ])
#
#         green_cell_puck = bt.SequenceWithMemoryNode([
#                             bt_ros.MoveLineToPoint(self.tactics.second_puck_landing, "move_client"),
#                             bt.FallbackWithMemoryNode([
#                                 bt.SequenceWithMemoryNode([
#                                     bt_ros.BlindStartCollectGround("manipulator_client"),
#                                     bt.ActionNode(lambda: self.score_master.add("REDIUM")),  # FIXME: color is undetermined without camera!
#                                     bt.ParallelWithMemoryNode([
#                                         bt_ros.CompleteCollectGround("manipulator_client"),
#                                         bt_ros.MoveLineToPoint(self.tactics.third_puck_landing, "move_client"),
#                                     ], threshold=2),
#                                 ]),
#                                 bt.ParallelWithMemoryNode([
#                                     bt_ros.SetManipulatortoWall("manipulator_client"),
#                                     bt_ros.MoveLineToPoint(self.tactics.third_puck_landing, "move_client"),
#                                 ], threshold=2),
#                             ])
#                         ])
#
#         blue_cell_puck = bt.SequenceWithMemoryNode([
#                             bt.FallbackWithMemoryNode([
#                                 bt.SequenceWithMemoryNode([
#                                     bt_ros.BlindStartCollectGround("manipulator_client"),
#                                     bt.ActionNode(lambda: self.score_master.add("REDIUM")),  # FIXME: color is undetermined without camera!
#                                     bt.ParallelWithMemoryNode([
#                                         bt_ros.CompleteCollectGround("manipulator_client"),
#                                         bt_ros.MoveLineToPoint(self.tactics.third_puck_rotate_pose, "move_client"),
#                                     ], threshold=2),
#                                 ]),
#                                 bt.ParallelWithMemoryNode([
#                                     bt_ros.SetManipulatortoUp("manipulator_client"),  # FIXME when adding chaos
#                                     bt_ros.MoveLineToPoint(self.tactics.third_puck_rotate_pose, "move_client"),
#                                 ], threshold=2),
#                             ])
#                         ])
#
#         finish_move_blunium_and_push = bt.SequenceWithMemoryNode([
#                                             bt_ros.MoveLineToPoint(self.tactics.blunium_start_push_pose, "move_client"),
#                                             bt.ParallelWithMemoryNode([
#                                                 bt_ros.MainSetManipulatortoGround("manipulator_client"),  # FIXME when adding chaos
#                                                 bt_ros.MoveLineToPoint(self.tactics.blunium_start_push_pose, "move_client"),
#                                             ], threshold=2),
#                                             # bt_ros.MoveLineToPoint(self.tactics.blunium_prepose, "move_client"),
#                                             bt_ros.MoveLineToPoint(self.tactics.blunium_end_push_pose, "move_client"),
#                                             bt.ActionNode(lambda: self.score_master.add("BLUNIUM")),
#                                             bt.ActionNode(lambda: self.score_master.unload("ACC")),
#                                             bt.ActionNode(lambda: self.score_master.reward("UNLOCK_GOLDENIUM_BONUS")),
#                                         ])
#
#         approach_acc = bt.SequenceWithMemoryNode([
#                             bt_ros.MoveLineToPoint(self.tactics.blunium_get_back_pose, "move_client"),
#                             bt_ros.MoveLineToPoint(self.tactics.accelerator_PREunloading_pos, "move_client"),  # FIXME try Arc
#                             bt_ros.SetSpeedSTM([0, -0.1, 0], 0.6, "stm_client"),
#                         ])
#
#         push_unload_first_in_acc = bt.SequenceWithMemoryNode([
#                                     bt_ros.StepperUp("manipulator_client"),  # FIXME do we need to do that? NO if all 7 pucks inside
#                                     bt_ros.UnloadAccelerator("manipulator_client"),
#                                     bt.ActionNode(lambda: self.score_master.unload("ACC")),
#                                 ])
#
#         unload_acc = bt.SequenceNode([
#                         bt.FallbackNode([
#                             bt.ConditionNode(self.is_robot_empty),
#                             bt.SequenceWithMemoryNode([
#                                 bt_ros.UnloadAccelerator("manipulator_client"),
#                                 bt.ActionNode(lambda: self.score_master.unload("ACC")),
#                             ])
#                         ]),
#                         bt.ConditionNode(self.is_robot_empty_1)
#                     ])
#
#         collect_goldenium = bt.SequenceWithMemoryNode([
#                                 bt_ros.MoveLineToPoint(self.tactics.goldenium_1_PREgrab_pos, "move_client"),
#                                 bt_ros.MoveLineToPoint(self.tactics.goldenium_2_PREgrab_pos, "move_client"),
#                                 bt_ros.StartCollectGoldenium("manipulator_client"),
#                                 bt_ros.MoveLineToPoint(self.tactics.goldenium_grab_pos, "move_client"),
#                                 bt_ros.GrabGoldeniumAndHoldUp("manipulator_client"),
#                                 bt.ActionNode(lambda: self.score_master.add("GOLDENIUM")),
#                                 bt.ActionNode(lambda: self.score_master.reward("GRAB_GOLDENIUM_BONUS")),
#                             ])
#
#         # collect_goldenium = bt.SequenceWithMemoryNode([
#         #                         bt_ros.MoveLineToPoint(self.tactics.goldenium_1_PREgrab_pos, "move_client"),
#         #                         bt_ros.MoveLineToPoint(self.tactics.goldenium_2_PREgrab_pos, "move_client"),
#         #                         bt_ros.StartCollectGoldenium("manipulator_client"),
#
#         #                         bt.ParallelWithMemoryNode([
#         #                             bt_ros.MoveLineToPoint(self.tactics.goldenium_grab_pos, "move_client"),
#         #                             bt_ros.CheckLimitSwitchInf("manipulator_client")
#         #                         ], threshold=1),
#
#         #                         bt_ros.GrabGoldeniumAndHoldUp("manipulator_client"),
#         #                         bt.ActionNode(lambda: self.score_master.add("GOLDENIUM")),
#         #                         bt.ActionNode(lambda: self.score_master.reward("GRAB_GOLDENIUM_BONUS")),
#         #                     ])
#
#         move_to_goldenium_prepose = bt.SequenceWithMemoryNode([
#                                         bt_ros.MoveLineToPoint(self.tactics.goldenium_back_rot_pose, "move_client"),
#                                         bt_ros.MoveLineToPoint(self.tactics.scales_goldenium_PREpos, "move_client")
#                                     ])
#
#         unload_goldenium = bt.SequenceWithMemoryNode([
#                                 bt.ConditionNode(self.is_scales_landing_free),
#                                 bt.SequenceWithMemoryNode([
#                                     bt_ros.MoveLineToPoint(self.tactics.scales_goldenium_pos + np.array([0, -0.05, 0]), "move_client"),
#                                     bt.ActionNode(lambda: self.score_master.unload("SCALES")),
#                                     bt_ros.SetManipulatortoWall("manipulator_client"),
#                                     bt_ros.MoveLineToPoint(self.tactics.scales_goldenium_pos, "move_client"),
#                                     bt_ros.UnloadGoldenium("manipulator_client"),
#                                 ])
#                             ])
#
#         self.tree = bt.SequenceWithMemoryNode([
#                                                 red_cell_puck,
#                                                 green_cell_puck,
#                                                 blue_cell_puck,
#                                                 finish_move_blunium_and_push,
#                                                 approach_acc,
#                                                 push_unload_first_in_acc,
#                                                 unload_acc,
#                                                 collect_goldenium,
#                                                 move_to_goldenium_prepose,
#                                                 unload_goldenium,
#                                                 ])
#



# class OptimalStrategy(Strategy):
#     def __init__(self, side):
#         super(OptimalStrategy, self).__init__()

#         if side == SideStatus.PURPLE:
#             self.param = "purple_side"
#             self.side_sign = -1
#         elif side == SideStatus.YELLOW:
#             self.param = "yellow_side"
#             self.side_sign = 1

#         rospy.sleep(15)
#         self.tree = bt.FallbackWithMemoryNode([
#                         bt.SequenceNode([
#                             bt.ConditionNode(self.is_observed),
#                             CollectChaos(self.known_chaos_pucks.get(), "move_client")
#                         ]),
#                         bt.ConditionNode(lambda: bt.Status.RUNNING)
#                     ])


# class GreedyStrategy(Strategy):
#     def __init__(self):
#         super(GreedyStrategy, self).__init__()
