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


# class CollectChaos(Strategy):
#     def __init__(self, side):
#         super(CollectChaos, self).__init__(side)
#         self.scale_factor = np.array(rospy.get_param("scale_factor"))  # used in calculating outer bissectrisa for hull's angles
#         self.critical_angle = rospy.get_param("critical_angle")
#         self.approach_vec = np.array([-1 * self.HPAD, 0, 0])
#         self.drive_back_dist = np.array(rospy.get_param("drive_back_dist"))  # FIXME
#         self.drive_back_vec = np.array([-1*self.drive_back_dist, 0, 0])
#         self.closest_landing = bt.BTVariable()
#         self.nearest_PRElanding = bt.BTVariable()

#         self.guard_chaos_loc_var = bt.BTVariable(np.array([self.chaos_center[0] - self.sign * 0.3,
#                                                          self.chaos_center[1] - 0.3,
#                                                          1.57 - self.sign * 0.6]))  # FIXME change to another angle and loc * 0.785

#         self.starting_pos_var = bt.BTVariable(np.array([1.5 + self.sign * 1.2,  # y/p 2.7 / 0.3
#                                                         0.45,
#                                                         1.57 + self.sign * 1.57]))  # y/p 3.14 / 0

#         # TODO: add checking if all received coords lie inside chaos zone

        #  self.tree = bt.SequenceWithMemoryNode([

    # def tree(self):
    #     # super(CollectChaos, self).__init__([
    #     strategy = bt.SequenceWithMemoryNode([
    #                     # 1st
    #                     bt_ros.ArcMoveToVariable(self.guard_chaos_loc_var, "move_client"),

    #                     bt.ActionNode(self.calculate_pucks_configuration),
    #                     bt.ActionNode(self.calculate_closest_landing),
    #                     bt.ActionNode(self.calculate_prelanding),

    #                     bt_ros.MoveToVariable(self.nearest_PRElanding, "move_client"),
    #                     bt_ros.ArcMoveToVariable(self.closest_landing, "move_client"),
    #                     bt_ros.BlindStartCollectGround("manipulator_client"),
    #                     bt.ActionNode(self.update_chaos_pucks),
    #                     bt.ActionNode(lambda: self.score_master.add(self.incoming_puck_color.get())),
    #                     bt_ros.MoveToVariable(self.nearest_PRElanding, "move_client"),

    #                     # 2nd
    #                     bt.ActionNode(self.calculate_pucks_configuration),
    #                     bt.ActionNode(self.calculate_closest_landing),
    #                     bt.ActionNode(self.calculate_prelanding),

    #                     bt.ParallelWithMemoryNode([
    #                         bt_ros.CompleteCollectGround("manipulator_client"),
    #                         bt.SequenceWithMemoryNode([
    #                             bt_ros.ArcMoveToVariable(self.nearest_PRElanding, "move_client"),
    #                             bt_ros.MoveToVariable(self.closest_landing, "move_client"),
    #                         ])
    #                     ], threshold=2),

    #                     bt_ros.BlindStartCollectGround("manipulator_client"),
    #                     bt.ActionNode(self.update_chaos_pucks),
    #                     bt.ActionNode(lambda: self.score_master.add(self.incoming_puck_color.get())),
    #                     bt_ros.MoveToVariable(self.nearest_PRElanding, "move_client"),

    #                     # 3rd
    #                     bt.ActionNode(self.calculate_pucks_configuration),
    #                     bt.ActionNode(self.calculate_closest_landing),
    #                     bt.ActionNode(self.calculate_prelanding),

    #                     bt.ParallelWithMemoryNode([
    #                         bt_ros.CompleteCollectGround("manipulator_client"),
    #                         bt.SequenceWithMemoryNode([
    #                             bt_ros.ArcMoveToVariable(self.nearest_PRElanding, "move_client"),
    #                             bt_ros.MoveToVariable(self.closest_landing, "move_client"),
    #                         ])
    #                     ], threshold=2),

    #                     bt_ros.BlindStartCollectGround("manipulator_client"),
    #                     bt.ActionNode(self.update_chaos_pucks),
    #                     bt.ActionNode(lambda: self.score_master.add(self.incoming_puck_color.get())),
    #                     bt_ros.MoveToVariable(self.nearest_PRElanding, "move_client"),

    #                     # 4th
    #                     bt.ActionNode(self.calculate_pucks_configuration),
    #                     bt.ActionNode(self.calculate_closest_landing),

    #                     bt.ParallelWithMemoryNode([
    #                         bt_ros.CompleteCollectGround("manipulator_client"),
    #                         bt_ros.MoveToVariable(self.closest_landing, "move_client"),
    #                     ], threshold=2),

    #                     bt_ros.BlindStartCollectGround("manipulator_client"),
    #                     bt.ActionNode(self.update_chaos_pucks),
    #                     bt.ActionNode(lambda: self.score_master.add(self.incoming_puck_color.get())),

    #                     # back_to_start
    #                     bt.ParallelWithMemoryNode([
    #                         bt.SequenceWithMemoryNode([
    #                             bt_ros.CompleteCollectGround("manipulator_client"),
    #                             bt_ros.StepperUp("manipulator_client"),
    #                             bt_ros.MainSetManipulatortoGround("manipulator_client")
    #                         ]),
    #                         bt_ros.MoveToVariable(self.starting_pos_var, "move_client"),
    #                     ], threshold=2),
    #     ])
    #     return strategy

    # def update_chaos_pucks(self):
    #     """
    #     delete taken puck from known on the field
    #     get color of last taken puck
    #     :return: None
    #     """
    #     incoming_puck_color = get_color(self.known_chaos_pucks.get()[0])
    #     self.incoming_puck_color.set(incoming_puck_color)
    #     rospy.loginfo("incoming_puck_color: " + str(self.incoming_puck_color.get()))
    #     self.known_chaos_pucks.set(np.delete(self.known_chaos_pucks.get(), 0, axis=0))
    #     rospy.loginfo("Known pucks after removing: " + str(self.known_chaos_pucks.get()))

    # def calculate_pucks_configuration(self):
    #     """

    #     :return: # [(0.95, 1.1, 3, 0, 0, 1), ...]
    #     """
    #     while not self.update_main_coords():
    #         print "no coords available"
    #         rospy.sleep(0.5)

    #     known_chaos_pucks = sort_wrt_robot(self.main_coords, self.known_chaos_pucks.get())
    #     print "sorted"
    #     self.known_chaos_pucks.set(known_chaos_pucks)
    #     if len(self.known_chaos_pucks.get()) >= 3:
    #         is_hull_safe_to_approach, coords_sorted_by_angle = sort_by_inner_angle_and_check_if_safe(self.main_coords,
    #                                                                                                  self.known_chaos_pucks.get(),
    #                                                                                                  self.critical_angle)

    #         if not is_hull_safe_to_approach:
    #             self.known_chaos_pucks.set(coords_sorted_by_angle)  # calc vert-angle, sort by angle, return vertices (sorted)
    #             rospy.loginfo("hull is not safe to approach, sorted by angle")
    #         else:  # only sharp angles
    #             rospy.loginfo("hull is SAFE to approach, keep already sorted wrt robot")
    #     rospy.loginfo("Known pucks sorted: " + str(self.known_chaos_pucks.get()))

    # #     when we finally sorted them, chec if one of them is blue. If so, roll it so blue becomes last one to collect
    # #     if self.known_chaos_pucks.get().size > 1 and all(self.known_chaos_pucks.get()[0][3:6] == [0, 0, 1]):
    # #         # self.known_chaos_pucks.set(np.roll(self.known_chaos_pucks.get(), -1, axis=0))
    # #         rospy.loginfo("blue rolled")

    # def calculate_closest_landing(self):
    #     """

    #     :return: [(x, y, theta), ...]
    #     """
    #     if len(self.known_chaos_pucks.get()) == 1:
    #         landings = calculate_closest_landing_to_point(self.main_coords,
    #                                                       self.known_chaos_pucks.get()[:, :2],
    #                                                       self.approach_vec)
    #     else:
    #         landings = unleash_power_of_geometry(self.known_chaos_pucks.get()[:, :2],
    #                                              self.scale_factor,
    #                                              self.HPAD)

    #     self.closest_landing.set(landings[0])
    #     rospy.loginfo("Inside calculate_closest_landing, closest_landing is : ")
    #     print(self.closest_landing.get())
    #     print " "

    # def calculate_prelanding(self):
    #     nearest_PRElanding = cvt_local2global(self.drive_back_vec, self.closest_landing.get())
    #     self.nearest_PRElanding.set(nearest_PRElanding)
    #     rospy.loginfo("Nearest PRElanding calculated: " + str(self.nearest_PRElanding.get()))
    #     print " "


# class Combobombo(Strategy):
#     def __init__(self, side):
#         super(Combobombo, self).__init__(side)
#         chaos = CollectChaos(side)

#         red_cell_puck = bt.SequenceWithMemoryNode([
#                             bt_ros.MoveLineToPoint(self.first_puck_landing, "move_client"),
#                             bt.FallbackWithMemoryNode([
#                                 bt.SequenceWithMemoryNode([
#                                     bt_ros.BlindStartCollectGround("manipulator_client"),
#                                     bt.ActionNode(lambda: self.score_master.add("REDIUM")),  # FIXME: color is undetermined without camera!
#                                     bt.ParallelWithMemoryNode([
#                                         bt_ros.CompleteCollectGround("manipulator_client"),
#                                         bt_ros.MoveLineToPoint(self.first_puck_landing_finish, "move_client"),
#                                     ], threshold=2),
#                                 ]),
#                                 bt.ParallelWithMemoryNode([
#                                     bt_ros.SetManipulatortoWall("manipulator_client"),
#                                     bt_ros.MoveLineToPoint(self.first_puck_landing_finish, "move_client"),
#                                 ], threshold=2)
#                             ]),
#                         ])

#         green_cell_puck = bt.SequenceWithMemoryNode([
#                             bt_ros.MoveLineToPoint(self.second_puck_landing, "move_client"),
#                             bt.FallbackWithMemoryNode([
#                                 bt.SequenceWithMemoryNode([
#                                     bt_ros.BlindStartCollectGround("manipulator_client"),
#                                     bt.ActionNode(lambda: self.score_master.add("REDIUM")),  # FIXME: color is undetermined without camera!
#                                     bt.ParallelWithMemoryNode([
#                                         bt_ros.CompleteCollectGround("manipulator_client"),
#                                         bt_ros.MoveLineToPoint(self.third_puck_landing, "move_client"),
#                                     ], threshold=2),
#                                 ]),
#                                 bt.ParallelWithMemoryNode([
#                                     bt_ros.SetManipulatortoWall("manipulator_client"),
#                                     bt_ros.MoveLineToPoint(self.third_puck_landing, "move_client"),
#                                 ], threshold=2)
#                             ])
#                         ])

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

#         self.tree = bt.SequenceWithMemoryNode([
#                         red_cell_puck,

#                         green_cell_puck,

#                         bt.FallbackWithMemoryNode([
#                             bt.SequenceNode([
#                                 bt.ConditionNode(self.is_observed),
#                                 chaos.tree()
#                             ]),
#                             bt.ConditionNode(lambda: bt.Status.RUNNING)
#                         ]),

#                         unload_acc
#                     ])


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
