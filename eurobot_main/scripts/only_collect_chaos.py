#!/usr/bin/env python

import rospy
import numpy as np
import behavior_tree as bt
import bt_ros
import tf2_ros
from tf.transformations import euler_from_quaternion
from std_msgs.msg import String
from bt_controller import SideStatus, BTController
from score_controller import ScoreController
from core_functions import *
from shapely.geometry import Point
from shapely.geometry.polygon import Polygon
from std_msgs.msg import String

from tactics_math import *
from visualization_msgs.msg import MarkerArray




    # def pucks_callback(self, data):
    #     self.is_observed.set(True)
    #     # [(0.95, 1.1, 3, 0, 0, 1), ...] - blue, id=3  IDs are not guaranteed to be the same from frame to frame
    #     # red (1, 0, 0)
    #     # green (0, 1, 0)
    #     # blue (0, 0, 1)

    #     if self.known_chaos_pucks.get().size == 0:
    #         try:
    #             new_observation_pucks = [[marker.pose.position.x,
    #                                       marker.pose.position.y,
    #                                       marker.id,
    #                                       marker.color.r,
    #                                       marker.color.g,
    #                                       marker.color.b] for marker in data.markers]

    #             self.known_chaos_pucks.set(np.append(self.known_chaos_pucks.get(), new_observation_pucks))  # Action
    #             rospy.loginfo("Got pucks observation:")
    #             rospy.loginfo(self.known_chaos_pucks.get())
    #             self.pucks_subscriber.unregister()

    #         except Exception:  # FIXME
    #             rospy.loginfo("list index out of range - no visible pucks on the field ")

    # def is_chaos_collected_completely(self):
    #     if self.known_chaos_pucks.get().size == 0:
    #         rospy.loginfo("Chaos collected completely")
    #         return bt.Status.SUCCESS
    #     else:
    #         return bt.Status.FAILED

    # # FIXME remove this
    # def is_chaos_collected_completely1(self):
    #     if self.known_chaos_pucks.get().size == 0:
    #         rospy.loginfo("Chaos collected completely")
    #         return bt.Status.SUCCESS
    #     else:
    #         return bt.Status.RUNNING

    # def is_chaos_observed(self):  # Condition
    #     if self.is_observed.get():
    #         return bt.Status.SUCCESS
    #     else:
    #         return bt.Status.RUNNING

    # # FIXME inside
    # def calculate_pucks_configuration(self):
    #     """

    #     :return: # [(0.95, 1.1, 3, 0, 0, 1), ...]
    #     """
    #     self.known_chaos_pucks = sort_wrt_robot(self.main_coords, self.known_chaos_pucks)

    #     if len(self.known_chaos_pucks) >= 3:
    #         is_hull_safe_to_approach, coords_sorted_by_angle = sort_by_inner_angle_and_check_if_safe(self.main_coords,
    #                                                                                                  self.known_chaos_pucks,
    #                                                                                                  self.critical_angle)
    #         if is_hull_safe_to_approach:  # only sharp angles
    #             rospy.loginfo("hull is SAFE to approach, sorted wrt robot")

    #         if not is_hull_safe_to_approach:
    #             known_chaos_pucks = coords_sorted_by_angle  # calc vert-angle, sort by angle, return vertices (sorted)
    #             rospy.loginfo("hull is not safe to approach, sorted by angle")

    #     # when we finally sorted them, chec if one of them is blue. If so, roll it so blue becomes last one to collect
    #     if len(self.known_chaos_pucks) > 1 and all(self.known_chaos_pucks[0][3:6] == [0, 0, 1]):
    #         self.known_chaos_pucks = np.roll(self.known_chaos_pucks, -1, axis=0)
    #         rospy.loginfo("blue rolled")

    # def calculate_landings(self):
    #     """

    #     :return: [(x, y, theta), ...]
    #     """
    #     coords = self.known_chaos_pucks[:, :2]
    #     if len(coords) == 1:
    #         landings = calculate_closest_landing_to_point(self.main_coords, coords, self.approach_vec)
    #         self.sorted_chaos_landings.set(np.append(self.sorted_chaos_landings.get(), landings))
    #     else:
    #         landings = unleash_power_of_geometry(coords, self.scale_factor, self.approach_dist)
    #         self.sorted_chaos_landings.set(np.append(self.sorted_chaos_landings.get(), landings))

    # def choose_new_landing(self):
    #     self.nearest_landing = self.sorted_chaos_landings.get()[0]
    #     rospy.loginfo("Nearest landing chosen: " + str(self.nearest_landing))

    # def calculate_prelanding(self):
    #     self.nearest_PRElanding = cvt_local2global(self.drive_back_vec, self.nearest_landing)
    #     rospy.loginfo("Nearest PRElanding calculated: " + str(self.nearest_landing))

    # def calculate_drive_back_point(self):
    #     self.drive_back_point = cvt_local2global(self.drive_back_vec, self.nearest_landing)
    #     rospy.loginfo("Nearest drive_back_point calculated: " + str(self.drive_back_point))

    # def is_last_puck(self):
    #     if self.known_chaos_pucks.get().size == 1:
    #         return bt.Status.SUCCESS
    #     else:
    #         return bt.Status.RUNNING

    # @staticmethod
    # def get_color(puck):
    #     """
    #     red (1, 0, 0)
    #     green (0, 1, 0)
    #     blue (0, 0, 1)
    #     :param puck: (x, y, id, 0, 0, 1)
    #     :return:
    #     """
    #     pucks_colors = {
    #         (1, 0, 0): "REDIUM",
    #         (0, 1, 0): "GREENIUM",
    #         (0, 0, 1): "BLUNIUM"
    #     }
    #     color_key = puck[3:]
    #     color_val = pucks_colors.get(color_key)
    #     return color_val

    # def update_chaos_pucks(self):
    #     """
    #     delete taken puck from known on the field
    #     get color of last taken puck
    #     :return: None
    #     """
    #     self.incoming_puck_color = self.get_color(self.known_chaos_pucks.get()[0])
    #     self.known_chaos_pucks.set(np.delete(self.known_chaos_pucks.get(), 0, axis=0))


        # move_chaos_to_red = bt.SequenceWithMemoryNode([
        #                         bt_ros.MoveLineToPoint(self.tactics.chaos_push_pose, "move_client"),
        #                         bt.ActionNode(lambda: self.score_master.add("BLUNIUM")),
        #                         bt.ActionNode(lambda: self.score_master.add("GREENIUM")),
        #                         bt.ActionNode(lambda: self.score_master.add("REDIUM")),
        #                         bt.ActionNode(lambda: self.score_master.add("REDIUM")),

        #                         bt_ros.MoveLineToPoint(self.tactics.chaos_in_red_zone, "move_client"),
        #                         bt.ActionNode(lambda: self.score_master.unload("RED")),
        #                         bt.ActionNode(lambda: self.score_master.unload("RED")),
        #                         bt.ActionNode(lambda: self.score_master.unload("RED")),
        #                         bt.ActionNode(lambda: self.score_master.unload("RED"))
        #                     ])

        # calculate and move to first
        # move_to_chaos = bt.SequenceWithMemoryNode([
        #                     bt.ActionNode(self.update_main_coords),
        #                     bt.ActionNode(self.calculate_pucks_configuration),
        #                     bt.ActionNode(self.calculate_landings),
        #                     bt.ActionNode(self.choose_new_landing),
        #                     bt.ActionNode(self.calculate_prelanding),

        #                     bt_ros.MoveLineToPoint(self.nearest_PRElanding, "move_client"),
        #                     bt_ros.MoveLineToPoint(self.nearest_landing, "move_client"),
        #                 ])

        # TODO take into account, that when colecting 7 pucks, don't make step down
        # collect_chaos = bt.SequenceWithMemoryNode([
        #                     bt.FallbackNode([
        #                         # completely?
        #                         bt.ConditionNode(self.is_chaos_collected_completely),

        #                         # if last puck, just collect it, return success and get fuck out of here
        #                         bt.SequenceWithMemoryNode([
        #                             bt.ConditionNode(self.is_last_puck),
        #                             bt.SequenceWithMemoryNode([
        #                                 bt_ros.StartCollectGround("manipulator_client"),
        #                                 bt_ros.CompleteCollectGround("manipulator_client"),
        #                                 bt.ActionNode(self.update_chaos_pucks),
        #                                 bt.ActionNode(lambda: self.score_master.add(self.incoming_puck_color))
        #                             ])
        #                         ]),

        #                         # calc config and start collect
        #                         bt.SequenceWithMemoryNode([
        #                             bt_ros.StartCollectGround("manipulator_client"),
        #                             bt.ActionNode(self.calculate_drive_back_point),
        #                             bt_ros.MoveLineToPoint(self.drive_back_point, "move_client"), # FIXME make it closer

        #                             # calc new landing
        #                             bt.ActionNode(self.calculate_pucks_configuration),
        #                             bt.ActionNode(self.calculate_landings),

        #                             # drive back, collect and move to new prelanding
        #                             bt.ParallelWithMemoryNode([
        #                                 bt_ros.CompleteCollectGround("manipulator_client"),
        #                                 bt.ActionNode(self.update_chaos_pucks),
        #                                 bt.ActionNode(lambda: self.score_master.add(self.incoming_puck_color)),
        #                                 bt_ros.MoveLineToPoint(self.nearest_PRElanding, "move_client"),
        #                             ], threshold=3),

        #                             bt_ros.MoveLineToPoint(self.nearest_landing, "move_client"),
        #                         ]),
        #                     ]),
        #                     bt.ConditionNode(lambda: self.is_chaos_collected_completely1())
        #                 ])
