#!/usr/bin/env python
import rospy
import behavior_tree as bt
import bt_ros
from std_msgs.msg import String
import numpy as np

# import tf2_ros
# from tf.transformations import euler_from_quaternion
# from threading import Lock
# from manipulator import Manipulator
# from tactics_math import *
# from core_functions import cvt_local2global
# from visualization_msgs.msg import MarkerArray

# ====================================

# parallel2 = bt.ParallelNode([bt.SequenceNode([move_2_back,move_2_back2]), complete_take_2_puck], threshold=2)

# calibrate
# move to first puck on the field and collect it
# move to second puck on the field and collect it
# move to third puck on the field and KEEP IT UP, NOT COLLECT!
# move to blunium on acc, prepare manipulator with puck to push it
# push blunium and and finish collect puck in manip
# FIXME move side and unload pucks in acc
# approach goldenium, grab it and push up
# move to scales and unload Goldenium

# ====================================

# class CollectChaosPucks(bt.FallbackNode):
#     def __init__(self, move_client, manipulator_client):
#
#         # Init parameters
#         self.pucks = bt.BTVariable([])
#         self.is_observed = bt.BTVariable(False)
#
#         rospy.Subscriber("/pucks", MarkerArray, self.pucks_callback, queue_size=1)
#
#         # Init useful child nodes
#         self.move_to_waypoint_node = bt_ros.ActionClientNode("move 0 0 0", move_client, name="move_to_waypoint")
#         self.move_to_waypoint_node = bt_ros.ActionClientNode("move 0 0 0", manipulator_client, name="move_to_waypoint")
#         self.choose_new_waypoint_latch = bt.Latch(bt.ActionNode(self.choose_new_waypoint))
#
#         # Make BT
#         super(CollectChaosPucks, self).__init__([
#             bt.ConditionNode(self.is_chaos_collected_completely),
#             bt.SequenceNode([
#                 bt.SequenceWithMemoryNode([
#                     bt.FallbackNode([
#                         bt.ActionNode(self.get_chaos_observation_from_camera)  # FIXME
#                     ]),
#                     bt.ConditionNode(self.is_chaos_observed), # return RUNNING when not observed
#
#                     bt.FallbackWithMemoryNode([
#                         bt.ConditionNode(self.is_not_first_puck),
#                         bt.SequenceWithMemoryNode([
#                             bt.ActionNode(self.calculate_pucks_configuration),
#                             bt.ActionNode(self.calculate_landings),
#                             bt_ros.MoveLineToPoint(prelanding, "move_client"),  # approach_nearest_prelanding
#                             bt_ros.MoveLineToPoint(landings[0], "move_client"),
#                         ]),
#
#                     ]),
#
#                     bt_ros.StartCollectGround("manipulator_client"),  # self.start_suck()
#
#                     bt.FallbackWithMemoryNode([
#                         bt.ConditionNode(self.is_safe_away_from_other_pucks_to_suck),
#                         bt.SequenceNode([
#                             bt_ros.MoveLineToPoint(drive_back_point, "move_client")  # self.drive_back()
#                         ])
#                     ]),
#
#                     bt.ParallelNode([
#                         bt_ros.CompleteCollectGround("manipulator_client"),
#
#                         bt.FallbackWithMemoryNode([
#                             bt.ConditionNode(self.is_last_puck),
#                             bt.SequenceWithMemoryNode([
#                                 bt.ActionNode(self.calculate_pucks_configuration),
#                                 bt.ActionNode(self.calculate_landings)
#                                 bt_ros.MoveArcToPoint(prelanding, "move_client"),  # approach_nearest_prelanding
#                                 bt_ros.MoveLineToPoint(landings[0], "move_client"),  # approach_nearest_landing
#                             ])
#                         ])
#                     ], threshold=2),  # FIXME
#                 ]),
#                 bt.ConditionNode(lambda: bt.Status.RUNNING),
#             ])
#         ])
#
#     def pucks_callback(self, data):
#         self.pucks.set(data)  # Action
#         self.is_observed.set(True)
#
#     def is_chaos_observed(self):  # Condition
#         if self.is_observed.get():
#             return bt.Status.SUCCESS
#         else:
#             return bt.Status.RUNNING


class MainRobotBT(object):
    # noinspection PyTypeChecker
    def __init__(self):
        rospy.init_node("CHAOS_BT")

        rospy.Subscriber("navigation/response", String, self.move_client.response_callback)
        rospy.Subscriber("manipulator/response", String, self.manipulator_client.response_callback)

        self.move_publisher = rospy.Publisher("navigation/command", String, queue_size=100)
        self.manipulator_publisher = rospy.Publisher("manipulator/command", String, queue_size=100)

        self.move_client = bt_ros.ActionClient(self.move_publisher)
        self.manipulator_client = bt_ros.ActionClient(self.manipulator_publisher)

        self.approach_dist = rospy.get_param("approach_dist")  # 0.127 meters, distance from robot to puck where robot will try to grab it
        self.approach_dist = np.array(self.approach_dist)
        self.approach_vec = np.array([-1*self.approach_dist, 0, 0])  # 0.11

        # self.drive_back_dist = rospy.get_param("drive_back_dist")  # 0.04
        # self.drive_back_dist = np.array(self.drive_back_dist)
        # self.drive_back_vec = np.array([-1*self.drive_back_dist, 0, 0])

        # FIXME change to if zone == "orange" then

        self.start_zone = "purple"
        # bt.BTVariable()
        if self.start_zone == "purple":

            self.red_cell_puck = rospy.get_param("red_cell_puck")
            self.green_cell_puck = rospy.get_param("green_cell_puck")
            self.blue_cell_puck = rospy.get_param("blue_cell_puck")

            self.first_puck_landing = np.array([self.red_cell_puck[0]-self.approach_dist,
                                               self.red_cell_puck[1],
                                               0])

            self.second_puck_landing = np.array([self.green_cell_puck[0],
                                                self.green_cell_puck[1]-self.approach_dist,
                                                1.57])

            self.third_puck_landing = np.array([self.blue_cell_puck[0],
                                               self.blue_cell_puck[1]-self.approach_dist,
                                               1.57])

            self.blunium_start_push_pos = rospy.get_param("blunium_start_push_pos")
            self.blunium_finish_push_pos = rospy.get_param("blunium_finish_push_pos")
            self.accelerator_unloading_pos = rospy.get_param("accelerator_unloading_pos")
            self.goldenium_grab_pos = rospy.get_param("goldenium_grab_pos")
            self.scales_unloading_pos = rospy.get_param("scales_unloading_pos")

        rospy.sleep(2)

        # self.bt = bt.Root(CollectChaosPucks("move_client", "manipulator_client"),
        #                   action_clients={"move_client": self.move_client, "manipulator_client": self.manipulator_client})

        self.bt = bt.Root(
            bt.SequenceWithMemoryNode([
                bt_ros.SetToDefaultState("manipulator_client"),

                bt_ros.MoveLineToPoint(self.first_puck_landing, "move_client"),
                bt_ros.StartCollectGround("manipulator_client"),
                bt.ParallelNode([
                    bt_ros.CompleteCollectGround("manipulator_client"),
                    bt_ros.MoveLineToPoint(self.second_puck_landing, "move_client"),
                ], threshold=2),

                bt_ros.StartCollectGround("manipulator_client"),
                bt.ParallelNode([
                    bt_ros.CompleteCollectGround("manipulator_client"),
                    bt_ros.MoveLineToPoint(self.third_puck_landing, "move_client"),
                ], threshold=2),

                bt_ros.StartCollectGround("manipulator_client"),
                bt.ParallelNode([
                    bt_ros.PuckUpAndHold("manipulator_client"),
                    bt_ros.MoveLineToPoint(self.blunium_start_push_pos, "move_client"),
                ], threshold=2),

                bt_ros.SetAngleToPushBlunium("manipulator_client"),
                bt_ros.MoveLineToPoint(self.blunium_finish_push_pos, "move_client"),

                bt.ParallelNode([
                    bt_ros.CompleteCollectGround("manipulator_client"),
                    # FIXME Sasha have to fix height of unloading mechanism
                    bt_ros.MoveLineToPoint(self.accelerator_unloading_pos, "move_client"),
                ], threshold=2),

                bt_ros.UnloadAccelerator("manipulator_client"),
                bt_ros.UnloadAccelerator("manipulator_client"),
                bt_ros.UnloadAccelerator("manipulator_client"),
                bt_ros.UnloadAccelerator("manipulator_client"),

                bt_ros.MoveLineToPoint(self.goldenium_grab_pos, "move_client"),
                bt_ros.GrabGoldeniumAndHoldUp("manipulator_client"),
                bt_ros.MoveLineToPoint(self.scales_unloading_pos, "move_client"),
                bt_ros.UnloadGoldenium("manipulator_client")
            ]),
            action_clients={"move_client": self.move_client, "manipulator_client": self.manipulator_client})

        self.bt_timer = rospy.Timer(rospy.Duration(0.1), self.timer_callback)

    def timer_callback(self, event):
        status = self.bt.tick()
        if status != bt.Status.RUNNING:
            self.bt_timer.shutdown()
        print("============== BT LOG ================")
        self.bt.log(0)


if __name__ == '__main__':
    try:
        tree = MainRobotBT()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
