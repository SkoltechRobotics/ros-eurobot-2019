#!/usr/bin/env python
import rospy
import behavior_tree as bt
import bt_ros
from std_msgs.msg import String
from visualization_msgs.msg import MarkerArray

import tf2_ros
import numpy as np
from tf.transformations import euler_from_quaternion
from threading import Lock
from manipulator import Manipulator
from tactics_math import *
from core_functions import cvt_local2global


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







self.drive_back_dist = rospy.get_param("drive_back_dist")  # 0.04
self.drive_back_dist = np.array(self.drive_back_dist)

self.approach_vec = np.array([-1*self.approach_dist, 0, 0])  # 0.11
self.drive_back_vec = np.array([-1*self.drive_back_dist, 0, 0])

prelanding = cvt_local2global(self.drive_back_vec, self.goal_landing)


class CollectChaosPucks(bt.FallbackNode):
    def __init__(self, move_client, manipulator_client):

        # Init parameters
        self.pucks = bt.BTVariable([])
        self.is_observed = bt.BTVariable(False)

        rospy.Subscriber("/pucks", MarkerArray, self.pucks_callback, queue_size=1)

        # Init useful child nodes
        self.move_to_waypoint_node = bt_ros.ActionClientNode("move 0 0 0", move_client, name="move_to_waypoint")
        self.move_to_waypoint_node = bt_ros.ActionClientNode("move 0 0 0", manipulator_client, name="move_to_waypoint")
        self.choose_new_waypoint_latch = bt.Latch(bt.ActionNode(self.choose_new_waypoint))

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


class MainRobotBT(object):
    # noinspection PyTypeChecker
    def __init__(self):
        rospy.init_node("CHAOS_BT")

        self.approach_dist = rospy.get_param("approach_dist")  # 0.127 meters, distance from robot to puck where robot will try to grab it
        self.approach_dist = np.array(self.approach_dist)

        # FIXME change to if zone == "orange" then
        red_cell_puck = rospy.get_param("red_cell_puck")
        green_cell_puck = rospy.get_param("green_cell_puck")
        blue_cell_puck = rospy.get_param("blue_cell_puck")

        bt.BTVariable()
        # first puck coords + rot, landing!
        if self.start_zone == "purple":
            first_puck_landing = np.array([red_cell_puck[0]-self.approach_dist,
                                           red_cell_puck[1],
                                           0])

            second_puck_landing = np.array([green_cell_puck[0],
                                            green_cell_puck[1]-self.approach_dist,
                                            1.57])

            third_puck_landing = np.array([blue_cell_puck[0],
                                           blue_cell_puck[1]-self.approach_dist,
                                           1.57])

            blunium_start_push_pos =
            blunium_finish_push_pos
            accelerator_unloading_pos
            goldenium_grab_pos
            scales_unloading_pos


        self.move_publisher = rospy.Publisher("navigation/command", String, queue_size=100)
        self.manipulator_publisher = rospy.Publisher("manipulator/command", String, queue_size=100)

        self.move_client = bt_ros.ActionClient(self.move_publisher)
        self.manipulator_client = bt_ros.ActionClient(self.manipulator_publisher)

        rospy.Subscriber("navigation/response", String, self.move_client.response_callback)
        rospy.Subscriber("manipulator/response", String, self.manipulator_client.response_callback)

        rospy.sleep(2)

        # self.bt = bt.Root(CollectChaosPucks("move_client", "manipulator_client"),
        #                   action_clients={"move_client": self.move_client, "manipulator_client": self.manipulator_client})

        self.bt = bt.Root(
            bt.SequenceWithMemoryNode([
                bt_ros.SetToDefaultState("manipulator_client"),

                bt_ros.MoveLineToPoint(first_puck_landing, "move_client"),
                bt_ros.StartCollectGround("manipulator_client"),
                bt.ParallelNode([
                    bt_ros.CompleteCollectGround("manipulator_client"),
                    bt_ros.MoveLineToPoint(second_puck_landing, "move_client"),
                ], threshold=2),

                bt_ros.StartCollectGround("manipulator_client"),
                bt.ParallelNode([
                    bt_ros.CompleteCollectGround("manipulator_client"),
                    bt_ros.MoveLineToPoint(third_puck_landing, "move_client"),
                ], threshold=2),

                bt_ros.StartCollectGround("manipulator_client"),
                bt.ParallelNode([
                    bt_ros.PuckUpAndHold("manipulator_client"),
                    bt_ros.MoveLineToPoint(blunium_start_push_pos, "move_client"),
                ], threshold=2),

                bt_ros.SetAngleToPushBlunium("manipulator_client"),
                bt_ros.MoveLineToPoint(blunium_finish_push_pos, "move_client"),

                bt.ParallelNode([
                    bt_ros.CompleteCollectGround("manipulator_client"),
                    # FIXME Sasha have to fix height of unloading mechanism
                    bt_ros.MoveLineToPoint(accelerator_unloading_pos, "move_client"),
                ], threshold=2),

                bt_ros.UnloadAccelerator("manipulator_client"),
                bt_ros.UnloadAccelerator("manipulator_client"),
                bt_ros.UnloadAccelerator("manipulator_client"),
                bt_ros.UnloadAccelerator("manipulator_client"),

                bt_ros.MoveLineToPoint(goldenium_grab_pos, "move_client"),
                bt_ros.SetAngleToGrabGoldenium("manipulator_client"),
                bt_ros.GoldeniumUpAndHold("manipulator_client"),
                bt_ros.MoveLineToPoint(scales_unloading_pos, "move_client"),
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
