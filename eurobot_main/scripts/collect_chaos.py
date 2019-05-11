#!/usr/bin/env python

import rospy
import numpy as np
import behavior_tree as bt
import bt_ros
# import tf2_ros
from tf.transformations import euler_from_quaternion
from core_functions import *
# from std_msgs.msg import String
from tactics_math import *
# from score_controller import ScoreController
# from visualization_msgs.msg import MarkerArray
from main_strategies import Strategy


class CollectChaos(Strategy, bt.SequenceWithMemoryNode):
    def __init__(self, side):  # action_client_id
        super(CollectChaos, self).__init__(side)
        self.scale_factor = np.array(rospy.get_param("scale_factor"))  # used in calculating outer bissectrisa for hull's angles
        # self.critical_angle = np.pi * 2/3
        self.critical_angle = rospy.get_param("critical_angle")
        self.approach_vec = np.array([-1 * self.HPAD, 0, 0])
        self.drive_back_dist = np.array(rospy.get_param("drive_back_dist"))  # FIXME
        self.drive_back_vec = np.array([-1*self.drive_back_dist, 0, 0])

        self.guard_chaos_loc_var = bt.BTVariable(np.array([self.chaos_center[0] + self.sign * 0.4,
                                                         self.chaos_center[1] - 0.4,
                                                         1.57 + self.sign * 0.785]))  # FIXME change to another angle and loc

        self.starting_pos_var = bt.BTVariable(np.array([1.5 + self.sign * 1.2,  # y/p 2.7 / 0.3
                                                        0.45,
                                                        1.57 + self.sign * 1.57]))  # y/p 3.14 / 0

        self.closest_landing = bt.BTVariable()
        self.nearest_PRElanding = bt.BTVariable()

        super(CollectChaos, self).__init__([
                # 1st
                bt.ActionNode(self.calculate_pucks_configuration),
                bt.ActionNode(self.calculate_closest_landing),
                bt.ActionNode(self.calculate_prelanding),

                # bt.ParallelWithMemoryNode([
                #     bt_ros.SetManipulatortoUp("manipulator_client"),
                #     bt_ros.MoveToVariable(self.guard_chaos_point, "move_client")
                # ], threshold=2),

                bt_ros.MoveToVariable(self.guard_chaos_loc_var, "move_client"),
                bt_ros.MoveToVariable(self.nearest_PRElanding, "move_client"),
                bt_ros.ArcMoveToVariable(self.closest_landing, "move_client"),
                bt_ros.BlindStartCollectGround("manipulator_client"),
                bt.ActionNode(self.update_chaos_pucks),
                bt.ActionNode(lambda: self.score_master.add(self.incoming_puck_color.get())),
                bt_ros.MoveToVariable(self.nearest_PRElanding, "move_client"),

                # 2nd
                bt.ActionNode(self.calculate_pucks_configuration),
                bt.ActionNode(self.calculate_closest_landing),
                bt.ActionNode(self.calculate_prelanding),

                bt.ParallelWithMemoryNode([
                    bt_ros.CompleteCollectGround("manipulator_client"),
                    bt.SequenceWithMemoryNode([
                        bt_ros.ArcMoveToVariable(self.nearest_PRElanding, "move_client"),
                        bt_ros.MoveToVariable(self.closest_landing, "move_client"),
                    ])
                ], threshold=2),

                bt_ros.BlindStartCollectGround("manipulator_client"),
                bt.ActionNode(self.update_chaos_pucks),
                bt.ActionNode(lambda: self.score_master.add(self.incoming_puck_color.get())),
                bt_ros.MoveToVariable(self.nearest_PRElanding, "move_client"),

                # 3rd
                bt.ActionNode(self.calculate_pucks_configuration),
                bt.ActionNode(self.calculate_closest_landing),
                bt.ActionNode(self.calculate_prelanding),

                bt.ParallelWithMemoryNode([
                    bt_ros.CompleteCollectGround("manipulator_client"),
                    bt.SequenceWithMemoryNode([
                        bt_ros.ArcMoveToVariable(self.nearest_PRElanding, "move_client"),
                        bt_ros.MoveToVariable(self.closest_landing, "move_client"),
                    ])
                ], threshold=2),

                bt_ros.BlindStartCollectGround("manipulator_client"),
                bt.ActionNode(self.update_chaos_pucks),
                bt.ActionNode(lambda: self.score_master.add(self.incoming_puck_color.get())),
                bt_ros.MoveToVariable(self.nearest_PRElanding, "move_client"),

                # 4th
                bt.ActionNode(self.calculate_pucks_configuration),
                bt.ActionNode(self.calculate_closest_landing),

                bt.ParallelWithMemoryNode([
                    bt_ros.CompleteCollectGround("manipulator_client"),
                    bt_ros.MoveToVariable(self.closest_landing, "move_client"),
                ], threshold=2),

                bt_ros.BlindStartCollectGround("manipulator_client"),
                bt.ActionNode(self.update_chaos_pucks),
                bt.ActionNode(lambda: self.score_master.add(self.incoming_puck_color.get())),

                # back_to_start
                bt.ParallelWithMemoryNode([
                    bt.SequenceWithMemoryNode([
                        bt_ros.CompleteCollectGround("manipulator_client"),
                        bt_ros.StepperUp("manipulator_client"),
                        bt_ros.MainSetManipulatortoGround("manipulator_client")
                    ]),
                    bt_ros.MoveToVariable(self.starting_pos_var, "move_client"),
                ], threshold=2),

                bt_ros.UnloadAccelerator("manipulator_client"),
                bt.ActionNode(lambda: self.score_master.unload("ACC")),
                bt_ros.UnloadAccelerator("manipulator_client"),
                bt.ActionNode(lambda: self.score_master.unload("ACC")),
                bt_ros.UnloadAccelerator("manipulator_client"),
                bt.ActionNode(lambda: self.score_master.unload("ACC")),
                bt_ros.UnloadAccelerator("manipulator_client"),
                bt.ActionNode(lambda: self.score_master.unload("ACC")),
                bt_ros.SetManipulatortoUp("manipulator_client")
            ])

    def update_chaos_pucks(self):
        """
        delete taken puck from known on the field
        get color of last taken puck
        :return: None
        """
        incoming_puck_color = get_color(self.known_chaos_pucks.get()[0])
        self.incoming_puck_color.set(incoming_puck_color)
        rospy.loginfo("incoming_puck_color: " + str(self.incoming_puck_color.get()))
        self.known_chaos_pucks.set(np.delete(self.known_chaos_pucks.get(), 0, axis=0))
        rospy.loginfo("Known pucks after removing: " + str(self.known_chaos_pucks.get()))

    def calculate_pucks_configuration(self):
        """

        :return: # [(0.95, 1.1, 3, 0, 0, 1), ...]
        """
        while not self.update_main_coords():
            print "no coords available"
            rospy.sleep(0.5)

        known_chaos_pucks = sort_wrt_robot(self.main_coords, self.known_chaos_pucks.get())
        print "sorted"
        self.known_chaos_pucks.set(known_chaos_pucks)
        if len(self.known_chaos_pucks.get()) >= 3:
            is_hull_safe_to_approach, coords_sorted_by_angle = sort_by_inner_angle_and_check_if_safe(self.main_coords,
                                                                                                     self.known_chaos_pucks.get(),
                                                                                                     self.critical_angle)

            if not is_hull_safe_to_approach:
                self.known_chaos_pucks.set(coords_sorted_by_angle)  # calc vert-angle, sort by angle, return vertices (sorted)
                rospy.loginfo("hull is not safe to approach, sorted by angle")
            else:  # only sharp angles
                rospy.loginfo("hull is SAFE to approach, keep already sorted wrt robot")
        rospy.loginfo("Known pucks sorted: " + str(self.known_chaos_pucks.get()))

    #     when we finally sorted them, chec if one of them is blue. If so, roll it so blue becomes last one to collect
    #     if self.known_chaos_pucks.get().size > 1 and all(self.known_chaos_pucks.get()[0][3:6] == [0, 0, 1]):
    #         # self.known_chaos_pucks.set(np.roll(self.known_chaos_pucks.get(), -1, axis=0))
    #         rospy.loginfo("blue rolled")

    def calculate_closest_landing(self):
        """

        :return: [(x, y, theta), ...]
        """
        if len(self.known_chaos_pucks.get()) == 1:
            landings = calculate_closest_landing_to_point(self.main_coords,
                                                          self.known_chaos_pucks.get()[:, :2],
                                                          self.approach_vec)
        else:
            landings = unleash_power_of_geometry(self.known_chaos_pucks.get()[:, :2],
                                                 self.scale_factor,
                                                 self.HPAD)

        self.closest_landing.set(landings[0])
        rospy.loginfo("Inside calculate_closest_landing, closest_landing is : ")
        print(self.closest_landing.get())
        print " "

    def calculate_prelanding(self):
        nearest_PRElanding = cvt_local2global(self.drive_back_vec, self.closest_landing.get())
        self.nearest_PRElanding.set(nearest_PRElanding)
        rospy.loginfo("Nearest PRElanding calculated: " + str(self.nearest_PRElanding.get()))
        print " "

    # def update_main_coords(self):
    #     try:
    #         trans_main = self.tfBuffer.lookup_transform('map', "main_robot", rospy.Time(0))  # 0 means last measurment
    #         q_main = [trans_main.transform.rotation.x,
    #                   trans_main.transform.rotation.y,
    #                   trans_main.transform.rotation.z,
    #                   trans_main.transform.rotation.w]
    #         angle_main = euler_from_quaternion(q_main)[2] % (2 * np.pi)
    #         self.main_coords = np.array([trans_main.transform.translation.x,
    #                                      trans_main.transform.translation.y,
    #                                      angle_main])
    #         rospy.loginfo("main coords: " + str(self.main_coords))
    #         return True
    #     except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as msg:
    #         rospy.logwarn(str(msg))
    #         return False
