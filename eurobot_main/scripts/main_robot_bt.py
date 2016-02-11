#!/usr/bin/env python

import rospy
import numpy as np
import behavior_tree as bt
import bt_ros
import tf2_ros
from tf.transformations import euler_from_quaternion
from bt_controller import SideStatus, BTController
from score_controller import ScoreController
from core_functions import *
from shapely.geometry import Point
from shapely.geometry.polygon import Polygon
from std_msgs.msg import String
from visualization_msgs.msg import MarkerArray
from tactics_math import *


class Tactics(object):
    def __init__(self):
        # self.tfBuffer = tf2_ros.Buffer()
        # self.tfListener = tf2_ros.TransformListener(self.tfBuffer)
        # self.robot_coordinates = None

        self.approach_dist = rospy.get_param("approach_dist")  # 0.127 meters, distance from robot to puck where robot will try to grab it
        self.approach_dist = np.array(self.approach_dist)
        self.approach_vec = np.array([-1*self.approach_dist, 0, 0])
        self.ground_spacing_dist = rospy.get_param("ground_spacing_dist")
        self.robot_outer_radius = rospy.get_param("robot_outer_radius")  # FIXME
        self.stick_len = rospy.get_param("stick_len")
        self.delta = rospy.get_param("approach_delta")


class YellowTactics(Tactics):
    def __init__(self):
        super(YellowTactics, self).__init__()
        self.red_cell_puck = rospy.get_param("yellow_side/red_cell_puck")
        self.blunium = rospy.get_param("yellow_side/blunium")
        self.goldenium = rospy.get_param("yellow_side/goldenium")
        self.scales_area = rospy.get_param("yellow_side/scales_area")
        self.scales_area = np.array(self.scales_area)
        self.chaos_center = rospy.get_param("yellow_side/chaos_center")
        self.eighth_puck = rospy.get_param("yellow_side/eighth_puck")

        # self.start_zone = np.array([self.red_cell_puck[0] + self.ground_spacing_dist,
        #                             self.red_cell_puck[1],
        #                             -1.57])

        self.first_puck_landing = np.array([self.red_cell_puck[0] + self.approach_dist - self.delta,
                                           self.red_cell_puck[1],
                                           3.14])

        self.first_puck_landing_finish = np.array([self.red_cell_puck[0],
                                                    self.red_cell_puck[1] - 0.04,
                                                    1.57])

        self.second_puck_landing = np.array([self.red_cell_puck[0],
                                            self.red_cell_puck[1] + self.ground_spacing_dist - self.approach_dist + self.delta,
                                            1.57])

        self.third_puck_landing = np.array([self.red_cell_puck[0],
                                           self.red_cell_puck[1] + 2*self.ground_spacing_dist - self.approach_dist + self.delta,
                                           1.57])

        self.eighth_puck_landing = np.array([self.eighth_puck[0],
                                            self.eighth_puck[1] - 0.185,
                                            1.57])

        # self.eighth_puck_landing_rotate = np.array([self.eighth_puck_landing[0],
        #                                             self.eighth_puck_landing[1],
        #                                             -2.35])

        self.eighth_puck_landing_rotate = np.array([self.eighth_puck_landing[0] - 0.3,
                                                    self.eighth_puck_landing[1] - 0.1,
                                                    -2.35])

        self.chaos_post_pose = np.array([1.72, 0.75, -2.35])

        self.third_puck_rotate_pose = np.array([self.chaos_center[0],
                                                self.chaos_center[1] - 0.3,
                                                -2.35])

        self.blunium_prepose = np.array([self.blunium[0] + 0.07,
                                         self.blunium[1] + 0.35,
                                         -0.52])

        self.blunium_collect_PREpos = np.array([self.blunium[0],
                                                self.blunium[1] + 0.35,
                                                -1.57])

        self.blunium_collect_pos = np.array([self.blunium[0],
                                            self.blunium[1] + 0.185,  # 0.225
                                            self.blunium_collect_PREpos[2]])

        self.blunium_collect_pos_side = np.array([self.blunium[0] + 0.03,
                                                    self.blunium_collect_pos[1],
                                                    self.blunium_collect_PREpos[2]])

        self.blunium_start_push_pose = np.array([self.blunium_prepose[0],
                                                 self.blunium[1] + self.robot_outer_radius,
                                                 self.blunium_prepose[2]])

        self.blunium_end_push_pose = np.array([self.blunium_start_push_pose[0] - 0.08,
                                               self.blunium_start_push_pose[1],
                                               self.blunium_start_push_pose[2]])

        self.blunium_get_back_pose = np.array([self.blunium_end_push_pose[0],
                                               self.blunium_end_push_pose[1] + 0.1,
                                               self.blunium_end_push_pose[2]])

        self.accelerator_PREunloading_pos = np.array([self.blunium_end_push_pose[0] - 0.22,
                                                       self.blunium[1] + 0.13,
                                                       0.56])

        self.goldenium_1_PREgrab_pos = np.array([self.goldenium[0],
                                               self.goldenium[1] + 0.35,
                                               self.accelerator_PREunloading_pos[2]])

        self.goldenium_2_PREgrab_pos = np.array([self.goldenium[0],
                                               self.goldenium[1] + 0.29,
                                               -1.57])

        self.goldenium_grab_pos = np.array([self.goldenium[0],
                                               self.goldenium[1] + 0.185,
                                               self.goldenium_2_PREgrab_pos[2]])


        self.goldenium_back_pose = np.array([self.goldenium[0],
                                            self.goldenium_grab_pos[1] + 0.09,
                                            self.goldenium_2_PREgrab_pos[2]])

        self.goldenium_back_rot_pose = np.array([self.goldenium[0],
                                                 self.goldenium_back_pose[1],
                                                 1])

        self.scales_goldenium_PREpos = np.array([self.chaos_center[0] - 0.25,
                                                    self.chaos_center[1] - 0.15,
                                                    1.4])

        self.scales_goldenium_pos = np.array([self.scales_goldenium_PREpos[0] - 0.05,
                                              self.scales_goldenium_PREpos[1] + 0.52,
                                              1.76])  # 1.83


class PurpleTactics(Tactics):
    def __init__(self):
        super(PurpleTactics, self).__init__()

        self.red_cell_puck = rospy.get_param("purple_side/red_cell_puck")
        self.scales_area = rospy.get_param("purple_side/scales_area")
        self.scales_area = np.array(self.scales_area)
        self.chaos_center = rospy.get_param("purple_side/chaos_center")
        self.goldenium = rospy.get_param("purple_side/goldenium")
        self.blunium = rospy.get_param("purple_side/blunium")
        self.eighth_puck = rospy.get_param("purple_side/eighth_puck")

        # self.start_zone = np.array([self.red_cell_puck[0] - self.ground_spacing_dist,
        #                             self.red_cell_puck[1],
        #                             -1.57])

        self.first_puck_landing = np.array([self.red_cell_puck[0] - self.approach_dist + self.delta,
                                           self.red_cell_puck[1],
                                           0])

        self.first_puck_landing_finish = np.array([self.red_cell_puck[0],
                                                    self.red_cell_puck[1] - 0.04,
                                                    1.57])

        self.second_puck_landing = np.array([self.red_cell_puck[0],
                                            self.red_cell_puck[1] + self.ground_spacing_dist - self.approach_dist + self.delta,
                                            1.57])

        self.third_puck_landing = np.array([self.red_cell_puck[0],
                                           self.red_cell_puck[1] + 2*self.ground_spacing_dist - self.approach_dist + self.delta,
                                           1.57])

        self.eighth_puck_landing = np.array([self.eighth_puck[0],
                                            self.eighth_puck[1] - 0.185,
                                            1.57])

        # self.eighth_puck_landing_rotate = np.array([self.eighth_puck_landing[0],
        #                                             self.eighth_puck_landing[1],
        #                                             -0.78])

        self.eighth_puck_landing_rotate = np.array([self.eighth_puck_landing[0] + 0.3,
                                                    self.eighth_puck_landing[1] - 0.1,
                                                    -0.78])

        self.chaos_post_pose = np.array([1.3, 0.77, -0.78])  # 1.22, 0.75  CHECK

        self.third_puck_rotate_pose = np.array([self.chaos_center[0],
                                                self.chaos_center[1] - 0.3,
                                                -0.78])

        self.blunium_prepose = np.array([self.blunium[0] - 0.07,
                                         self.blunium[1] + 0.35,
                                         -0.52])

        self.blunium_collect_PREpos = np.array([self.blunium[0],
                                                self.blunium[1] + 0.35,
                                                -1.57])

        self.blunium_collect_pos = np.array([self.blunium[0],
                                            self.blunium[1] + 0.185,
                                            self.blunium_collect_PREpos[2]])

        self.blunium_collect_pos_side = np.array([self.blunium[0] - 0.03,
                                                    self.blunium_collect_pos[1],
                                                    self.blunium_collect_PREpos[2]])

        self.blunium_start_push_pose = np.array([self.blunium_prepose[0],
                                                 self.blunium[1] + self.robot_outer_radius,
                                                 self.blunium_prepose[2]])

        self.blunium_end_push_pose = np.array([self.blunium_start_push_pose[0] + 0.08,
                                               self.blunium_start_push_pose[1],
                                               self.blunium_start_push_pose[2]])

        self.blunium_get_back_pose = np.array([self.blunium_end_push_pose[0],
                                               self.blunium_end_push_pose[1] + 0.1,
                                               self.blunium_end_push_pose[2]])

        self.accelerator_PREunloading_pos = np.array([self.blunium_end_push_pose[0] + 0.22,
                                                       self.blunium[1] + 0.13,  # center of robot
                                                       0.56])

        self.goldenium_1_PREgrab_pos = np.array([self.goldenium[0],
                                               self.goldenium[1] + 0.35,
                                               self.accelerator_PREunloading_pos[2]])

        self.goldenium_2_PREgrab_pos = np.array([self.goldenium[0],
                                               self.goldenium[1] + 0.28,
                                               -1.57])

        self.goldenium_grab_pos = np.array([self.goldenium[0],
                                               self.goldenium[1] + 0.185,
                                               self.goldenium_2_PREgrab_pos[2]])

        self.goldenium_back_pose = np.array([self.goldenium[0],
                                            self.goldenium_grab_pos[1] + 0.09,
                                            self.goldenium_2_PREgrab_pos[2]])

        self.goldenium_back_rot_pose = np.array([self.goldenium[0],
                                                 self.goldenium_back_pose[1],
                                                 0.78])  # 2

        self.scales_goldenium_PREpos = np.array([self.chaos_center[0] + 0.25,
                                                    self.chaos_center[1] - 0.15,
                                                    0.78])  # 1.7

        self.scales_goldenium_pos = np.array([self.scales_goldenium_PREpos[0] + 0.05,
                                              self.scales_goldenium_PREpos[1] + 0.52,
                                              1.38])  # 1.31


class MainRobotBT(object):
    # noinspection PyTypeChecker
    def __init__(self):  # fixme  YELLOW  PURPLE  side_status=SideStatus.PURPLE
        self.robot_name = rospy.get_param("robot_name")
        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer)
        self.secondary_coords = np.array([0, 0, 0])
        self.main_coords = None

        self.move_publisher = rospy.Publisher("navigation/command", String, queue_size=100)
        self.manipulator_publisher = rospy.Publisher("manipulator/command", String, queue_size=100)
        self.stm_publisher = rospy.Publisher("stm/command", String, queue_size=100)

        self.move_client = bt_ros.ActionClient(self.move_publisher)
        self.manipulator_client = bt_ros.ActionClient(self.manipulator_publisher)
        self.stm_client = bt_ros.ActionClient(self.stm_publisher)

        self.purple_tactics = PurpleTactics()
        self.yellow_tactics = YellowTactics()
        self.tactics = None
        self.side_status = None

        self.bt = None
        self.bt_timer = None

        self.known_chaos_pucks = bt.BTVariable(np.array([]))  # (x, y, id, r, g, b)  # FIXME

        self.is_secondary_working = False

        self.scale_factor = rospy.get_param("scale_factor")  # used in calculating outer bissectrisa for hull's angles
        self.scale_factor = np.array(self.scale_factor)

        self.critical_angle = np.pi * 2/3
        # self.critical_angle = rospy.get_param("critical_angle")  # fixme

        self.red_zone_coords = np.array([0.3, 0.6, -np.pi/3])
        self.approach_dist = rospy.get_param("approach_dist")  # meters, distance from robot to puck where robot will try to grab it
        self.approach_dist = np.array(self.approach_dist)

        self.drive_back_dist = rospy.get_param("drive_back_dist")
        self.drive_back_dist = np.array(self.drive_back_dist)

        self.approach_vec = np.array([-1*self.approach_dist, 0, 0])
        self.drive_back_vec = np.array([-1*self.drive_back_dist, 0, 0])

        # TODO: pucks in front of starting cells are random, so while we aren't using camera
        #       will call them REDIUM  (it doesn't matter, because in this strategy we move them all to acc)

        # TODO: It will matter in case big robot faces hard collision and need to unload pucks in starting cells

        self.incoming_puck_color = bt.BTVariable(None)
        self.collected_pucks = bt.BTVariable(np.array([]))
        self.score_master = ScoreController(self.collected_pucks, self.robot_name)

        # self.pucks_subscriber = rospy.Subscriber("/pucks", MarkerArray, self.pucks_callback, queue_size=1)
        rospy.Subscriber("navigation/response", String, self.move_client.response_callback)
        rospy.Subscriber("manipulator/response", String, self.manipulator_client.response_callback)

    def is_robot_empty(self):
        rospy.loginfo("pucks inside")
        rospy.loginfo(self.collected_pucks.get())
        if len(self.collected_pucks.get()) == 0:
            rospy.loginfo('All pucks unloaded')
            return bt.Status.SUCCESS
        else:
            rospy.loginfo('Pucks inside: ' + str(len(self.collected_pucks.get())))
            return bt.Status.FAILED

    def is_robot_empty_1(self):
        rospy.loginfo("pucks inside")
        rospy.loginfo(self.collected_pucks.get())
        if len(self.collected_pucks.get()) == 0:
            rospy.loginfo('All pucks unloaded')
            return bt.Status.SUCCESS
        else:
            rospy.loginfo('Pucks inside: ' + str(len(self.collected_pucks.get())))
            return bt.Status.RUNNING

    def is_scales_landing_free(self):
        """
        Secondary may be: 
        - not working at all -- than 
        - somewhere else and we know of it
        - working but we don't get info about it -- wait

        if we don't get secondary coords - wait 10 sec
        if we get them, wait until it gets out of zone

        """
        self.is_secondary_working = self.update_secondary_coords()
        rospy.loginfo("Checking if scales are available to approach...")

        area = self.tactics.scales_area
        robot = self.secondary_coords  # can be 0, 0, 0  or  some value

        point = Point(robot[0], robot[1])
        polygon = Polygon([area[0], area[1], area[2], area[3]])

        if not self.is_secondary_working:
            rospy.sleep(5)
            return bt.Status.SUCCESS
        else:
            print("got coords in condition:")
            print(self.secondary_coords)
            if polygon.contains(point):
                rospy.loginfo('Landing busy')
                return bt.Status.RUNNING
            else:
                rospy.loginfo('Landing is free to go')
                return bt.Status.SUCCESS

    # def strategy_msc(self):
    #     red_cell_puck = bt.SequenceWithMemoryNode([
    #                         bt_ros.MoveLineToPoint(self.tactics.first_puck_landing, "move_client"),
    #                         bt.FallbackWithMemoryNode([
    #                             bt.SequenceWithMemoryNode([
    #                                 bt_ros.BlindStartCollectGround("manipulator_client"),
    #                                 bt.ActionNode(lambda: self.score_master.add("REDIUM")),  # FIXME: color is undetermined without camera!
    #                                 bt.ParallelWithMemoryNode([
    #                                     bt_ros.CompleteCollectGround("manipulator_client"),
    #                                     bt_ros.MoveLineToPoint(self.tactics.first_puck_landing_finish, "move_client"),
    #                                 ], threshold=2),
    #                             ]),
    #                             bt.ParallelWithMemoryNode([
    #                                 bt_ros.SetManipulatortoWall("manipulator_client"),
    #                                 bt_ros.MoveLineToPoint(self.tactics.first_puck_landing_finish, "move_client"),
    #                             ], threshold=2),
    #                         ]),
    #                     ])

    #     green_cell_puck = bt.SequenceWithMemoryNode([
    #                         bt_ros.MoveLineToPoint(self.tactics.second_puck_landing, "move_client"),
    #                         bt.FallbackWithMemoryNode([
    #                             bt.SequenceWithMemoryNode([
    #                                 bt_ros.BlindStartCollectGround("manipulator_client"),
    #                                 bt.ActionNode(lambda: self.score_master.add("REDIUM")),  # FIXME: color is undetermined without camera!
    #                                 bt.ParallelWithMemoryNode([
    #                                     bt_ros.CompleteCollectGround("manipulator_client"),
    #                                     bt_ros.MoveLineToPoint(self.tactics.third_puck_landing, "move_client"),
    #                                 ], threshold=2),
    #                             ]),
    #                             bt.ParallelWithMemoryNode([
    #                                 bt_ros.SetManipulatortoWall("manipulator_client"),
    #                                 bt_ros.MoveLineToPoint(self.tactics.third_puck_landing, "move_client"),
    #                             ], threshold=2),
    #                         ])
    #                     ])

    #     blue_cell_puck = bt.SequenceWithMemoryNode([
    #                         bt.FallbackWithMemoryNode([
    #                             bt.SequenceWithMemoryNode([
    #                                 bt_ros.BlindStartCollectGround("manipulator_client"),
    #                                 bt.ActionNode(lambda: self.score_master.add("REDIUM")),  # FIXME: color is undetermined without camera!
    #                                 bt.ParallelWithMemoryNode([
    #                                     bt_ros.CompleteCollectGround("manipulator_client"),
    #                                     bt_ros.MoveLineToPoint(self.tactics.third_puck_rotate_pose, "move_client"),
    #                                 ], threshold=2),
    #                             ]),
    #                             bt.ParallelWithMemoryNode([
    #                                 bt_ros.SetManipulatortoUp("manipulator_client"),  # FIXME when adding chaos
    #                                 bt_ros.MoveLineToPoint(self.tactics.third_puck_rotate_pose, "move_client"),
    #                             ], threshold=2),
    #                         ])
    #                     ])

    #     finish_move_blunium_and_push = bt.SequenceWithMemoryNode([
    #                                         bt_ros.MoveLineToPoint(self.tactics.blunium_start_push_pose, "move_client"),
    #                                         bt.ParallelWithMemoryNode([
    #                                             bt_ros.MainSetManipulatortoGround("manipulator_client"),  # FIXME when adding chaos
    #                                             bt_ros.MoveLineToPoint(self.tactics.blunium_start_push_pose, "move_client"),
    #                                         ], threshold=2),
    #                                         # bt_ros.MoveLineToPoint(self.tactics.blunium_prepose, "move_client"),
    #                                         bt_ros.MoveLineToPoint(self.tactics.blunium_end_push_pose, "move_client"),
    #                                         bt.ActionNode(lambda: self.score_master.add("BLUNIUM")),
    #                                         bt.ActionNode(lambda: self.score_master.unload("ACC")),
    #                                         bt.ActionNode(lambda: self.score_master.reward("UNLOCK_GOLDENIUM_BONUS")),
    #                                     ])

    #     approach_acc = bt.SequenceWithMemoryNode([
    #                         bt_ros.MoveLineToPoint(self.tactics.blunium_get_back_pose, "move_client"),
    #                         bt_ros.MoveLineToPoint(self.tactics.accelerator_PREunloading_pos, "move_client"),  # FIXME try Arc
    #                         bt_ros.SetSpeedSTM([0, -0.1, 0], 0.6, "stm_client"),
    #                     ])

    #     push_unload_first_in_acc = bt.SequenceWithMemoryNode([
    #                                 bt_ros.StepperUp("manipulator_client"),  # FIXME do we need to do that? NO if all 7 pucks inside
    #                                 bt_ros.UnloadAccelerator("manipulator_client"),
    #                                 bt.ActionNode(lambda: self.score_master.unload("ACC")),
    #                             ])
        
    #     unload_acc = bt.SequenceNode([
    #                     bt.FallbackNode([
    #                         bt.ConditionNode(self.is_robot_empty),
    #                         bt.SequenceWithMemoryNode([
    #                             bt_ros.UnloadAccelerator("manipulator_client"),
    #                             bt.ActionNode(lambda: self.score_master.unload("ACC")),
    #                         ])
    #                     ]),
    #                     bt.ConditionNode(self.is_robot_empty_1)
    #                 ])
        
    #     collect_goldenium = bt.SequenceWithMemoryNode([
    #                             bt_ros.MoveLineToPoint(self.tactics.goldenium_1_PREgrab_pos, "move_client"),
    #                             bt_ros.MoveLineToPoint(self.tactics.goldenium_2_PREgrab_pos, "move_client"),
    #                             bt_ros.StartCollectGoldenium("manipulator_client"),
    #                             bt_ros.MoveLineToPoint(self.tactics.goldenium_grab_pos, "move_client"),
    #                             bt_ros.GrabGoldeniumAndHoldUp("manipulator_client"),
    #                             bt.ActionNode(lambda: self.score_master.add("GOLDENIUM")),
    #                             bt.ActionNode(lambda: self.score_master.reward("GRAB_GOLDENIUM_BONUS")),
    #                         ])

    #     # collect_goldenium = bt.SequenceWithMemoryNode([
    #     #                         bt_ros.MoveLineToPoint(self.tactics.goldenium_1_PREgrab_pos, "move_client"),
    #     #                         bt_ros.MoveLineToPoint(self.tactics.goldenium_2_PREgrab_pos, "move_client"),
    #     #                         bt_ros.StartCollectGoldenium("manipulator_client"),

    #     #                         bt.ParallelWithMemoryNode([
    #     #                             bt_ros.MoveLineToPoint(self.tactics.goldenium_grab_pos, "move_client"),
    #     #                             bt_ros.CheckLimitSwitchInf("manipulator_client")
    #     #                         ], threshold=1),

    #     #                         bt_ros.GrabGoldeniumAndHoldUp("manipulator_client"),
    #     #                         bt.ActionNode(lambda: self.score_master.add("GOLDENIUM")),
    #     #                         bt.ActionNode(lambda: self.score_master.reward("GRAB_GOLDENIUM_BONUS")),
    #     #                     ])

    #     move_to_goldenium_prepose = bt.SequenceWithMemoryNode([
    #                                     bt_ros.MoveLineToPoint(self.tactics.goldenium_back_rot_pose, "move_client"),
    #                                     bt_ros.MoveLineToPoint(self.tactics.scales_goldenium_PREpos, "move_client")
    #                                 ])
        
    #     unload_goldenium = bt.SequenceWithMemoryNode([
    #                             bt.ConditionNode(self.is_scales_landing_free),
    #                             bt.SequenceWithMemoryNode([
    #                                 bt_ros.MoveLineToPoint(self.tactics.scales_goldenium_pos + np.array([0, -0.05, 0]), "move_client"),
    #                                 bt.ActionNode(lambda: self.score_master.unload("SCALES")),
    #                                 bt_ros.SetManipulatortoWall("manipulator_client"),
    #                                 bt_ros.MoveLineToPoint(self.tactics.scales_goldenium_pos, "move_client"),
    #                                 bt_ros.UnloadGoldenium("manipulator_client"),
    #                             ])
    #                         ])

    #     strategy = bt.SequenceWithMemoryNode([
    #                     red_cell_puck,
    #                     green_cell_puck,
    #                     blue_cell_puck,
    #                     finish_move_blunium_and_push,
    #                     approach_acc,
    #                     push_unload_first_in_acc,
    #                     unload_acc,
    #                     collect_goldenium,
    #                     move_to_goldenium_prepose,
    #                     unload_goldenium,
    #                     ])

    #     return strategy

    def strategy_with_chaos(self):
        red_cell_puck = bt.SequenceWithMemoryNode([
                            bt_ros.MoveLineToPoint(self.tactics.first_puck_landing, "move_client"),
                            bt.FallbackWithMemoryNode([
                                bt.SequenceWithMemoryNode([
                                    bt_ros.BlindStartCollectGround("manipulator_client"),
                                    bt.ActionNode(lambda: self.score_master.add("REDIUM")),  # FIXME: color is undetermined without camera!
                                    bt.ParallelWithMemoryNode([
                                        bt_ros.CompleteCollectGround("manipulator_client"),
                                        bt_ros.MoveLineToPoint(self.tactics.first_puck_landing_finish, "move_client"),
                                    ], threshold=2),
                                ]),
                                bt.ParallelWithMemoryNode([
                                    bt_ros.SetManipulatortoWall("manipulator_client"),
                                    bt_ros.MoveLineToPoint(self.tactics.first_puck_landing_finish, "move_client"),
                                ], threshold=2)
                            ]),
                        ])

        green_cell_puck = bt.SequenceWithMemoryNode([
                            bt_ros.MoveLineToPoint(self.tactics.second_puck_landing, "move_client"),
                            bt.FallbackWithMemoryNode([
                                bt.SequenceWithMemoryNode([
                                    bt_ros.BlindStartCollectGround("manipulator_client"),
                                    bt.ActionNode(lambda: self.score_master.add("REDIUM")),  # FIXME: color is undetermined without camera!
                                    bt.ParallelWithMemoryNode([
                                        bt_ros.CompleteCollectGround("manipulator_client"),
                                        bt_ros.MoveLineToPoint(self.tactics.third_puck_landing, "move_client"),
                                    ], threshold=2),
                                ]),
                                bt.ParallelWithMemoryNode([
                                    bt_ros.SetManipulatortoWall("manipulator_client"),
                                    bt_ros.MoveLineToPoint(self.tactics.third_puck_landing, "move_client"),
                                ], threshold=2)
                            ])
                        ])

        blue_cell_puck = bt.SequenceWithMemoryNode([
                            bt.FallbackWithMemoryNode([
                                bt.SequenceWithMemoryNode([
                                    bt_ros.BlindStartCollectGround("manipulator_client"),
                                    bt.ActionNode(lambda: self.score_master.add("GREENIUM")),  # FIXME: color is undetermined without camera!

                                    bt.ParallelWithMemoryNode([
                                        bt_ros.CompleteCollectGround("manipulator_client"),
                                        bt_ros.MoveLineToPoint(self.tactics.eighth_puck_landing + np.array([0, -0.07, 0]), "move_client"),
                                    ], threshold=2),

                                    bt.ParallelWithMemoryNode([
                                        bt_ros.StartTakeWallPuck("manipulator_client"),
                                        bt_ros.MoveLineToPoint(self.tactics.eighth_puck_landing, "move_client"),
                                        bt_ros.CheckLimitSwitchInfLong("manipulator_client")
                                    ], threshold=2),
                                ]),
                                bt.ParallelWithMemoryNode([
                                    bt_ros.SetManipulatortoUp("manipulator_client"),  # FIXME when adding chaos
                                    bt_ros.MoveLineToPoint(self.tactics.third_puck_rotate_pose, "move_client"),
                                ], threshold=2)
                            ])
                        ])

        # move_through_chaos = bt.SequenceWithMemoryNode([
        #                         bt_ros.CompleteCollectGround("manipulator_client"),  # this is not blunium, but red in 6_wall
        #                         bt.ActionNode(lambda: self.score_master.add("REDIUM")),
        #                         bt_ros.MoveLineToPoint(self.tactics.eighth_puck_landing_rotate, "move_client"),

        #                         bt_ros.MoveLineToPoint(self.tactics.chaos_post_pose, "move_client"),
        #                         bt.ParallelWithMemoryNode([
        #                             bt_ros.MoveLineToPoint(self.tactics.blunium_collect_PREpos, "move_client"),
        #                             bt.FallbackWithMemoryNode([
        #                                 bt.SequenceWithMemoryNode([
        #                                     bt_ros.StartCollectGround("manipulator_client"),
        #                                     bt_ros.CompleteCollectGround("manipulator_client"),
        #                                     bt.ActionNode(lambda: self.score_master.add("REDIUM")),  # FIXME: color is undetermined without camera!
        #                                     bt_ros.StartCollectBlunium("manipulator_client"),
        #                                 ]),
        #                                 bt_ros.StopPump("manipulator_client"),
        #                                 bt_ros.StartCollectBlunium("manipulator_client")
        #                             ])
        #                             # bt_ros.BlindStartCollectGround("manipulator_client"),
        #                         ], threshold=2),
        #                     ])

        # works
        move_through_chaos = bt.SequenceWithMemoryNode([
                                bt_ros.Delay500("manipulator_client"),
                                bt_ros.CompleteCollectGround("manipulator_client"),  # this is not blunium, but red in 6_wall
                                bt.ActionNode(lambda: self.score_master.add("REDIUM")),
                                bt_ros.MoveLineToPoint(self.tactics.eighth_puck_landing_rotate, "move_client"),

                                bt_ros.MoveLineToPoint(self.tactics.chaos_post_pose, "move_client"),
                                bt.FallbackWithMemoryNode([
                                    bt.SequenceWithMemoryNode([
                                        bt_ros.StartCollectGround("manipulator_client"),
                                        bt_ros.CompleteCollectGround("manipulator_client"),
                                        bt.ActionNode(lambda: self.score_master.add("REDIUM")),  # FIXME: color is undetermined without camera!
                                    ]),
                                    bt_ros.StopPump("manipulator_client"),
                                ]),
                                bt_ros.MoveLineToPoint(self.tactics.blunium_collect_PREpos, "move_client"),
                                bt_ros.StartCollectBlunium("manipulator_client")
                                # bt_ros.BlindStartCollectGround("manipulator_client"),
                            ])

        move_and_collect_blunium = bt.SequenceWithMemoryNode([
                                        # bt_ros.StartCollectBlunium("manipulator_client"),
                                        bt.ParallelWithMemoryNode([
                                            bt_ros.MoveLineToPoint(self.tactics.blunium_collect_pos, "move_client"),
                                            bt_ros.CheckLimitSwitchInfLong("manipulator_client")
                                        ], threshold=1),
                                        # bt_ros.MoveLineToPoint(self.tactics.blunium_collect_pos_side, "move_client"),
                                        bt_ros.MoveLineToPoint(self.tactics.blunium_collect_pos + np.array([0, 0.04, 0]), "move_client"),
                                        bt_ros.FinishCollectBlunium("manipulator_client"),
                                        bt.ActionNode(lambda: self.score_master.add("BLUNIUM")),
                                        bt_ros.MainSetManipulatortoGround("manipulator_client")
                                    ])

        approach_acc = bt.SequenceWithMemoryNode([
                            bt_ros.MoveLineToPoint(self.tactics.blunium_get_back_pose, "move_client"),
                            bt_ros.MoveLineToPoint(self.tactics.accelerator_PREunloading_pos, "move_client"),  # FIXME try Arc
                            bt_ros.SetSpeedSTM([0, -0.1, 0], 0.9, "stm_client")
                        ])

        collect_unload_first_in_acc = bt.SequenceWithMemoryNode([
                                    # bt_ros.StepperUp("manipulator_client"),  # FIXME do we need to do that? NO if all 7 pucks inside
                                    bt_ros.UnloadAccelerator("manipulator_client"),
                                    bt.ActionNode(lambda: self.score_master.unload("ACC")),
                                    bt.ActionNode(lambda: self.score_master.reward("UNLOCK_GOLDENIUM_BONUS"))
                                ])

        unload_acc = bt.SequenceNode([
                        bt.FallbackNode([
                            bt.ConditionNode(self.is_robot_empty),
                            bt.SequenceWithMemoryNode([
                                bt_ros.UnloadAccelerator("manipulator_client"),
                                bt.ActionNode(lambda: self.score_master.unload("ACC")),
                            ])
                        ]),
                        bt.ConditionNode(self.is_robot_empty_1)
                    ])

        collect_goldenium = bt.SequenceWithMemoryNode([
                                bt_ros.Delay500("manipulator_client"),
                                # bt_ros.MoveLineToPoint(self.tactics.goldenium_1_PREgrab_pos, "move_client"),  CHECK
                                bt_ros.MoveLineToPoint(self.tactics.goldenium_2_PREgrab_pos, "move_client"),
                                bt_ros.StartCollectGoldenium("manipulator_client"),

                                bt.ParallelWithMemoryNode([
                                    bt_ros.MoveLineToPoint(self.tactics.goldenium_grab_pos, "move_client"),
                                    bt_ros.CheckLimitSwitchInfLong("manipulator_client")
                                ], threshold=1)
                            ])

        move_to_goldenium_prepose = bt.SequenceWithMemoryNode([
                                        bt_ros.MoveLineToPoint(self.tactics.goldenium_back_pose, "move_client"),
                                        bt.ActionNode(lambda: self.score_master.add("GOLDENIUM")),
                                        bt.ActionNode(lambda: self.score_master.reward("GRAB_GOLDENIUM_BONUS")),
                                        bt.ParallelWithMemoryNode([
                                            bt_ros.GrabGoldeniumAndHoldUp("manipulator_client"),
                                            bt_ros.MoveLineToPoint(self.tactics.scales_goldenium_PREpos, "move_client")
                                        ], threshold=2)
                                    ])

        unload_goldenium = bt.SequenceWithMemoryNode([
                                bt.ConditionNode(self.is_scales_landing_free),
                                bt.SequenceWithMemoryNode([
                                    # bt_ros.MoveLineToPoint(self.tactics.scales_goldenium_pos + np.array([0, -0.01, 0]), "move_client"),
                                    bt_ros.MoveLineToPoint(self.tactics.scales_goldenium_pos, "move_client"),
                                    bt.ActionNode(lambda: self.score_master.unload("SCALES")),
                                    bt_ros.SetManipulatortoWall("manipulator_client"),
                                    bt_ros.GrabGoldeniumAndHoldUp("manipulator_client"),
                                    # bt_ros.SetManipulatortoWall("manipulator_client"),
                                    # bt_ros.GrabGoldeniumAndHoldUp("manipulator_client"),
                                    # bt_ros.MoveLineToPoint(self.tactics.scales_goldenium_pos, "move_client"),
                                    bt_ros.UnloadGoldenium("manipulator_client"),
                                    bt_ros.SetManipulatortoUp("manipulator_client")
                                ])
                            ])

        strategy = bt.SequenceWithMemoryNode([
                        red_cell_puck,
                        green_cell_puck,
                        blue_cell_puck,
                        move_through_chaos,
                        move_and_collect_blunium,
                        approach_acc,
                        collect_unload_first_in_acc,
                        unload_acc,
                        collect_goldenium,
                        move_to_goldenium_prepose,
                        unload_goldenium
                        ])

        return strategy

    def strategy_rus(self):
        red_cell_puck = bt.SequenceWithMemoryNode([
                            bt_ros.MoveLineToPoint(self.tactics.first_puck_landing, "move_client"),
                            bt.FallbackWithMemoryNode([
                                bt.SequenceWithMemoryNode([
                                    bt_ros.BlindStartCollectGround("manipulator_client"),
                                    bt.ActionNode(lambda: self.score_master.add("REDIUM")),  # FIXME: color is undetermined without camera!
                                    bt.ParallelWithMemoryNode([
                                        bt_ros.CompleteCollectGround("manipulator_client"),
                                        bt_ros.MoveLineToPoint(self.tactics.first_puck_landing_finish, "move_client"),
                                    ], threshold=2),
                                ]),
                                bt.ParallelWithMemoryNode([
                                    bt_ros.SetManipulatortoWall("manipulator_client"),
                                    bt_ros.MoveLineToPoint(self.tactics.first_puck_landing_finish, "move_client"),
                                ], threshold=2)
                            ]),
                        ])

        green_cell_puck = bt.SequenceWithMemoryNode([
                            bt_ros.MoveLineToPoint(self.tactics.second_puck_landing, "move_client"),
                            bt.FallbackWithMemoryNode([
                                bt.SequenceWithMemoryNode([
                                    bt_ros.BlindStartCollectGround("manipulator_client"),
                                    bt.ActionNode(lambda: self.score_master.add("REDIUM")),  # FIXME: color is undetermined without camera!
                                    bt.ParallelWithMemoryNode([
                                        bt_ros.CompleteCollectGround("manipulator_client"),
                                        bt_ros.MoveLineToPoint(self.tactics.third_puck_landing, "move_client"),
                                    ], threshold=2),
                                ]),
                                bt.ParallelWithMemoryNode([
                                    bt_ros.SetManipulatortoWall("manipulator_client"),
                                    bt_ros.MoveLineToPoint(self.tactics.third_puck_landing, "move_client"),
                                ], threshold=2)
                            ])
                        ])

        blue_cell_puck = bt.SequenceWithMemoryNode([
                            bt.FallbackWithMemoryNode([
                                bt.SequenceWithMemoryNode([
                                    bt_ros.BlindStartCollectGround("manipulator_client"),
                                    bt.ActionNode(lambda: self.score_master.add("GREENIUM")),  # FIXME: color is undetermined without camera!

                                    bt.ParallelWithMemoryNode([
                                        bt_ros.CompleteCollectGround("manipulator_client"),
                                        bt_ros.MoveLineToPoint(self.tactics.eighth_puck_landing + np.array([0, -0.06, 0]), "move_client"),
                                    ], threshold=2),

                                    bt.ParallelWithMemoryNode([
                                        bt_ros.StartTakeWallPuck("manipulator_client"),
                                        bt_ros.MoveLineToPoint(self.tactics.eighth_puck_landing, "move_client"),
                                        bt_ros.CheckLimitSwitchInfLong("manipulator_client")
                                    ], threshold=2),
                                ]),
                                bt.ParallelWithMemoryNode([
                                    bt_ros.SetManipulatortoUp("manipulator_client"),  # FIXME when adding chaos
                                    bt_ros.MoveLineToPoint(self.tactics.third_puck_rotate_pose, "move_client"),
                                ], threshold=2)
                            ])
                        ])

        move_and_collect_blunium = bt.SequenceWithMemoryNode([
                                        bt.ParallelWithMemoryNode([
                                            bt_ros.CompleteCollectGround("manipulator_client"),  # this is not blunium, but red in 6_wall
                                            bt.ActionNode(lambda: self.score_master.add("REDIUM")),
                                            bt_ros.MoveLineToPoint(self.tactics.blunium_collect_PREpos, "move_client"),
                                        ], threshold=3),

                                        bt_ros.StartCollectBlunium("manipulator_client"),
                                        bt.ParallelWithMemoryNode([
                                            bt_ros.MoveLineToPoint(self.tactics.blunium_collect_pos, "move_client"),
                                            bt_ros.CheckLimitSwitchInfLong("manipulator_client")
                                        ], threshold=1),
                                        # bt_ros.MoveLineToPoint(self.tactics.blunium_collect_pos_side, "move_client"),
                                        bt_ros.MoveLineToPoint(self.tactics.blunium_collect_pos + np.array([0, 0.04, 0]), "move_client"),
                                        bt_ros.FinishCollectBlunium("manipulator_client"),
                                        bt.ActionNode(lambda: self.score_master.add("BLUNIUM")),
                                        bt_ros.MainSetManipulatortoGround("manipulator_client")
                                    ])

        approach_acc = bt.SequenceWithMemoryNode([
                            bt_ros.MoveLineToPoint(self.tactics.blunium_get_back_pose, "move_client"),
                            bt_ros.MoveLineToPoint(self.tactics.accelerator_PREunloading_pos, "move_client"),  # FIXME try Arc
                            bt_ros.SetSpeedSTM([0, -0.1, 0], 0.9, "stm_client")
                        ])

        collect_unload_first_in_acc = bt.SequenceWithMemoryNode([
                                    # bt_ros.StepperUp("manipulator_client"),  # FIXME do we need to do that? NO if all 7 pucks inside
                                    bt_ros.UnloadAccelerator("manipulator_client"),
                                    bt.ActionNode(lambda: self.score_master.unload("ACC")),
                                    bt.ActionNode(lambda: self.score_master.reward("UNLOCK_GOLDENIUM_BONUS"))
                                ])

        unload_acc = bt.SequenceNode([
                        bt.FallbackNode([
                            bt.ConditionNode(self.is_robot_empty),
                            bt.SequenceWithMemoryNode([
                                bt_ros.UnloadAccelerator("manipulator_client"),
                                bt.ActionNode(lambda: self.score_master.unload("ACC")),
                            ])
                        ]),
                        bt.ConditionNode(self.is_robot_empty_1)
                    ])

        collect_goldenium = bt.SequenceWithMemoryNode([
                                bt_ros.Delay500("manipulator_client"),
                                # bt_ros.MoveLineToPoint(self.tactics.goldenium_1_PREgrab_pos, "move_client"),  CHECK
                                bt_ros.MoveLineToPoint(self.tactics.goldenium_2_PREgrab_pos, "move_client"),
                                bt_ros.StartCollectGoldenium("manipulator_client"),

                                bt.ParallelWithMemoryNode([
                                    bt_ros.MoveLineToPoint(self.tactics.goldenium_grab_pos, "move_client"),
                                    bt_ros.CheckLimitSwitchInfLong("manipulator_client")
                                ], threshold=1)
                            ])

        move_to_goldenium_prepose = bt.SequenceWithMemoryNode([
                                        bt_ros.MoveLineToPoint(self.tactics.goldenium_back_pose, "move_client"),
                                        bt.ActionNode(lambda: self.score_master.add("GOLDENIUM")),
                                        bt.ActionNode(lambda: self.score_master.reward("GRAB_GOLDENIUM_BONUS")),
                                        bt.ParallelWithMemoryNode([
                                            bt_ros.GrabGoldeniumAndHoldUp("manipulator_client"),
                                            bt_ros.MoveLineToPoint(self.tactics.scales_goldenium_PREpos, "move_client")
                                        ], threshold=2)
                                    ])

        unload_goldenium = bt.SequenceWithMemoryNode([
                                bt.ConditionNode(self.is_scales_landing_free),
                                bt.SequenceWithMemoryNode([
                                    # bt_ros.MoveLineToPoint(self.tactics.scales_goldenium_pos + np.array([0, -0.01, 0]), "move_client"),
                                    bt_ros.MoveLineToPoint(self.tactics.scales_goldenium_pos, "move_client"),
                                    bt.ActionNode(lambda: self.score_master.unload("SCALES")),
                                    bt_ros.SetManipulatortoWall("manipulator_client"),
                                    bt_ros.GrabGoldeniumAndHoldUp("manipulator_client"),
                                    # bt_ros.SetManipulatortoWall("manipulator_client"),
                                    # bt_ros.GrabGoldeniumAndHoldUp("manipulator_client"),
                                    # bt_ros.MoveLineToPoint(self.tactics.scales_goldenium_pos, "move_client"),
                                    bt_ros.UnloadGoldenium("manipulator_client"),
                                    bt_ros.SetManipulatortoUp("manipulator_client")
                                ])
                            ])

        strategy = bt.SequenceWithMemoryNode([
                        red_cell_puck,
                        green_cell_puck,
                        blue_cell_puck,
                        move_and_collect_blunium,
                        approach_acc,
                        collect_unload_first_in_acc,
                        unload_acc,
                        collect_goldenium,
                        move_to_goldenium_prepose,
                        unload_goldenium,
                        ])

        return strategy

    def start(self):

        self.bt = bt.Root(self.strategy_with_chaos(),  # strategy_rus
                          action_clients={"move_client": self.move_client,
                                          "manipulator_client": self.manipulator_client,
                                          "stm_client": self.stm_client})

        self.bt_timer = rospy.Timer(rospy.Duration(0.1), self.timer_callback)

    def change_side(self, side):
        if side == SideStatus.PURPLE:
            self.tactics = self.purple_tactics
        elif side == SideStatus.YELLOW:
            self.tactics = self.yellow_tactics
        else:
            self.tactics = None
        self.side_status = side

    def timer_callback(self, event):

        status = self.bt.tick()
        if status != bt.Status.RUNNING:
            self.bt_timer.shutdown()
        print("============== BT LOG ================")
        self.bt.log(0)

    def update_main_coords(self):
        try:
            trans_main = self.tfBuffer.lookup_transform('map', "main_robot", rospy.Time(0))  # 0 means last measurment
            q_main = [trans_main.transform.rotation.x,
                      trans_main.transform.rotation.y,
                      trans_main.transform.rotation.z,
                      trans_main.transform.rotation.w]
            angle_main = euler_from_quaternion(q_main)[2] % (2 * np.pi)
            self.main_coords = np.array([trans_main.transform.translation.x,
                                         trans_main.transform.translation.y,
                                         angle_main])
            rospy.loginfo("main coords: " + str(self.main_coords))
            # return True
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as msg:
            rospy.logwarn(str(msg))
            # return False

    def update_secondary_coords(self):
        try:
            trans_secondary = self.tfBuffer.lookup_transform('map', "secondary_robot", rospy.Time(0))
            q_secondary = [trans_secondary.transform.rotation.x,
                           trans_secondary.transform.rotation.y,
                           trans_secondary.transform.rotation.z,
                           trans_secondary.transform.rotation.w]
            angle_secondary = euler_from_quaternion(q_secondary)[2] % (2 * np.pi)
            self.secondary_coords = np.array([trans_secondary.transform.translation.x,
                                              trans_secondary.transform.translation.y,
                                              angle_secondary])
            
            rospy.loginfo("=============================================================")
            rospy.loginfo("Got coords of secondary robot: ")
            rospy.loginfo(self.secondary_coords)
            return True

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as msg:
            rospy.logwarn(str(msg))
            return False


if __name__ == '__main__':
    try:
        rospy.init_node("main_robot_BT")
        main_robot_bt = MainRobotBT()
        bt_controller = BTController(main_robot_bt)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
