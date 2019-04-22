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


class YellowTactics(Tactics):
    def __init__(self):
        super(YellowTactics, self).__init__()
        self.red_cell_puck = rospy.get_param("yellow_side/red_cell_puck")
        self.blunium = rospy.get_param("yellow_side/blunium")
        self.goldenium = rospy.get_param("yellow_side/goldenium")
        self.scales_area = rospy.get_param("yellow_side/scales_area")
        self.scales_area = np.array(self.scales_area)
        self.chaos_center = rospy.get_param("yellow_side/chaos_center")

        self.start_zone = np.array([self.red_cell_puck[0] + self.ground_spacing_dist,
                                    self.red_cell_puck[1],
                                    -1.57])

        self.first_puck_landing = np.array([self.red_cell_puck[0]+self.approach_dist,
                                           self.red_cell_puck[1],
                                           3.14])

        self.first_puck_landing_finish = np.array([self.red_cell_puck[0],
                                                    self.red_cell_puck[1] - 0.04,
                                                    1.57])

        self.second_puck_landing = np.array([self.red_cell_puck[0],
                                            self.red_cell_puck[1] + self.ground_spacing_dist - self.approach_dist,
                                            1.57])

        self.third_puck_landing = np.array([self.red_cell_puck[0],
                                           self.red_cell_puck[1] + 2*self.ground_spacing_dist - self.approach_dist,
                                           1.57])

        self.third_puck_rotate_pose = np.array([self.third_puck_landing[0],
                                                self.third_puck_landing[1] - 0.05,
                                                -2.35])

        self.blunium_prepose = np.array([self.blunium[0] + 0.07,
                                         self.blunium[1] + 0.35,
                                         -0.52])

        self.blunium_start_push_pose = np.array([self.blunium_prepose[0],
                                                 self.blunium[1] + self.robot_outer_radius,
                                                 self.blunium_prepose[2]])  # + self.stick_len

        self.blunium_end_push_pose = np.array([self.blunium_start_push_pose[0] - 0.08,
                                               self.blunium_start_push_pose[1],
                                               self.blunium_start_push_pose[2]])

        self.blunium_get_back_pose = np.array([self.blunium_end_push_pose[0],
                                               self.blunium_end_push_pose[1] + 0.1,
                                               self.blunium_end_push_pose[2]])

        self.accelerator_PREunloading_pos = np.array([self.blunium_end_push_pose[0] - 0.22,
                                                       self.blunium_end_push_pose[1] - 0.05,
                                                       0.56])

        self.goldenium_1_PREgrab_pos = np.array([self.goldenium[0],
                                               self.goldenium[1] + 0.4,
                                               self.accelerator_PREunloading_pos[2]])

        self.goldenium_2_PREgrab_pos = np.array([self.goldenium[0],
                                               self.goldenium_1_PREgrab_pos[1] - 0.1,
                                               -1.57])

        self.goldenium_grab_pos = np.array([self.goldenium[0],
                                               self.goldenium_2_PREgrab_pos[1] - 0.1,
                                               self.goldenium_2_PREgrab_pos[2]])

        self.goldenium_back_rot_pose = np.array([self.goldenium_grab_pos[0],
                                                 self.goldenium_grab_pos[1] + 0.06,
                                                 1])

        self.scales_goldenium_PREpos = np.array([self.chaos_center[0] - 0.25,
                                                    self.chaos_center[1] - 0.15,
                                                    1.4])

        self.scales_goldenium_pos = np.array([self.scales_goldenium_PREpos[0] - 0.05,
                                              self.scales_goldenium_PREpos[1] + 0.51,
                                              1.83])


class PurpleTactics(Tactics):
    def __init__(self):
        super(PurpleTactics, self).__init__()

        self.red_cell_puck = rospy.get_param("purple_side/red_cell_puck")
        self.scales_area = rospy.get_param("purple_side/scales_area")
        self.scales_area = np.array(self.scales_area)
        self.chaos_center = rospy.get_param("purple_side/chaos_center")
        self.goldenium = rospy.get_param("purple_side/goldenium")
        self.blunium = rospy.get_param("purple_side/blunium")

        self.start_zone = np.array([self.red_cell_puck[0] - self.ground_spacing_dist,
                                    self.red_cell_puck[1],
                                    -1.57])

        self.first_puck_landing = np.array([self.red_cell_puck[0] - self.approach_dist,
                                           self.red_cell_puck[1],
                                           3.14])

        self.first_puck_landing_finish = np.array([self.red_cell_puck[0],
                                                    self.red_cell_puck[1] - 0.04,
                                                    1.57])

        self.second_puck_landing = np.array([self.red_cell_puck[0],
                                            self.red_cell_puck[1] + self.ground_spacing_dist - self.approach_dist,
                                            1.57])

        self.third_puck_landing = np.array([self.red_cell_puck[0],
                                           self.red_cell_puck[1] + 2*self.ground_spacing_dist - self.approach_dist,
                                           1.57])

        self.third_puck_rotate_pose = np.array([self.third_puck_landing[0],
                                                self.third_puck_landing[1] - 0.05,
                                                -2.35])

        self.blunium_prepose = np.array([self.blunium[0] + 0.07,
                                         self.blunium[1] + 0.35,
                                         -0.52])

        self.blunium_start_push_pose = np.array([self.blunium_prepose[0],
                                                 self.blunium[1] + self.robot_outer_radius,
                                                 self.blunium_prepose[2]])  # + self.stick_len

        self.blunium_end_push_pose = np.array([self.blunium_start_push_pose[0] + 0.08,
                                               self.blunium_start_push_pose[1],
                                               self.blunium_start_push_pose[2]])

        self.blunium_get_back_pose = np.array([self.blunium_end_push_pose[0],
                                               self.blunium_end_push_pose[1] + 0.1,
                                               self.blunium_end_push_pose[2]])

        self.accelerator_PREunloading_pos = np.array([self.blunium_end_push_pose[0] + 0.22,
                                                       self.blunium_end_push_pose[1] - 0.05,
                                                       0.56])

        self.goldenium_1_PREgrab_pos = np.array([self.goldenium[0],
                                               self.goldenium[1] + 0.35,
                                               self.accelerator_PREunloading_pos[2]])

        self.goldenium_2_PREgrab_pos = np.array([self.goldenium[0],
                                               self.goldenium_1_PREgrab_pos[1] - 0.1,
                                               -1.57])

        self.goldenium_grab_pos = np.array([self.goldenium[0],
                                               self.goldenium_2_PREgrab_pos[1] - 0.1,
                                               self.goldenium_2_PREgrab_pos[2]])

        self.goldenium_back_rot_pose = np.array([self.goldenium_grab_pos[0],
                                                 self.goldenium_grab_pos[1] + 0.06,
                                                 2])

        self.scales_goldenium_PREpos = np.array([self.chaos_center[0] + 0.25,
                                                    self.chaos_center[1] - 0.15,
                                                    1.4])

        self.scales_goldenium_pos = np.array([self.scales_goldenium_PREpos[0] + 0.05,
                                              self.scales_goldenium_PREpos[1] + 0.51,
                                              1.57])


class MainRobotBT(object):
    # noinspection PyTypeChecker
    def __init__(self):  # fixme  YELLOW  PURPLE  side_status=SideStatus.PURPLE
        self.robot_name = rospy.get_param("robot_name")
        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer)
        self.secondary_coords = np.array([0, 0, 0])
        self.main_coords = None

        # self.manipulator = Manipulator()

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

        # self.known_chaos_pucks = bt.BTVariable(np.array([]))  # (x, y, id, r, g, b)  # FIXME
        self.known_chaos_pucks = bt.BTVariable(np.array([[1.7, 0.8, 1, 1, 0, 0],
                                                            [1.9, 0.9, 2, 0, 1, 0],
                                                            [2.1, 0.85, 3, 0, 0, 1],
                                                            [1.9, 1.1, 4, 1, 0, 0]]))  # (x, y, id, r, g, b)

        self.sorted_chaos_landings = bt.BTVariable(np.array([]))

        self.is_observed = bt.BTVariable(True)  # FIXME to FALSE!!!!!!!!!!!!!!!!!!
        self.is_secondary_working = False




        self.move_to_waypoint_node1 = bt_ros.ActionClientNode("move 0 0 0", "move_client", name="move_to_waypoint")
        self.move_to_waypoint_node2 = bt_ros.ActionClientNode("move 0 0 0", "move_client", name="move_to_waypoint")
        # self.choose_new_landing_latch = bt.ActionNode(self.choose_new_landing)
        # self.calculate_prelanding_latch = bt.ActionNode(self.calculate_prelanding)
        # self.calculate_drive_back_point_latch = bt.ActionNode(self.calculate_drive_back_point)


        # self.choose_new_landing_latch = bt.Latch(bt.ActionNode(self.choose_new_landing))
        # self.calculate_prelanding_latch = bt.Latch(bt.ActionNode(self.calculate_prelanding))
        # self.calculate_drive_back_point_latch = bt.Latch(bt.ActionNode(self.calculate_drive_back_point))

        self.nearest_landing = bt.BTVariable(np.array([1, 1, 1.57])) # None
        # self.nearest_PRElanding = bt.BTVariable(np.array([1, 1, 3.14]))
        # self.drive_back_point = bt.BTVariable(np.array([1, 1, 0]))

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

        # self.is_puck_grabbed = bt.BTVariable(False)
        self.incoming_puck_color = bt.BTVariable(None)
        self.collected_pucks = bt.BTVariable(np.array([]))
        self.score_master = ScoreController(self.collected_pucks)

        # self.pucks_subscriber = rospy.Subscriber("/pucks", MarkerArray, self.pucks_callback, queue_size=1)
        rospy.Subscriber("navigation/response", String, self.move_client.response_callback)
        rospy.Subscriber("manipulator/response", String, self.manipulator_client.response_callback)

    # def is_puck_grabbed(self):
    #     if manipulator.check_status_infinitely():
    #         return bt.Status.SUCCESS
    #     else:
    #         return bt.Status.FAILED

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

    # def pucks_callback(self, data):
    #     self.is_observed.set(True)
    #     # [(0.95, 1.1, 3, 0, 0, 1), ...] - blue, id=3  IDs are not guaranteed to be the same from frame to frame
    #     # red (1, 0, 0)
    #     # green (0, 1, 0)
    #     # blue (0, 0, 1)
    #
    #     if self.known_chaos_pucks.get().size == 0:
    #         try:
    #             new_observation_pucks = [[marker.pose.position.x,
    #                                       marker.pose.position.y,
    #                                       marker.id,
    #                                       marker.color.r,
    #                                       marker.color.g,
    #                                       marker.color.b] for marker in data.markers]
    #
    #             self.known_chaos_pucks.set(np.append(self.known_chaos_pucks.get(), new_observation_pucks))  # Action
    #             rospy.loginfo("Got pucks observation:")
    #             rospy.loginfo(self.known_chaos_pucks.get())
    #             self.pucks_subscriber.unregister()
    #
    #         except Exception:  # FIXME
    #             rospy.loginfo("list index out of range - no visible pucks on the field ")

    def is_chaos_collected_completely(self):
        if self.known_chaos_pucks.get().size == 0:
            rospy.loginfo("Chaos collected completely")
            return bt.Status.SUCCESS
        else:
            return bt.Status.FAILED

    def is_chaos_collected_completely1(self):
        if self.known_chaos_pucks.get().size == 0:
            rospy.loginfo("Chaos collected completely")
            return bt.Status.SUCCESS
        else:
            return bt.Status.RUNNING

    def is_chaos_observed(self):
        if self.is_observed.get():
            return bt.Status.SUCCESS
        else:
            return bt.Status.RUNNING

    def calculate_pucks_configuration(self):
        """

        :return: # [(0.95, 1.1, 3, 0, 0, 1), ...]
        """
        self.update_main_coords()
        known_chaos_pucks = sort_wrt_robot(self.main_coords, self.known_chaos_pucks.get())

        self.known_chaos_pucks.set(known_chaos_pucks)

        if self.known_chaos_pucks.get().size >= 3:
            is_hull_safe_to_approach, coords_sorted_by_angle = sort_by_inner_angle_and_check_if_safe(self.main_coords,
                                                                                                     self.known_chaos_pucks.get(),
                                                                                                     self.critical_angle)
            if is_hull_safe_to_approach:  # only sharp angles
                rospy.loginfo("hull is SAFE to approach, sorted wrt robot")

            if not is_hull_safe_to_approach:
                self.known_chaos_pucks.set(coords_sorted_by_angle)  # calc vert-angle, sort by angle, return vertices (sorted)
                # known_chaos_pucks = coords_sorted_by_angle
                rospy.loginfo("hull is not safe to approach, sorted by angle")

        # when we finally sorted them, chec if one of them is blue. If so, roll it so blue becomes last one to collect
        # if self.known_chaos_pucks.get().size > 1 and all(self.known_chaos_pucks.get()[0][3:6] == [0, 0, 1]):
        #     # self.known_chaos_pucks.set(np.roll(self.known_chaos_pucks.get(), -1, axis=0))
        #     rospy.loginfo("blue rolled")

    def calculate_landings(self):
        """

        :return: [(x, y, theta), ...]
        """
        print "calculating landings"
        coords = self.known_chaos_pucks.get()[:, :2]
        if coords.size == 1:
            landings = calculate_closest_landing_to_point(self.main_coords, coords, self.approach_vec)
            self.sorted_chaos_landings.set(landings)
        else:
            landings = unleash_power_of_geometry(coords, self.scale_factor, self.approach_dist)
            self.sorted_chaos_landings.set(landings)
        print("sorted landings")
        print(self.sorted_chaos_landings.get())














    # def choose_new_landing(self):
    #     self.calculate_pucks_configuration()
    #     self.calculate_landings()
    #
    #     nearest_landing = self.sorted_chaos_landings.get()[0]
    #     self.nearest_landing.set(nearest_landing)
    #     rospy.loginfo("Nearest landing chosen: " + str(nearest_landing))
    #     self.move_to_waypoint_node2.cmd.set("move_line %f %f %f" % tuple(nearest_landing))
    #
    # def calculate_prelanding(self):
    #     self.calculate_pucks_configuration()
    #     self.calculate_landings()
    #
    #     nearest_landing = self.sorted_chaos_landings.get()[0]
    #     nearest_PRElanding = cvt_local2global(self.drive_back_vec, nearest_landing)
    #     rospy.loginfo("Nearest PRElanding calculated: " + str(nearest_PRElanding))
    #     self.move_to_waypoint_node1.cmd.set("move_line %f %f %f" % tuple(nearest_PRElanding))
    #
    # def calculate_drive_back_point(self):
    #     drive_back_point = cvt_local2global(self.drive_back_vec, self.nearest_landing.get())
    #     rospy.loginfo("Nearest drive_back_point calculated: " + str(drive_back_point))
    #     self.move_to_waypoint_node.cmd.set("move_line %f %f %f" % tuple(drive_back_point))















    def is_last_puck(self):
        if self.known_chaos_pucks.get().size == 1:
            return bt.Status.SUCCESS
        else:
            return bt.Status.RUNNING

    @staticmethod
    def get_color(puck):
        """
        red (1, 0, 0)
        green (0, 1, 0)
        blue (0, 0, 1)
        :param puck: (x, y, id, 0, 0, 1)
        :return:
        """
        pucks_colors = {
            (1, 0, 0): "REDIUM",
            (0, 1, 0): "GREENIUM",
            (0, 0, 1): "BLUNIUM"
        }
        color_key = puck[3:]
        color_val = pucks_colors.get(color_key)
        return color_val

    def update_chaos_pucks(self):
        """
        delete taken puck from known on the field
        get color of last taken puck
        :return: None
        """
        incoming_puck_color = self.get_color(self.known_chaos_pucks.get()[0])
        self.incoming_puck_color.set(incoming_puck_color)
        self.known_chaos_pucks.set(np.delete(self.known_chaos_pucks.get(), 0, axis=0))

    def strategy_msc(self):
        red_cell_puck = bt.SequenceWithMemoryNode([
                            bt_ros.MoveLineToPoint(self.tactics.first_puck_landing, "move_client"),
                            bt.FallbackWithMemoryNode([
                                bt.SequenceWithMemoryNode([
                                    bt_ros.StartCollectGround("manipulator_client"),
                                    bt.ActionNode(lambda: self.score_master.add("REDIUM")),  # FIXME: color is undetermined without camera!
                                    bt.ParallelWithMemoryNode([
                                        bt_ros.CompleteCollectGround("manipulator_client"),
                                        bt_ros.MoveLineToPoint(self.tactics.first_puck_landing_finish, "move_client"),
                                    ], threshold=2),
                                ]),
                                bt.ParallelWithMemoryNode([
                                    bt_ros.SetManipulatortoWall("manipulator_client"),
                                    bt_ros.MoveLineToPoint(self.tactics.first_puck_landing_finish, "move_client"),
                                ], threshold=2),
                            ]),
                        ])

        green_cell_puck = bt.SequenceWithMemoryNode([
                            bt_ros.MoveLineToPoint(self.tactics.second_puck_landing, "move_client"),
                            bt.FallbackWithMemoryNode([
                                bt.SequenceWithMemoryNode([
                                    bt_ros.StartCollectGround("manipulator_client"),
                                    bt.ActionNode(lambda: self.score_master.add("REDIUM")),  # FIXME: color is undetermined without camera!
                                    bt.ParallelWithMemoryNode([
                                        bt_ros.CompleteCollectGround("manipulator_client"),
                                        bt_ros.MoveLineToPoint(self.tactics.third_puck_landing, "move_client"),
                                    ], threshold=2),
                                ]),
                                bt.ParallelWithMemoryNode([
                                    bt_ros.SetManipulatortoWall("manipulator_client"),
                                    bt_ros.MoveLineToPoint(self.tactics.third_puck_landing, "move_client"),
                                ], threshold=2),
                            ])
                        ])

        blue_cell_puck = bt.SequenceWithMemoryNode([
                            bt.FallbackWithMemoryNode([
                                bt.SequenceWithMemoryNode([
                                    bt_ros.StartCollectGround("manipulator_client"),
                                    bt.ActionNode(lambda: self.score_master.add("REDIUM")),  # FIXME: color is undetermined without camera!
                                    bt.ParallelWithMemoryNode([
                                        bt_ros.CompleteCollectGround("manipulator_client"),
                                        bt_ros.MoveLineToPoint(self.tactics.third_puck_rotate_pose, "move_client"),
                                    ], threshold=2),
                                ]),
                                bt.ParallelWithMemoryNode([
                                    bt_ros.SetManipulatortoUp("manipulator_client"),
                                    bt_ros.MoveLineToPoint(self.tactics.third_puck_rotate_pose, "move_client"),
                                ], threshold=2),
                            ])
                        ])

        finish_move_blunium_and_push = bt.SequenceWithMemoryNode([
                                            # bt_ros.MoveLineToPoint(self.tactics.blunium_prepose, "move_client"),
                                            bt_ros.MoveLineToPoint(self.tactics.blunium_start_push_pose, "move_client"),
                                            bt_ros.MoveLineToPoint(self.tactics.blunium_end_push_pose, "move_client"),
                                            bt.ActionNode(lambda: self.score_master.add("BLUNIUM")),
                                            bt.ActionNode(lambda: self.score_master.unload("ACC")),
                                            bt.ActionNode(lambda: self.score_master.reward("UNLOCK_GOLDENIUM_BONUS")),
                                        ])
        
        approach_acc = bt.SequenceWithMemoryNode([
                            bt_ros.MoveLineToPoint(self.tactics.blunium_get_back_pose, "move_client"),
                            bt_ros.MoveLineToPoint(self.tactics.accelerator_PREunloading_pos, "move_client"),  # FIXME try Arc
                            bt_ros.SetSpeedSTM([0, -0.1, 0], 0.6, "stm_client"),
                        ])
        
        unload_first_in_acc = bt.SequenceWithMemoryNode([
                                    bt_ros.StepperUp("manipulator_client"),  # FIXME do we need to do that? NO if all 7 pucks inside
                                    bt_ros.UnloadAccelerator("manipulator_client"),
                                    bt.ActionNode(lambda: self.score_master.unload("ACC")),
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
                                bt_ros.MoveLineToPoint(self.tactics.goldenium_1_PREgrab_pos, "move_client"),
                                bt_ros.MoveLineToPoint(self.tactics.goldenium_2_PREgrab_pos, "move_client"),
                                bt_ros.StartCollectGoldenium("manipulator_client"),
                                bt_ros.MoveLineToPoint(self.tactics.goldenium_grab_pos, "move_client"),
                                bt_ros.GrabGoldeniumAndHoldUp("manipulator_client"),
                                bt.ActionNode(lambda: self.score_master.add("GOLDENIUM")),
                                bt.ActionNode(lambda: self.score_master.reward("GRAB_GOLDENIUM_BONUS")),
                            ])
        
        move_to_goldenium_prepose = bt.SequenceWithMemoryNode([
                                        bt_ros.MoveLineToPoint(self.tactics.goldenium_back_rot_pose, "move_client"),
                                        bt_ros.MoveLineToPoint(self.tactics.scales_goldenium_PREpos, "move_client")
                                    ])
        
        unload_goldenium = bt.SequenceWithMemoryNode([
                                bt.ConditionNode(self.is_scales_landing_free),
                                bt.SequenceWithMemoryNode([
                                    bt_ros.MoveLineToPoint(self.tactics.scales_goldenium_pos + np.array([0, -0.05, 0]), "move_client"),
                                    bt_ros.SetManipulatortoWall("manipulator_client"),
                                    bt_ros.MoveLineToPoint(self.tactics.scales_goldenium_pos, "move_client"),
                                    bt_ros.UnloadGoldenium("manipulator_client"),
                                    bt.ActionNode(lambda: self.score_master.unload("SCALES"))
                                ])
                            ])


        # move_to_chaos = bt.SequenceWithMemoryNode([
        #                     self.calculate_prelanding_latch,
        #                     self.move_to_waypoint_node1,
        #
        #                     self.choose_new_landing_latch,
        #                     self.move_to_waypoint_node2,
        #                 ])

        # TODO take into account, that when colecting 7 pucks, don't make step down
        # collect_chaos = bt.SequenceNode([
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
        #                             self.calculate_drive_back_point_latch,
        #                             self.move_to_waypoint_node, # FIXME make it closer

        #                             # calc new landing
        #                             bt.ActionNode(self.calculate_pucks_configuration),
        #                             bt.ActionNode(self.calculate_landings),

        #                             # drive back, collect and move to new prelanding
        #                             bt.ParallelWithMemoryNode([
        #                                 bt_ros.CompleteCollectGround("manipulator_client"),
        #                                 bt.ActionNode(self.update_chaos_pucks),
        #                                 bt.ActionNode(lambda: self.score_master.add(self.incoming_puck_color.get())),
        #                                 self.calculate_prelanding_latch,
        #                                 self.move_to_waypoint_node,
        #                             ], threshold=5),

        #                             bt_ros.MoveLineToPoint(self.nearest_landing.get(), "move_client"),
        #                         ]),
        #                     ]),
        #                     bt.ConditionNode(lambda: self.is_chaos_collected_completely1())
        #                 ])

        strategy = bt.SequenceWithMemoryNode([
                        red_cell_puck,
                        green_cell_puck,
                        blue_cell_puck,
                        finish_move_blunium_and_push,
                        approach_acc,
                        unload_first_in_acc,
                        unload_acc,
                        collect_goldenium,
                        move_to_goldenium_prepose,
                        unload_goldenium,
                        # move_to_chaos,
                        # collect_chaos
                        ])

        return strategy

    def start(self):

        self.bt = bt.Root(self.strategy_msc(),
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
