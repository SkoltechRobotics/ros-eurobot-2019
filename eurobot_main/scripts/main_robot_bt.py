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


class Tactics(object):
    def __init__(self):
        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer)
        self.robot_coordinates = None


# class YellowTactics(Tactics):
#     def __init__(self):
#         super(YellowTactics, self).__init__()
#
#         self.approach_dist = rospy.get_param("approach_dist")  # 0.127 meters, distance from robot to puck where robot will try to grab it
#         self.approach_dist = np.array(self.approach_dist)
#         self.approach_vec = np.array([-1*self.approach_dist, 0, 0])  # 0.11
#
#         # can be reached using rospy.get_param("purple_zone/red_cell_puck"
#         self.red_cell_puck = rospy.get_param("yellow_zone/red_cell_puck")
#         self.green_cell_puck = rospy.get_param("yellow_zone/green_cell_puck")
#         self.blue_cell_puck = rospy.get_param("yellow_zone/blue_cell_puck")
#
#         # use find origin
#         self.first_puck_landing = np.array([self.red_cell_puck[0]+self.approach_dist,
#                                            self.red_cell_puck[1],
#                                            3.14])
#
#         self.second_puck_landing = np.array([self.green_cell_puck[0],
#                                             self.green_cell_puck[1]-self.approach_dist,
#                                             1.57])
#
#         self.third_puck_landing = np.array([self.blue_cell_puck[0],
#                                            self.blue_cell_puck[1]-self.approach_dist,
#                                            1.57])
#
#         self.start_zone = rospy.get_param("yellow_zone/start_zone")
#
#         self.blunium_push_PREpose = rospy.get_param("yellow_zone/blunium_push_PREpose")
#         self.blunium_push_pose = rospy.get_param("yellow_zone/blunium_push_pose")
#
#         self.goldenium_PREgrab_pos = rospy.get_param("yellow_zone/goldenium_PREgrab_pos")
#         self.goldenium_grab_pos = rospy.get_param("yellow_zone/goldenium_grab_pos")
#
#         self.scales_area = rospy.get_param("scales_area")
#         self.scales_area = np.array(self.scales_area)
#
#         self.scales_goldenium_PREpos = rospy.get_param("yellow_zone/scales_goldenium_PREpos")
#         self.scales_goldenium_pos = rospy.get_param("yellow_zone/scales_goldenium_pos")


class PurpleTactics(Tactics):
    def __init__(self):
        super(PurpleTactics, self).__init__()

        self.approach_dist = rospy.get_param("approach_dist")
        self.approach_dist = np.array(self.approach_dist)
        self.approach_vec = np.array([-1*self.approach_dist, 0, 0])

        self.red_cell_puck = rospy.get_param("purple_zone/red_cell_puck")
        self.green_cell_puck = rospy.get_param("purple_zone/green_cell_puck")
        self.blue_cell_puck = rospy.get_param("purple_zone/blue_cell_puck")

        # use find origin
        self.first_puck_landing = np.array([self.red_cell_puck[0]-self.approach_dist,
                                           self.red_cell_puck[1],
                                           0])

        self.second_puck_landing = np.array([self.green_cell_puck[0],
                                            self.green_cell_puck[1]-self.approach_dist,
                                            1.57])

        self.third_puck_landing = np.array([self.blue_cell_puck[0],
                                           self.blue_cell_puck[1]-self.approach_dist,
                                           1.57])

        self.start_zone = rospy.get_param("purple_zone/start_zone")

        self.blunium_push_PREpose = rospy.get_param("purple_zone/blunium_push_PREpose")
        self.blunium_push_pose = rospy.get_param("purple_zone/blunium_push_pose")

        self.goldenium_PREgrab_pos = rospy.get_param("purple_zone/goldenium_PREgrab_pos")
        self.goldenium_grab_pos = rospy.get_param("purple_zone/goldenium_grab_pos")

        self.scales_area = rospy.get_param("scales_area")
        self.scales_area = np.array(self.scales_area)

        self.scales_goldenium_PREpos = rospy.get_param("purple_zone/scales_goldenium_PREpos")
        self.scales_goldenium_pos = rospy.get_param("purple_zone/scales_goldenium_pos")


class MainRobotBT(object):
    # noinspection PyTypeChecker
    def __init__(self, side_status=SideStatus.PURPLE):
        self.robot_name = rospy.get_param("robot_name")
        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer)
        self.secondary_coords = None
        self.main_coords = None

        self.move_publisher = rospy.Publisher("navigation/command", String, queue_size=100)
        self.manipulator_publisher = rospy.Publisher("manipulator/command", String, queue_size=100)
        self.stm_publisher = rospy.Publisher("stm/command", String, queue_size=100)

        self.move_client = bt_ros.ActionClient(self.move_publisher)
        self.manipulator_client = bt_ros.ActionClient(self.manipulator_publisher)
        self.stm_client = bt_ros.ActionClient(self.stm_publisher)

        self.purple_tactics = PurpleTactics()
        # self.yellow_tactics = YellowTactics()

        self.side_status = side_status

        if self.side_status == SideStatus.PURPLE:
            self.tactics = self.purple_tactics
        elif self.side_status == SideStatus.YELLOW:
            rospy.loginfo("ERROOOOOOOOR")
            # self.tactics = self.yellow_tactics
        else:
            self.tactics = None

        self.bt = None
        self.bt_timer = None

        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer)

        # Init parameters
        self.known_chaos_pucks = bt.BTVariable(np.array([]))  # (x, y, id, r, g, b)
        self.sorted_chaos_landings = bt.BTVariable(np.array([]))

        self.is_observed = bt.BTVariable(False)

        self.nearest_landing = None
        self.nearest_PRElanding = None
        self.drive_back_point = None

        self.scale_factor = rospy.get_param("scale_factor")  # used in calculating outer bissectrisa for hull's angles
        self.scale_factor = np.array(self.scale_factor)

        self.critical_angle = np.pi * 2/3
        # self.critical_angle = rospy.get_param("critical_angle")  # fixme

        self.red_zone_coords = np.array([0.3, 0.6, -np.pi/3])
        self.approach_dist = rospy.get_param("approach_dist")  # meters, distance from robot to puck where robot will try to grab it
        self.approach_dist = np.array(self.approach_dist)

        self.drive_back_dist = rospy.get_param("drive_back_dist")  # 0.04
        self.drive_back_dist = np.array(self.drive_back_dist)

        self.approach_vec = np.array([-1*self.approach_dist, 0, 0])  # 0.11
        self.drive_back_vec = np.array([-1*self.drive_back_dist, 0, 0])

        self.pucks_subscriber = rospy.Subscriber("/pucks", MarkerArray, self.pucks_callback, queue_size=1)

        # TODO: pucks in front of starting cells are random, so while we aren't using camera
        #       will call them REDIUM  (it doesn't matter, because in this strategy we move them all to acc)

        # TODO: It will matter in case big robot faces hard collision and need to unload pucks in starting cells

        # self.is_puck_grabbed = grab_status  TODO

        self.collected_pucks = bt.BTVariable(np.array([]))  # FIXME change to np.array!
        self.score_master = ScoreController(self.collected_pucks)

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

    # def is_robot_empty_1(self):
    #     rospy.loginfo("pucks inside")
    #     rospy.loginfo(self.collected_pucks.get())
    #     if len(self.collected_pucks.get()) == 0:
    #         rospy.loginfo('All pucks unloaded')
    #         return bt.Status.SUCCESS
    #     else:
    #         rospy.loginfo('Pucks inside: ' + str(len(self.collected_pucks.get())))
    #         return bt.Status.RUNNING

    def is_scales_landing_free(self):
        area = self.tactics.scales_area
        robot = self.secondary_coords

        self.update_secondary_coords()
        if self.secondary_coords is None:
            return bt.Status.RUNNING

        point = Point(robot[0], robot[1])
        polygon = Polygon([area[0], area[1], area[2], area[3]])

        rospy.loginfo("Checking if scales are available to approach...")

        if polygon.contains(point):
            rospy.loginfo('Landing busy')
            return bt.Status.RUNNING
        else:
            rospy.loginfo('Landing is free to go')
            return bt.Status.SUCCESS

    def pucks_callback(self, data):
        self.is_observed.set(True)
        # [(0.95, 1.1, 3, 0, 0, 1), ...] - blue, id=3  IDs are not guaranteed to be the same from frame to frame

        if self.known_chaos_pucks.get().size == 0:
            try:
                new_observation_pucks = [[marker.pose.position.x,
                                          marker.pose.position.y,
                                          marker.id,
                                          marker.color.r,
                                          marker.color.g,
                                          marker.color.b] for marker in data.markers]

                self.known_chaos_pucks.set(np.append(self.known_chaos_pucks.get(), new_observation_pucks))  # Action
                rospy.loginfo("Got pucks observation:")
                rospy.loginfo(self.known_chaos_pucks.get())
                self.pucks_subscriber.unregister()

            except Exception:  # FIXME
                rospy.loginfo("list index out of range - no visible pucks on the field ")

    def is_chaos_collected_completely(self):
        if self.known_chaos_pucks.get().size == 0:
            rospy.loginfo("Chaos collected completely")
            return bt.Status.SUCCESS
        else:
            return bt.Status.FAILED

    # FIXME remove this
    def is_chaos_collected_completely1(self):
        if self.known_chaos_pucks.get().size == 0:
            rospy.loginfo("Chaos collected completely")
            return bt.Status.SUCCESS
        else:
            return bt.Status.RUNNING

    def is_chaos_observed(self):  # Condition
        if self.is_observed.get():
            return bt.Status.SUCCESS
        else:
            return bt.Status.RUNNING

    # FIXME inside
    def calculate_pucks_configuration(self):
        """

        :return: # [(0.95, 1.1, 3, 0, 0, 1), ...]
        """
        self.known_chaos_pucks = sort_wrt_robot(self.main_coords, self.known_chaos_pucks)

        if len(self.known_chaos_pucks) >= 3:
            is_hull_safe_to_approach, coords_sorted_by_angle = sort_by_inner_angle_and_check_if_safe(self.main_coords,
                                                                                                     self.known_chaos_pucks,
                                                                                                     self.critical_angle)
            if is_hull_safe_to_approach:  # only sharp angles
                rospy.loginfo("hull is SAFE to approach, sorted wrt robot")

            if not is_hull_safe_to_approach:
                known_chaos_pucks = coords_sorted_by_angle  # calc vert-angle, sort by angle, return vertices (sorted)
                rospy.loginfo("hull is not safe to approach, sorted by angle")

        # when we finally sorted them, chec if one of them is blue. If so, roll it so blue becomes last one to collect
        if len(self.known_chaos_pucks) > 1 and all(self.known_chaos_pucks[0][3:6] == [0, 0, 1]):
            self.known_chaos_pucks = np.roll(self.known_chaos_pucks, -1, axis=0)
            rospy.loginfo("blue rolled")

    def calculate_landings(self):
        """

        :return: [(x, y, theta), ...]
        """
        coords = self.known_chaos_pucks[:, :2]
        if len(coords) == 1:
            landings = calculate_closest_landing_to_point(self.main_coords, coords, self.approach_vec)
            self.sorted_chaos_landings.set(np.append(self.sorted_chaos_landings.get(), landings))
        else:
            landings = unleash_power_of_geometry(coords, self.scale_factor, self.approach_dist)
            self.sorted_chaos_landings.set(np.append(self.sorted_chaos_landings.get(), landings))

    def choose_new_landing(self):
        self.nearest_landing = self.sorted_chaos_landings.get()[0]
        rospy.loginfo("Nearest landing chosen: " + str(self.nearest_landing))

    def calculate_prelanding(self):
        self.nearest_PRElanding = cvt_local2global(self.drive_back_vec, self.nearest_landing)
        rospy.loginfo("Nearest PRElanding calculated: " + str(self.nearest_landing))

    def calculate_drive_back_point(self):
        self.drive_back_point = cvt_local2global(self.drive_back_vec, self.nearest_landing)
        rospy.loginfo("Nearest drive_back_point calculated: " + str(self.drive_back_point))

    def is_last_puck(self):
        if self.known_chaos_pucks.get().size == 1:
            return bt.Status.SUCCESS
        else:
            return bt.Status.RUNNING

    def strategy_msc(self):
        red_cell_puck = bt.SequenceWithMemoryNode([
                            bt_ros.SetManipulatortoWall("manipulator_client"),
                            bt_ros.MoveLineToPoint(self.tactics.first_puck_landing, "move_client"),
                            bt_ros.StartCollectGround("manipulator_client"),
                            bt.ActionNode(lambda: self.score_master.add("REDIUM"))  # FIXME: color is undetermined without camera!
                        ])

        green_cell_puck = bt.SequenceWithMemoryNode([
                            bt.ParallelWithMemoryNode([
                                bt_ros.CompleteCollectGround("manipulator_client"),
                                bt_ros.MoveLineToPoint(self.tactics.second_puck_landing, "move_client"),
                            ], threshold=2),
                            bt_ros.StartCollectGround("manipulator_client"),
                            bt.ActionNode(lambda: self.score_master.add("REDIUM")) 
                        ])

        blue_cell_puck = bt.SequenceWithMemoryNode([
                            bt.ParallelWithMemoryNode([
                                bt_ros.CompleteCollectGround("manipulator_client"),
                                bt_ros.MoveLineToPoint(self.tactics.third_puck_landing, "move_client"),
                            ], threshold=2),
                            bt_ros.StartCollectGround("manipulator_client"),
                            bt.ActionNode(lambda: self.score_master.add("REDIUM"))  # FIXME
                        ])

        go_to_blunium = bt.ParallelWithMemoryNode([
                            bt_ros.SetManipulatortoUp("manipulator_client"),
                            bt_ros.CompleteCollectGround("manipulator_client"),
                            bt_ros.MoveLineToPoint(self.tactics.blunium_push_PREpose, "move_client"),
                            bt_ros.StepUp("manipulator_client"),  # FIXME do we need to do that? NO if all 7 pucks inside
                        ], threshold=4)

        push_blunium = bt.SequenceWithMemoryNode([
                        # bt_ros.SetManipulatorToPushBlunium("manipulator_client"),
                        bt_ros.MoveLineToPoint(self.tactics.blunium_push_pose, "move_client"),
                        bt_ros.SetSpeedSTM([0, -0.1, 0], 1, "stm_client"),
                        bt.ActionNode(lambda: self.score_master.add("BLUNIUM")),
                        bt.ActionNode(lambda: self.score_master.unload("ACC")),
                        bt.ActionNode(lambda: self.score_master.reward("UNLOCK_GOLDENIUM_BONUS")),
                    ])

        # works
        # unload = bt.SequenceNode([
        #                 bt.FallbackNode([
        #                     bt.ConditionNode(self.is_robot_empty),
        #                     bt.SequenceWithMemoryNode([
        #                         bt_ros.UnloadAccelerator("manipulator_client"),
        #                         bt.ActionNode(lambda: self.score_master.unload("ACC")),
        #                     ])
        #                 ]),
        #                 bt.ConditionNode(self.is_robot_empty_1)
        #             ])

        # NEED TO TEST
        unload = bt.FallbackNode([
                    bt.ConditionNode(self.is_robot_empty),
                    bt.SequenceWithMemoryNode([
                        bt_ros.UnloadAccelerator("manipulator_client"),
                        bt.ActionNode(lambda: self.score_master.unload("ACC")),
                        bt.ConditionNode(lambda: bt.Status.RUNNING)
                    ])
                ])

        collect_goldenium = bt.SequenceWithMemoryNode([
                                bt_ros.SetSpeedSTM([0, 0.3, 0], 1, "stm_client"),
                                bt_ros.MoveLineToPoint(self.tactics.goldenium_PREgrab_pos, "move_client"),
                                bt.ParallelWithMemoryNode([
                                    bt_ros.SetManipulatortoGoldenium("manipulator_client"),
                                    bt_ros.MoveLineToPoint(self.tactics.goldenium_grab_pos, "move_client"),
                                ], threshold=2),
                                bt_ros.GrabGoldeniumAndHoldUp("manipulator_client"),
                                bt.ActionNode(lambda: self.score_master.add("GOLDENIUM")),
                                bt.ActionNode(lambda: self.score_master.reward("GRAB_GOLDENIUM_BONUS")),
                            ])

        move_to_goldenium_prepose = bt_ros.MoveLineToPoint(self.tactics.scales_goldenium_PREpos, "move_client")

        unload_goldenium = bt.SequenceNode([
                                bt.ConditionNode(self.is_scales_landing_free),
                                bt.SequenceWithMemoryNode([
                                    bt_ros.MoveLineToPoint(self.tactics.scales_goldenium_pos, "move_client"),
                                    bt_ros.UnloadGoldenium("manipulator_client"),
                                    bt.ActionNode(lambda: self.score_master.unload("SCALES"))
                                ])
        ])

        # calculate and move to first
        move_to_chaos = bt.SequenceWithMemoryNode([
                            bt.ActionNode(self.update_main_coords),  # FIXME check with Alexey
                            bt.ActionNode(self.calculate_pucks_configuration),
                            bt.ActionNode(self.calculate_landings),
                            bt.ActionNode(self.choose_new_landing),
                            bt.ActionNode(self.calculate_prelanding),

                            bt_ros.MoveLineToPoint(self.nearest_PRElanding, "move_client"),
                            bt_ros.MoveLineToPoint(self.nearest_landing, "move_client"),
                        ])

        collect_chaos = bt.SequenceWithMemoryNode([
                            bt.FallbackNode([
                                # completely?
                                bt.ConditionNode(self.is_chaos_collected_completely),

                                # if last puck, just collect it, return success and get fuck out of here
                                bt.SequenceWithMemoryNode([
                                    bt.ConditionNode(self.is_last_puck),
                                    bt.SequenceWithMemoryNode([
                                        bt_ros.StartCollectGround("manipulator_client"),
                                        bt.ActionNode(lambda: self.score_master.add("BLUNIUM")),  # FIXME get color from camera
                                        bt_ros.CompleteCollectGround("manipulator_client")
                                    ])
                                ]),

                                # calc config and start collect
                                bt.SequenceWithMemoryNode([
                                    bt_ros.StartCollectGround("manipulator_client"),
                                    bt.ActionNode(self.calculate_drive_back_point),
                                    bt_ros.MoveLineToPoint(self.drive_back_point, "move_client"), # FIXME make it closer

                                    # calc new landing
                                    bt.ActionNode(self.calculate_pucks_configuration),
                                    bt.ActionNode(self.calculate_landings),

                                    # drive back, collect and move to new prelanding
                                    bt.ParallelWithMemoryNode([
                                        bt_ros.CompleteCollectGround("manipulator_client"),
                                        bt.ActionNode(lambda: self.score_master.add("BLUNIUM")),  # FIXME get color from camera
                                        bt_ros.MoveLineToPoint(self.nearest_PRElanding, "move_client"),
                                    ], threshold=3),

                                    bt_ros.MoveLineToPoint(self.nearest_landing, "move_client"),
                                ]),
                            ]),
                            bt.ConditionNode(lambda: self.is_chaos_collected_completely1())
                        ])

        move_finish = bt.SequenceWithMemoryNode([
                        # bt_ros.MoveLineToPoint(self.tactics.first_puck_landing, "move_client"),
                        bt_ros.MoveLineToPoint(self.tactics.start_zone, "move_client"),
                    ])

        strategy = bt.SequenceWithMemoryNode([
                        red_cell_puck,
                        green_cell_puck,
                        blue_cell_puck,
                        go_to_blunium,
                        push_blunium,
                        # go_to_acc,
                        unload,
                        # careful_approach,
                        collect_goldenium,
                        move_to_goldenium_prepose,
                        unload_goldenium,
                        move_finish])

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
            # self.tactics = self.yellow_tactics
            rospy.loginfo("Error 2")
        else:
            self.tactics = None

    def timer_callback(self, event):

        status = self.bt.tick()
        if status != bt.Status.RUNNING:
            self.bt_timer.shutdown()
        print("============== BT LOG ================")
        self.bt.log(0)

    def update_main_coords(self):
        try:
            trans_main = self.tfBuffer.lookup_transform('map', "main_robot", rospy.Time())
            q_main = [trans_main.transform.rotation.x,
                      trans_main.transform.rotation.y,
                      trans_main.transform.rotation.z,
                      trans_main.transform.rotation.w]
            angle_main = euler_from_quaternion(q_main)[2] % (2 * np.pi)
            self.main_coords = np.array([trans_main.transform.translation.x,
                                         trans_main.transform.translation.y,
                                         angle_main])
            return True
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as msg:
            rospy.logwarn(str(msg))
            return False

    def update_secondary_coords(self):
        try:
            trans_secondary = self.tfBuffer.lookup_transform('map', "secondary_robot", rospy.Time())
            q_secondary = [trans_secondary.transform.rotation.x,
                           trans_secondary.transform.rotation.y,
                           trans_secondary.transform.rotation.z,
                           trans_secondary.transform.rotation.w]
            angle_secondary = euler_from_quaternion(q_secondary)[2] % (2 * np.pi)
            self.secondary_coords = np.array([trans_secondary.transform.translation.x,
                                              trans_secondary.transform.translation.y,
                                              angle_secondary])
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
