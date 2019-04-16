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


class YellowTactics(Tactics):
    def __init__(self):
        super(YellowTactics, self).__init__()

        self.approach_dist = rospy.get_param("approach_dist")  # 0.127 meters, distance from robot to puck where robot will try to grab it
        self.approach_dist = np.array(self.approach_dist)
        self.approach_vec = np.array([-1*self.approach_dist, 0, 0])  # 0.11

        self.red_cell_puck = rospy.get_param("yellow_side/red_cell_puck")
        self.green_cell_puck = rospy.get_param("yellow_side/green_cell_puck")
        self.blue_cell_puck = rospy.get_param("yellow_side/blue_cell_puck")
        self.third_puck_rotate_pose = rospy.get_param("yellow_side/third_puck_rotate_pose")
        self.third_puck_rotate_pose_end = rospy.get_param("yellow_side/third_puck_rotate_pose_end")

        # use find origin
        self.first_puck_landing = np.array([self.red_cell_puck[0]+self.approach_dist,
                                           self.red_cell_puck[1],
                                           3.14])

        self.second_puck_landing = np.array([self.green_cell_puck[0],
                                            self.green_cell_puck[1]-self.approach_dist,
                                            1.57])

        self.third_puck_landing = np.array([self.blue_cell_puck[0],
                                           self.blue_cell_puck[1]-self.approach_dist,
                                           1.57])

        self.chaos_push_pose = np.array([1.8, 1.2, -0.78])
        self.chaos_in_red_zone = np.array([2.65, 0.6, -1.57])
        self.start_zone = rospy.get_param("yellow_side/start_zone")

        self.blunium_collect_PREpos = rospy.get_param("yellow_side/blunium_collect_PREpos")
        self.blunium_collect_pos = rospy.get_param("yellow_side/blunium_collect_pos")

        # self.blunium_push_PREpose = rospy.get_param("yellow_zone/blunium_push_PREpose")
        # self.blunium_push_pose = rospy.get_param("yellow_zone/blunium_push_pose")
        self.accelerator_unloading_pos = rospy.get_param("yellow_side/accelerator_unloading_pos")
        self.accelerator_PREunloading_pos = rospy.get_param("yellow_side/accelerator_PREunloading_pos")

        self.goldenium_first_PREgrab_pos = rospy.get_param("yellow_side/goldenium_first_PREgrab_pos")
        self.goldenium_second_PREgrab_pos = rospy.get_param("yellow_side/goldenium_second_PREgrab_pos")
        self.goldenium_grab_pos = rospy.get_param("yellow_side/goldenium_grab_pos")

        self.scales_area = rospy.get_param("yellow_side/scales_area")
        self.scales_area = np.array(self.scales_area)
        self.scales_goldenium_PREpos = rospy.get_param("yellow_side/scales_goldenium_PREpos")
        self.scales_goldenium_pos = rospy.get_param("yellow_side/scales_goldenium_pos")


class PurpleTactics(Tactics):
    def __init__(self):
        super(PurpleTactics, self).__init__()

        self.approach_dist = rospy.get_param("approach_dist")
        self.approach_dist = np.array(self.approach_dist)
        self.approach_vec = np.array([-1*self.approach_dist, 0, 0])

        self.red_cell_puck = rospy.get_param("purple_side/red_cell_puck")
        self.green_cell_puck = rospy.get_param("purple_side/green_cell_puck")
        self.blue_cell_puck = rospy.get_param("purple_side/blue_cell_puck")
        self.third_puck_rotate_pose = rospy.get_param("purple_side/third_puck_rotate_pose")
        self.third_puck_rotate_pose_end = rospy.get_param("purple_side/third_puck_rotate_pose_end")

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

        self.chaos_push_pose = np.array([1.2, 1.2, -2.35])
        self.chaos_in_red_zone = np.array([0.35, 0.6, -1.57])
        self.start_zone = rospy.get_param("purple_side/start_zone")

        self.blunium_collect_PREpos = rospy.get_param("purple_side/blunium_collect_PREpos")
        self.blunium_collect_pos = rospy.get_param("purple_side/blunium_collect_pos")

        # self.blunium_push_PREpose = rospy.get_param("purple_zone/blunium_push_PREpose")
        # self.blunium_push_pose = rospy.get_param("purple_zone/blunium_push_pose")
        self.accelerator_unloading_pos = rospy.get_param("purple_side/accelerator_unloading_pos")
        self.accelerator_PREunloading_pos = rospy.get_param("purple_side/accelerator_PREunloading_pos")

        self.goldenium_first_PREgrab_pos = rospy.get_param("purple_side/goldenium_first_PREgrab_pos")
        self.goldenium_second_PREgrab_pos = rospy.get_param("purple_side/goldenium_second_PREgrab_pos")
        self.goldenium_grab_pos = rospy.get_param("purple_side/goldenium_grab_pos")

        self.scales_area = rospy.get_param("purple_side/scales_area")
        self.scales_area = np.array(self.scales_area)
        self.scales_goldenium_PREpos = rospy.get_param("purple_side/scales_goldenium_PREpos")
        self.scales_goldenium_pos = rospy.get_param("purple_side/scales_goldenium_pos")


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

        self.side_status = None

        self.bt = None
        self.bt_timer = None

        # self.known_chaos_pucks = bt.BTVariable(np.array([]))  # (x, y, id, r, g, b)
        # self.sorted_chaos_landings = bt.BTVariable(np.array([]))

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

        # TODO: pucks in front of starting cells are random, so while we aren't using camera
        #       will call them REDIUM  (it doesn't matter, because in this strategy we move them all to acc)

        # TODO: It will matter in case big robot faces hard collision and need to unload pucks in starting cells

        # self.is_puck_grabbed = grab_status  TODO
        self.incoming_puck_color = None
        self.collected_pucks = bt.BTVariable(np.array([]))  # FIXME change to np.array!
        self.score_master = ScoreController(self.collected_pucks)

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
            rospy.sleep(10)
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

        safely_start_move_to_blunium = bt.ParallelWithMemoryNode([
                                            bt_ros.CompleteCollectGround("manipulator_client"),
                                            bt_ros.MoveLineToPoint(self.tactics.third_puck_rotate_pose, "move_client")
                                        ], threshold=2)
                
        safely_finish_move_to_blunium = bt.SequenceWithMemoryNode([
                                            bt_ros.MoveLineToPoint(self.tactics.blunium_collect_PREpos, "move_client")
                                    ])

        start_collect_blunium = bt.ParallelWithMemoryNode([
                                    # bt_ros.MoveLineToPoint(self.tactics.blunium_push_PREpose, "move_client"),
                                    bt_ros.StartCollectBlunium("manipulator_client"),
                                    bt_ros.MoveLineToPoint(self.tactics.blunium_collect_pos, "move_client"),
                                ], threshold=2)

        finish_collect_blunium = bt.SequenceWithMemoryNode([
                                    bt_ros.FinishCollectBlunium("manipulator_client"),
                                    bt.ActionNode(lambda: self.score_master.add("BLUNIUM")),
                                ])

        move_to_acc = bt.SequenceWithMemoryNode([
                            bt_ros.MoveLineToPoint(self.tactics.blunium_collect_pos + np.array([0, 0.04, 0]), "move_client"),  # FIXME
                            bt_ros.MoveLineToPoint(self.tactics.accelerator_PREunloading_pos, "move_client"),
                            bt_ros.SetSpeedSTM([0, -0.1, 0], 1.3, "stm_client"),
                    ])

        unload_first_in_acc = bt.SequenceWithMemoryNode([
                                    bt_ros.StepUp("manipulator_client"),  # FIXME do we need to do that? NO if all 7 pucks inside
                                    bt_ros.UnloadAccelerator("manipulator_client"),
                                    bt.ActionNode(lambda: self.score_master.unload("ACC")),
                                    bt.ActionNode(lambda: self.score_master.reward("UNLOCK_GOLDENIUM_BONUS")),
                        ])

        push_blunium = bt.SequenceWithMemoryNode([
                            # bt_ros.SetManipulatorToPushBlunium("manipulator_client"),
                            #bt_ros.MoveLineToPoint(self.tactics.blunium_push_pose, "move_client"),
                            #bt_ros.MoveLineToPoint(self.tactics.blunium_push_pose + np.array([0, 0.04, 0]), "move_client"),
                            bt_ros.MoveLineToPoint(self.tactics.accelerator_unloading_pos, "move_client"),
                            bt_ros.SetSpeedSTM([0, -0.1, 0], 1.3, "stm_client"),
                            bt.ActionNode(lambda: self.score_master.add("BLUNIUM")),
                            bt.ActionNode(lambda: self.score_master.unload("ACC")),
                            bt.ActionNode(lambda: self.score_master.reward("UNLOCK_GOLDENIUM_BONUS")),
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
                                bt_ros.MoveLineToPoint(self.tactics.goldenium_first_PREgrab_pos, "move_client"),
                                bt_ros.MoveLineToPoint(self.tactics.goldenium_second_PREgrab_pos, "move_client"),
                                bt.ParallelWithMemoryNode([
                                    bt_ros.SetManipulatortoGoldenium("manipulator_client"),
                                    bt_ros.MoveLineToPoint(self.tactics.goldenium_grab_pos, "move_client"),
                                ], threshold=2),
                                bt_ros.GrabGoldeniumAndHoldUp("manipulator_client"),
                                bt.ActionNode(lambda: self.score_master.add("GOLDENIUM")),
                                bt.ActionNode(lambda: self.score_master.reward("GRAB_GOLDENIUM_BONUS")),
                                bt_ros.MoveLineToPoint(self.tactics.goldenium_grab_pos + np.array([0, 0.05, 0]), "move_client"),
                                bt_ros.MoveLineToPoint(self.tactics.goldenium_grab_pos + np.array([0, 0, 3.14]), "move_client"),
                            ])

        move_to_goldenium_prepose = bt.SequenceWithMemoryNode([
                                        bt_ros.MoveLineToPoint(self.tactics.scales_goldenium_PREpos, "move_client")
                                    ])

        unload_goldenium = bt.SequenceWithMemoryNode([  # FIXME
                                bt.ConditionNode(self.is_scales_landing_free),
                                bt.SequenceWithMemoryNode([
                                    bt_ros.MoveLineToPoint(self.tactics.scales_goldenium_pos, "move_client"),
                                    bt_ros.UnloadGoldenium("manipulator_client"),
                                    bt.ActionNode(lambda: self.score_master.unload("SCALES"))
                                ])
                            ])

        move_finish = bt.SequenceWithMemoryNode([
                        # bt_ros.MoveLineToPoint(self.tactics.first_puck_landing, "move_client"),
                        bt_ros.MoveLineToPoint(self.tactics.start_zone, "move_client"),
                    ])

        strategy = bt.SequenceWithMemoryNode([
                        red_cell_puck,
                        green_cell_puck,
                        blue_cell_puck,
                        safely_start_move_to_blunium,
                        safely_finish_move_to_blunium,
                        # move_chaos_to_red,
                        # go_to_blunium,
                        # push_blunium,
                        start_collect_blunium,
                        finish_collect_blunium,
                        move_to_acc,
                        unload_first_in_acc,
                        unload_acc,
                        collect_goldenium,
                        move_to_goldenium_prepose,
                        unload_goldenium
                        
                        # move_to_chaos,
                        # collect_chaos,
                        # move_finish
                        # move_chaos_to_red
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
            self.tactics = self.purple_tactics # FIXME
            # rospy.loginfo("Error 2")
        elif side == SideStatus.YELLOW:
            self.tactics = self.yellow_tactics
            # rospy.loginfo("Error 2")
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
            return True
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as msg:
            rospy.logwarn(str(msg))
            return False

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
