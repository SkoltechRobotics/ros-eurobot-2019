#!/usr/bin/env python
import rospy
import behavior_tree as bt
import bt_ros
from std_msgs.msg import String
from bt_controller import SideStatus, BTController
from core_functions import *

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

# add in MainRobotBT when collect_chaos ready
# self.bt = bt.Root(CollectChaosPucks("move_client", "manipulator_client"),
#                   action_clients={"move_client": self.move_client, "manipulator_client": self.manipulator_client})


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

        # can be reached using rospy.get_param("purple_zone/red_cell_puck"
        self.red_cell_puck = rospy.get_param("yellow_zone/red_cell_puck")
        self.green_cell_puck = rospy.get_param("yellow_zone/green_cell_puck")
        self.blue_cell_puck = rospy.get_param("yellow_zone/blue_cell_puck")

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

        self.start_zone = rospy.get_param("yellow_zone/start_zone")

        self.blunium_collect_PREpos = rospy.get_param("yellow_zone/blunium_collect_PREpos")
        self.blunium_collect_pos = rospy.get_param("yellow_zone/blunium_collect_pos")

        self.accelerator_PREunloading_pos = rospy.get_param("yellow_zone/accelerator_PREunloading_pos")
        self.accelerator_unloading_pos = rospy.get_param("yellow_zone/accelerator_unloading_pos")
        self.accelerator_unloading_pos_far = rospy.get_param("yellow_zone/accelerator_unloading_pos_far")

        self.goldenium_PREgrab_pos = rospy.get_param("yellow_zone/goldenium_PREgrab_pos")
        self.goldenium_grab_pos = rospy.get_param("yellow_zone/goldenium_grab_pos")

        self.scales_goldenium_PREpos = rospy.get_param("yellow_zone/scales_goldenium_PREpos")
        self.scales_goldenium_pos = rospy.get_param("yellow_zone/scales_goldenium_pos")


class PurpleTactics(Tactics):
    def __init__(self):
        super(PurpleTactics, self).__init__()

        self.approach_dist = rospy.get_param("approach_dist")  # 0.127 meters, distance from robot to puck where robot will try to grab it
        self.approach_dist = np.array(self.approach_dist)
        self.approach_vec = np.array([-1*self.approach_dist, 0, 0])  # 0.11

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

        self.blunium_collect_PREpos = rospy.get_param("purple_zone/blunium_collect_PREpos")
        self.blunium_collect_pos = rospy.get_param("purple_zone/blunium_collect_pos")

        self.accelerator_PREunloading_pos = rospy.get_param("purple_zone/accelerator_PREunloading_pos")
        self.accelerator_unloading_pos = rospy.get_param("purple_zone/accelerator_unloading_pos")
        self.accelerator_unloading_pos_far = rospy.get_param("purple_zone/accelerator_unloading_pos_far")

        self.goldenium_PREgrab_pos = rospy.get_param("purple_zone/goldenium_PREgrab_pos")
        self.goldenium_grab_pos = rospy.get_param("purple_zone/goldenium_grab_pos")

        self.scales_goldenium_PREpos = rospy.get_param("purple_zone/scales_goldenium_PREpos")
        self.scales_goldenium_pos = rospy.get_param("purple_zone/scales_goldenium_pos")


class MainRobotBT(object):
    # noinspection PyTypeChecker
    def __init__(self, side_status=SideStatus.PURPLE):
        self.robot_name = rospy.get_param("robot_name")

        self.move_publisher = rospy.Publisher("navigation/command", String, queue_size=100)
        self.manipulator_publisher = rospy.Publisher("manipulator/command", String, queue_size=100)

        self.move_client = bt_ros.ActionClient(self.move_publisher)
        self.manipulator_client = bt_ros.ActionClient(self.manipulator_publisher)

        self.purple_tactics = PurpleTactics()
        self.yellow_tactics = YellowTactics()

        self.side_status = side_status

        if self.side_status == SideStatus.PURPLE:
            self.tactics = self.purple_tactics
        elif self.side_status == SideStatus.YELLOW:
            self.tactics = self.yellow_tactics
        else:
            self.tactics = None

        self.bt = None
        self.bt_timer = None

        rospy.Subscriber("navigation/response", String, self.move_client.response_callback)
        rospy.Subscriber("manipulator/response", String, self.manipulator_client.response_callback)

    def start(self):

        # FIXME change structure to subtrees!
        
        self.bt = bt.Root(
            bt.SequenceWithMemoryNode([
                bt_ros.SetToDefaultState("manipulator_client"),

                bt_ros.MoveLineToPoint(self.tactics.first_puck_landing, "move_client"),
                bt_ros.StartCollectGround("manipulator_client"),

                bt.ParallelWithMemoryNode([
                    bt_ros.CompleteCollectGround("manipulator_client"),
                    bt_ros.MoveLineToPoint(self.tactics.second_puck_landing, "move_client"),
                ], threshold=2),
                bt_ros.StartCollectGround("manipulator_client"),

                bt.ParallelWithMemoryNode([
                    bt_ros.CompleteCollectGround("manipulator_client"),
                    bt_ros.MoveLineToPoint(self.tactics.third_puck_landing, "move_client"),
                ], threshold=2),
                bt_ros.StartCollectGround("manipulator_client"),

                # move to blunium and collect it
                bt.ParallelWithMemoryNode([
                    bt_ros.CompleteCollectGround("manipulator_client"),
                    bt_ros.MoveLineToPoint(self.tactics.blunium_collect_PREpos, "move_client"),
                ], threshold=2),
                bt_ros.StartCollectBlunium("manipulator_client"),
                bt_ros.MoveLineToPoint(self.tactics.blunium_collect_pos, "move_client"),
                bt_ros.CompleteCollectGround("manipulator_client"),

                bt_ros.PumpUp("manipulator_client"),
                bt_ros.MoveLineToPoint(self.tactics.accelerator_PREunloading_pos, "move_client"),
                # FIXME Sasha have to fix height of unloading mechanism

                # make step up
                bt_ros.StepUp("manipulator_client"),
                # unload first in acc
                bt_ros.MoveLineToPoint(self.tactics.accelerator_unloading_pos, "move_client"),
                bt_ros.UnloadAccelerator("manipulator_client"),
                bt_ros.MoveLineToPoint(self.tactics.accelerator_unloading_pos_far, "move_client"),

                # unload second in acc
                bt_ros.MoveLineToPoint(self.tactics.accelerator_unloading_pos, "move_client"),
                bt_ros.UnloadAccelerator("manipulator_client"),
                bt_ros.MoveLineToPoint(self.tactics.accelerator_unloading_pos_far, "move_client"),

                # unload third in acc
                bt_ros.MoveLineToPoint(self.tactics.accelerator_unloading_pos, "move_client"),
                bt_ros.UnloadAccelerator("manipulator_client"),
                bt_ros.MoveLineToPoint(self.tactics.accelerator_unloading_pos_far, "move_client"),

                # unload forth in acc
                bt_ros.MoveLineToPoint(self.tactics.accelerator_unloading_pos, "move_client"),
                bt_ros.UnloadAccelerator("manipulator_client"),
                bt_ros.MoveLineToPoint(self.tactics.accelerator_unloading_pos_far, "move_client"),

                # to make sure that the LAST puck is unloaded
                bt_ros.MoveLineToPoint(self.tactics.accelerator_unloading_pos, "move_client"),

                # collect Goldenium
                bt_ros.MoveLineToPoint(self.tactics.goldenium_PREgrab_pos, "move_client"),
                bt_ros.SetManipulatortoGoldenium("manipulator_client"),
                bt_ros.MoveLineToPoint(self.tactics.goldenium_grab_pos, "move_client"),
                bt_ros.GrabGoldeniumAndHoldUp("manipulator_client"),

                # move to scales and unload
                bt_ros.MoveLineToPoint(self.tactics.scales_goldenium_PREpos, "move_client"), # FIXME
                bt_ros.MoveLineToPoint(self.tactics.scales_goldenium_pos, "move_client"), # FIXME
                bt_ros.UnloadGoldenium("manipulator_client"),

                bt_ros.MoveLineToPoint(self.tactics.first_puck_landing, "move_client"),
                bt_ros.MoveLineToPoint(self.tactics.start_zone, "move_client"),
            ]),
            action_clients={"move_client": self.move_client, "manipulator_client": self.manipulator_client})

        self.bt_timer = rospy.Timer(rospy.Duration(0.1), self.timer_callback)

    def change_side(self, side):
        if side == SideStatus.PURPLE:
            self.tactics = self.purple_tactics
        elif side == SideStatus.YELLOW:
            self.tactics = self.yellow_tactics
        else:
            self.tactics = None

    def timer_callback(self, event):

        status = self.bt.tick()
        if status != bt.Status.RUNNING:
            self.bt_timer.shutdown()
        print("============== BT LOG ================")
        self.bt.log(0)


if __name__ == '__main__':
    try:
        rospy.init_node("main_robot_BT")
        main_robot_bt = MainRobotBT()
        bt_controller = BTController(main_robot_bt)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
