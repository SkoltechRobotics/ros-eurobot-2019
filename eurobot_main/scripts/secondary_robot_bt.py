#!/usr/bin/env python
import numpy as np

import rospy
import tf2_ros
from tf.transformations import euler_from_quaternion

import bt_ros
from std_msgs.msg import String

import behavior_tree as bt
from bt_controller import SideStatus, BTController
from core_functions import *


class Tactics(object):
    def __init__(self):
        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer)
        self.robot_coordinates = None

    def update_coordinates(self):
        try:
            trans = self.tfBuffer.lookup_transform('map', self.robot_name, rospy.Time(0))
            q = [trans.transform.rotation.x,
                 trans.transform.rotation.y,
                 trans.transform.rotation.z,
                 trans.transform.rotation.w]
            angle = euler_from_quaternion(q)[2] % (2 * np.pi)
            self.robot_coordinates = np.array([trans.transform.translation.x,
                                               trans.transform.translation.y,
                                               angle])
            rospy.loginfo("Robot coords:\t" + str(self.robot_coordinates))
            return True
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as msg:
            rospy.logwarn(str(msg))
            rospy.logwarn("SMT WROND QQ")
            return False

    def is_coordinates_reached(self, coordinates, threshold=0.01):
        # FIXME:: self.update_coordinates() replace???!!!
        self.update_coordinates()
        distance, _ = calculate_distance(self.robot_coordinates, coordinates)
        norm_distance = np.linalg.norm(distance)
        if norm_distance < threshold:
            return bt.Status.SUCCESS
        else:
            return bt.Status.FAILED


class YellowTactics(Tactics):
    def __init__(self):
        super(YellowTactics, self).__init__()

        self.first_puck = np.array(rospy.get_param("secondary_robot/yellow_side/first_puck_zone"))
        self.second_puck = np.array(rospy.get_param("secondary_robot/yellow_side/second_puck_zone"))
        self.third_puck = np.array(rospy.get_param("secondary_robot/yellow_side/third_puck_zone"))
        self.forth_puck = np.array(rospy.get_param("secondary_robot/yellow_side/forth_puck_zone"))
        self.fifth_puck = np.array(rospy.get_param("secondary_robot/yellow_side/fifth_puck_zone"))
        self.scales_zone = np.array(rospy.get_param("secondary_robot/yellow_side/scales_zone"))
        self.start_zone = np.array(rospy.get_param("secondary_robot/yellow_side/start_zone"))

        default_state = bt.Latch(bt_ros.SetToDefaultState("manipulator_client"))

        first_puck = bt.SequenceNode([
            bt.ParallelNode([
                bt.Latch(bt_ros.MoveLineToPoint(self.first_puck, "move_client")),
                bt.FallbackNode([
                    bt.ConditionNode(super(YellowTactics, self).is_coordinates_reached(2.8, 1.2, 0)),
                    bt.Latch(bt_ros.SetManipulatortoWall("manipulator_client"))])
            ], threshold=2),

            bt.Latch(bt_ros.StartTakeWallPuck("manipulator_client")),
            bt.ParallelNode([
                bt.SequenceWithMemoryNode([
                    bt_ros.MoveLineToPoint(self.first_puck + (0, -0.07, 0), "move_client"),
                    bt_ros.MoveLineToPoint(self.first_puck + (0.1, -0.07, 0),"move_client")]),
                bt.Latch(bt_ros.CompleteTakeWallPuck("manipulator_client"))
            ], threshold=2)
        ])

        second_puck = bt.SequenceNode([
            bt.Latch(bt_ros.MoveLineToPoint(self.second_puck, "move_client")),
            bt.Latch(bt_ros.StartTakeWallPuck("manipulator_client")),
            bt.ParallelNode([
                bt.SequenceWithMemoryNode([
                    bt_ros.MoveLineToPoint(self.second_puck + (0, -0.5, 0), "move_client"),
                    bt_ros.MoveLineToPoint(self.second_puck + (-0.49, -0.5, 0), "move_client")]),
                bt.Latch(bt_ros.CompleteTakeWallPuck("manipulator_client"))
            ], threshold=2)
        ])

        third_puck = bt.SequenceNode([
            bt.Latch(bt_ros.MoveLineToPoint(self.third_puck, "move_client")),
            bt.Latch(bt_ros.StartTakeWallPuck("manipulator_client")),
            bt.ParallelNode([
                bt.SequenceWithMemoryNode([
                    bt_ros.MoveLineToPoint(self.third_puck + (0, -0.04, 0), "move_client"),
                    bt_ros.MoveLineToPoint(self.third_puck + (-0.2, -0.04, 0), "move_client")]),
                bt.Latch(bt_ros.CompleteTakeWallPuck("manipulator_client"))
            ], threshold=2)
        ])

        forth_puck = bt.SequenceNode([
            bt.Latch(bt_ros.MoveLineToPoint(self.forth_puck, "move_client")),
            bt.Latch(bt_ros.StartTakeWallPuck("manipulator_client")),
            bt.ParallelNode([
                bt.SequenceWithMemoryNode([
                    bt_ros.MoveLineToPoint(self.forth_puck + (0, -0.04, 0), "move_client"),
                    bt.Latch(bt_ros.MoveLineToPoint(self.forth_puck + (-0.2, -0.04, 0), "move_client"))]),
                bt.Latch(bt_ros.CompleteTakeWallPuck("manipulator_client"))
            ], threshold=2)
        ])

        fifth_puck = bt.SequenceNode([
            bt.Latch(bt_ros.MoveLineToPoint(self.fifth_puck, "move_client")),
            bt.Latch(bt_ros.StartTakeWallPuck("manipulator_client")),
            bt.ParallelNode([
                bt.Latch(bt_ros.MoveLineToPoint(self.scales_zone, "move_client")),
                bt.Latch(bt_ros.CompleteCollectLastPuck("manipulator_client"))
            ], threshold=2)
        ])

        scales = bt.SequenceWithMemoryNode([
            bt_ros.MoveLineToPoint(self.scales_zone + (0, 0.14, 0), "move_client"),
            bt_ros.MoveLineToPoint(self.scales_zone + (0, 0.14, 0), "move_client")])

        start_zone = bt.Latch(bt_ros.MoveLineToPoint(self.start_zone, "move_client"))

        self.leaf = bt.SequenceNode([default_state,
                                     first_puck,
                                     second_puck,
                                     third_puck,
                                     forth_puck,
                                     fifth_puck,
                                     scales,
                                     start_zone])


class PurpleTactics(Tactics):
    def __init__(self):
        super(PurpleTactics, self).__init__()

        self.first_puck = np.array(rospy.get_param("secondary_robot/purple_side/first_puck_zone"))
        self.second_puck = np.array(rospy.get_param("secondary_robot/purple_side/second_puck_zone"))
        self.third_puck = np.array(rospy.get_param("secondary_robot/purple_side/third_puck_zone"))
        self.forth_puck = np.array(rospy.get_param("secondary_robot/purple_side/forth_puck_zone"))
        self.fifth_puck = np.array(rospy.get_param("secondary_robot/purple_side/fifth_puck_zone"))
        self.scales_zone = np.array(rospy.get_param("secondary_robot/purple_side/scales_zone"))
        self.start_zone = np.array(rospy.get_param("secondary_robot/purple_side/start_zone"))

        default_state = bt.Latch(bt_ros.SetToDefaultState("manipulator_client"))

        first_puck = bt.SequenceNode([
            bt.ParallelNode([
                bt.Latch(bt_ros.MoveLineToPoint(self.first_puck, "move_client")),
                bt.FallbackNode([
                    bt.ConditionNode(super(PurpleTactics, self).is_coordinates_reached(0.4, 1.2, 0)),
                    bt.Latch(bt_ros.SetManipulatortoWall("manipulator_client"))
                ])
            ], threshold=2),

            bt.Latch(bt_ros.StartTakeWallPuck("manipulator_client")),
            bt.ParallelNode([
                bt.SequenceWithMemoryNode([
                    bt_ros.MoveLineToPoint(self.first_puck + (0, -0.07, 0), "move_client"),
                    bt_ros.MoveLineToPoint(self.first_puck + (-0.1, -0.07, 0), "move_client")
                ]),
                bt.Latch(bt_ros.CompleteTakeWallPuck("manipulator_client"))
            ], threshold=2)
        ])

        second_puck = bt.SequenceNode([
            bt.Latch(bt_ros.MoveLineToPoint(self.second_puck, "move_client")),
            bt.Latch(bt_ros.StartTakeWallPuck("manipulator_client")),
            bt.ParallelNode([
                bt.SequenceWithMemoryNode([
                    bt_ros.MoveLineToPoint(self.second_puck + (0, -0.5, 0), "move_client"),
                    bt_ros.MoveLineToPoint(self.second_puck + (+0.49, -0.5, 0), "move_client")]),
                bt.Latch(bt_ros.CompleteTakeWallPuck("manipulator_client"))
            ], threshold=2)
        ])

        third_puck = bt.SequenceNode([
            bt.Latch(bt_ros.MoveLineToPoint(self.third_puck, "move_client")),
            bt.Latch(bt_ros.StartTakeWallPuck("manipulator_client")),
            bt.ParallelNode([
                bt.SequenceWithMemoryNode([
                    bt_ros.MoveLineToPoint(self.third_puck + (0, -0.04, 0), "move_client"),
                    bt_ros.MoveLineToPoint(self.third_puck + (0.2, -0.04, 0), "move_client")]),
                bt.Latch(bt_ros.CompleteTakeWallPuck("manipulator_client"))
            ], threshold=2)
        ])

        forth_puck = bt.SequenceNode([
            bt.Latch(bt_ros.MoveLineToPoint(self.forth_puck, "move_client")),
            bt.Latch(bt_ros.StartTakeWallPuck("manipulator_client")),
            bt.ParallelNode([
                bt.SequenceWithMemoryNode([
                    bt_ros.MoveLineToPoint(self.forth_puck + (0, -0.04, 0), "move_client"),
                    bt.Latch(bt_ros.MoveLineToPoint(self.forth_puck + (0.2, -0.04, 0), "move_client"))]),
                bt.Latch(bt_ros.CompleteTakeWallPuck("manipulator_client"))
            ], threshold=2)
        ])

        fifth_puck = bt.SequenceNode([
            bt.Latch(bt_ros.MoveLineToPoint(self.fifth_puck, "move_client")),
            bt.Latch(bt_ros.StartTakeWallPuck("manipulator_client")),
            bt.ParallelNode([
                bt.Latch(bt_ros.MoveLineToPoint(self.scales_zone, "move_client")),
                bt.Latch(bt_ros.CompleteCollectLastPuck("manipulator_client"))
            ], threshold=2)
        ])

        scales = bt.SequenceWithMemoryNode([
            bt_ros.MoveLineToPoint(self.scales_zone + (0, 0.14, 0), "move_client"),
            bt_ros.MoveLineToPoint(self.scales_zone + (0, 0.14, 0), "move_client")
        ])

        start_zone = bt.Latch(bt_ros.MoveLineToPoint(self.start_zone, "move_client"))

        self.leaf = bt.SequenceNode([default_state,
                                     first_puck,
                                     second_puck,
                                     third_puck,
                                     forth_puck,
                                     fifth_puck,
                                     scales,
                                     start_zone])


class SecondaryRobotBT(object):
    def __init__(self, side_status=SideStatus.PURPLE):
        self.robot_name = rospy.get_param("robot_name")

        self.move_publisher = rospy.Publisher("navigation/command", String, queue_size=100)
        self.move_client = bt_ros.ActionClient(self.move_publisher)
        rospy.Subscriber("navigation/response", String, self.move_client.response_callback)

        self.manipulator_publisher = rospy.Publisher("manipulator/command", String, queue_size=100)
        self.manipulator_client = bt_ros.ActionClient(self.manipulator_publisher)
        rospy.Subscriber("manipulator/response", String, self.manipulator_client.response_callback)

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

    def start(self):
        self.bt = bt.Root(self.tactics.leaf,
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
        rospy.init_node("secondary_robot_BT")
        secondary_robot_bt = SecondaryRobotBT()
        bt_controller = BTController(secondary_robot_bt)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
