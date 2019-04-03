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


class Strategy(object):
    def __init__(self):
        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer)
        self.robot_name = rospy.get_param("robot_name")
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


class YellowStrategy(Strategy):
    def __init__(self):
        super(YellowStrategy, self).__init__()

        self.first_puck = np.array(rospy.get_param("secondary_robot/yellow_side/first_puck_zone"))
        self.second_puck = np.array(rospy.get_param("secondary_robot/yellow_side/second_puck_zone"))
        self.third_puck = np.array(rospy.get_param("secondary_robot/yellow_side/third_puck_zone"))
        self.forth_puck = np.array(rospy.get_param("secondary_robot/yellow_side/forth_puck_zone"))
        self.fifth_puck = np.array(rospy.get_param("secondary_robot/yellow_side/fifth_puck_zone"))
        self.scales_zone = np.array(rospy.get_param("secondary_robot/yellow_side/scales_zone"))
        self.start_zone = np.array(rospy.get_param("secondary_robot/yellow_side/start_zone"))
        self.sixth_puck = np.array(rospy.get_param("secondary_robot/yellow_side/sixth_puck_zone"))
        self.seventh_puck = np.array(rospy.get_param("secondary_robot/yellow_side/seventh_puck_zone"))
        self.eighth_puck = np.array(rospy.get_param("secondary_robot/yellow_side/eighth_puck_zone"))
        self.nineth_puck = np.array(rospy.get_param("secondary_robot/yellow_side/nineth_puck_zone"))
        self.start_zone = np.array(rospy.get_param("secondary_robot/yellow_side/start_zone"))
        self.redium_zone_first = np.array(rospy.get_param("secondary_robot/yellow_side/redium_zone_first"))
        self.redium_zone_second = np.array(rospy.get_param("secondary_robot/yellow_side/redium_zone_second"))
        self.redium_zone_third = np.array(rospy.get_param("secondary_robot/yellow_side/redium_zone_third"))
        self.redium_zone_forth = np.array(rospy.get_param("secondary_robot/yellow_side/redium_zone_forth"))



        first_puck = bt.FallbackWithMemoryNode([
            bt.SequenceWithMemoryNode([
                bt.ParallelWithMemoryNode([
                    bt_ros.MoveLineToPoint(self.first_puck, "move_client"),
                    # bt_ros.SetManipulatortoWall("manipulator_client")
                    bt_ros.SetToWall_ifReachedGoal((2.6, 1.6, 0), "manipulator_client")
                ], threshold=2),
                bt.FallbackWithMemoryNode([
                    bt_ros.StartTakeWallPuck("manipulator_client"),
                    bt.SequenceWithMemoryNode([
                        bt_ros.MoveLineToPoint(self.first_puck + (0, -0.05, 0), "move_client"),
                        bt_ros.MoveLineToPoint(self.first_puck, "move_client"),
                        bt_ros.StartTakeWallPuck("manipulator_client"),
                    ])
                ]),
                # bt_ros.StartTakeWallPuck("manipulator_client"),
                bt.ParallelWithMemoryNode([
                    bt.SequenceWithMemoryNode([
                        bt_ros.MoveLineToPoint(self.first_puck + (0, -0.07, 0), "move_client"),
                        bt_ros.MoveLineToPoint(self.first_puck + (0.1, -0.07, 0), "move_client")
                    ]),
                    bt_ros.CompleteTakeWallPuck("manipulator_client")
                ], threshold=2)
            ]),
            bt.SequenceWithMemoryNode([
                    bt_ros.StopPump("manipulator_client"),
                    bt_ros.MoveLineToPoint(self.first_puck + (0, -0.07, 0), "move_client"),
                    bt_ros.MoveLineToPoint(self.first_puck + (0.1, -0.07, 0), "move_client")
                ])
        ])

        second_puck = bt.FallbackWithMemoryNode([
            bt.SequenceWithMemoryNode([
                bt_ros.MoveLineToPoint(self.second_puck, "move_client"),
                bt.FallbackWithMemoryNode([
                    bt_ros.StartTakeWallPuck("manipulator_client"),
                    bt.SequenceWithMemoryNode([
                        bt_ros.MoveLineToPoint(self.second_puck + (0, -0.05, 0), "move_client"),
                        bt_ros.MoveLineToPoint(self.second_puck, "move_client"),
                        bt_ros.StartTakeWallPuck("manipulator_client"),
                    ])
                ]),
                # bt_ros.StartTakeWallPuck("manipulator_client"),
                bt.ParallelWithMemoryNode([
                    bt.SequenceWithMemoryNode([
                        bt_ros.MoveLineToPoint(self.second_puck + (0, -0.5, 0), "move_client"),
                        bt_ros.MoveLineToPoint(self.second_puck + (-0.46, -0.5, 0), "move_client")
                    ]),
                    bt_ros.CompleteTakeWallPuck("manipulator_client")
                ], threshold=2)
            ]),
            bt.SequenceWithMemoryNode([
                    bt_ros.StopPump("manipulator_client"),
                    bt_ros.MoveLineToPoint(self.second_puck + (0, -0.5, 0), "move_client"),
                    bt_ros.MoveLineToPoint(self.second_puck + (-0.46, -0.5, 0), "move_client")
                ])
        ])

        third_puck = bt.FallbackWithMemoryNode([
            bt.SequenceWithMemoryNode([
                bt_ros.MoveLineToPoint(self.third_puck, "move_client"),
                bt.FallbackWithMemoryNode([
                    bt_ros.StartTakeWallPuck("manipulator_client"),
                    bt.SequenceWithMemoryNode([
                        bt_ros.MoveLineToPoint(self.third_puck + (0, -0.05, 0), "move_client"),
                        bt_ros.MoveLineToPoint(self.third_puck, "move_client"),
                        bt_ros.StartTakeWallPuck("manipulator_client"),
                    ])
                ]),
                # bt_ros.StartTakeWallPuck("manipulator_client"),
                bt.ParallelWithMemoryNode([
                    bt.SequenceWithMemoryNode([
                        bt_ros.MoveLineToPoint(self.third_puck + (0, -0.04, 0), "move_client"),
                        bt_ros.MoveLineToPoint(self.third_puck + (-0.2, -0.04, 0), "move_client")
                    ]),
                    bt_ros.CompleteTakeWallPuck("manipulator_client")
                ], threshold=2)
            ]),
            bt.SequenceWithMemoryNode([
                    bt_ros.StopPump("manipulator_client"),
                    bt_ros.MoveLineToPoint(self.third_puck + (0, -0.04, 0), "move_client"),
                    bt_ros.MoveLineToPoint(self.third_puck + (-0.2, -0.04, 0), "move_client")
                ])
        ])

        forth_puck = bt.FallbackWithMemoryNode([
            bt.SequenceWithMemoryNode([
                bt_ros.MoveLineToPoint(self.forth_puck, "move_client"),
                bt.FallbackWithMemoryNode([
                    bt_ros.StartTakeWallPuck("manipulator_client"),
                    bt.SequenceWithMemoryNode([
                        bt_ros.MoveLineToPoint(self.forth_puck + (0, -0.05, 0), "move_client"),
                        bt_ros.MoveLineToPoint(self.forth_puck, "move_client"),
                        bt_ros.StartTakeWallPuck("manipulator_client"),
                    ])
                ]),
                # bt_ros.StartTakeWallPuck("manipulator_client"),
                bt.ParallelWithMemoryNode([
                    bt.SequenceWithMemoryNode([
                        bt_ros.MoveLineToPoint(self.forth_puck + (0, -0.04, 0), "move_client"),
                        bt_ros.MoveLineToPoint(self.forth_puck + (-0.2, -0.04, 0), "move_client")
                    ]),
                    bt_ros.CompleteTakeWallPuck("manipulator_client")
                ], threshold=2)
            ]),
            bt.SequenceWithMemoryNode([
                    bt_ros.StopPump("manipulator_client"),
                    bt_ros.MoveLineToPoint(self.forth_puck + (0, -0.04, 0), "move_client"),
                    bt_ros.MoveLineToPoint(self.forth_puck + (-0.2, -0.04, 0), "move_client")
                ])
        ])

        scales = bt.SequenceWithMemoryNode([
            bt.ParallelWithMemoryNode([
                bt_ros.MoveLineToPoint(self.scales_zone + (0, -0.14, 0), "move_client"),
                bt_ros.CompleteCollectLastWall("manipulator_client")
            ], threshold=2),
            bt_ros.MoveLineToPoint(self.scales_zone, "move_client")
        ])

        fifth_puck = bt.FallbackWithMemoryNode([
            bt.SequenceWithMemoryNode([
                bt_ros.MoveLineToPoint(self.fifth_puck, "move_client"),
                bt.FallbackWithMemoryNode([
                        bt_ros.StartTakeWallPuck("manipulator_client"),
                        bt.SequenceWithMemoryNode([
                            bt_ros.MoveLineToPoint(self.fifth_puck + (0, -0.05, 0), "move_client"),
                            bt_ros.MoveLineToPoint(self.fifth_puck, "move_client"),
                            bt_ros.StartTakeWallPuck("manipulator_client"),
                        ])
                ]),
                scales
            ]),
            bt_ros.MoveLineToPoint(self.scales_zone + (0, -0.14, 0), "move_client"),
            bt_ros.MoveLineToPoint(self.scales_zone, "move_client")
        ])
        
        release_and_back = bt.SequenceWithMemoryNode([
            bt_ros.ReleaseFivePucks("manipulator_client"),
            bt_ros.MoveLineToPoint(self.scales_zone + (0, -0.14, 0), "move_client")])

        sixth_puck = bt.SequenceWithMemoryNode([
            bt.ParallelWithMemoryNode([
                bt_ros.MoveLineToPoint(self.sixth_puck + (0, -0.04, 0), "move_client"),
                bt_ros.MoveLineToPoint(self.sixth_puck + (0, -0.04, 0), "move_client"),
                bt_ros.SetToWall_ifReachedGoal(self.sixth_puck, "manipulator_client")
                # bt_ros.SetManipulatortoWall("manipulator_client")
            ], threshold=2),
            bt_ros.MoveLineToPoint(self.sixth_puck, "move_client"),
            bt_ros.StartTakeWallPuckWithoutGrabber("manipulator_client"),
            bt.ParallelWithMemoryNode([
                bt_ros.MoveLineToPoint(self.redium_zone_first, "move_client"),
                bt_ros.SetManipulatortoGroundDelay("manipulator_client")
            ], threshold=2),
            bt_ros.ReleaseFromManipulator("manipulator_client")
        ])

        seventh_puck = bt.SequenceWithMemoryNode([
            bt.ParallelWithMemoryNode([
                bt_ros.MoveLineToPoint(self.seventh_puck + (0, -0.04, 0), "move_client"),
                bt_ros.SetToWall_ifReachedGoal(self.seventh_puck, "manipulator_client")
                # bt_ros.SetManipulatortoWall("manipulator_client")
            ], threshold=2),
            bt_ros.MoveLineToPoint(self.seventh_puck, "move_client"),
            bt_ros.StartTakeWallPuckWithoutGrabber("manipulator_client"),
            bt.ParallelWithMemoryNode([
                bt_ros.MoveLineToPoint(self.redium_zone_second, "move_client"),
                bt_ros.SetManipulatortoGroundDelay("manipulator_client")
            ], threshold=2),
            bt_ros.ReleaseFromManipulator("manipulator_client")
        ])

        eighth_puck = bt.SequenceWithMemoryNode([
            bt.ParallelWithMemoryNode([
                bt_ros.MoveLineToPoint(self.eighth_puck + (0, -0.04, 0), "move_client"),
                bt_ros.SetToWall_ifReachedGoal(self.eighth_puck, "manipulator_client")
                # bt_ros.SetManipulatortoWall("manipulator_client")
            ], threshold=2),
            bt_ros.MoveLineToPoint(self.eighth_puck, "move_client"),
            bt_ros.StartTakeWallPuckWithoutGrabber("manipulator_client"),
            bt.ParallelWithMemoryNode([
                bt_ros.MoveLineToPoint(self.redium_zone_third, "move_client"),
                bt_ros.SetManipulatortoGroundDelay("manipulator_client")
            ], threshold=2),
            bt_ros.ReleaseFromManipulator("manipulator_client")
        ])

        nineth_puck = bt.SequenceWithMemoryNode([
            bt_ros.MoveLineToPoint(self.nineth_puck + (0.1, -0.14, 0), "move_client"),
            bt_ros.SetToWall_ifReachedGoal(self.nineth_puck, "manipulator_client"),
            # bt_ros.SetManipulatortoWall("manipulator_client"),
            # bt_ros.MoveLineToPoint(self.nineth_puck + (0, -0.04, 0), "move_client"),
            bt_ros.MoveLineToPoint(self.nineth_puck, "move_client"),
            bt_ros.StartTakeWallPuckWithoutGrabber("manipulator_client"),
            bt.ParallelWithMemoryNode([
                bt_ros.MoveLineToPoint(self.redium_zone_forth, "move_client"),
                bt_ros.SetManipulatortoGroundDelay("manipulator_client")
            ], threshold=2),
            bt_ros.ReleaseFromManipulator("manipulator_client")
        ])

        # start_zone = bt.Latch(bt_ros.MoveLineToPoint(self.start_zone, "move_client"))

        self.tree = bt.SequenceWithMemoryNode([
            first_puck,
            second_puck,
            third_puck,
            forth_puck,
            fifth_puck,
            release_and_back,
            sixth_puck,
            seventh_puck,
            eighth_puck,
            nineth_puck])
            # start_zone])


class PurpleStrategy(Strategy):
    def __init__(self):
        super(PurpleStrategy, self).__init__()

        self.first_puck = np.array(rospy.get_param("secondary_robot/purple_side/first_puck_zone"))
        self.second_puck = np.array(rospy.get_param("secondary_robot/purple_side/second_puck_zone"))
        self.third_puck = np.array(rospy.get_param("secondary_robot/purple_side/third_puck_zone"))
        self.forth_puck = np.array(rospy.get_param("secondary_robot/purple_side/forth_puck_zone"))
        self.fifth_puck = np.array(rospy.get_param("secondary_robot/purple_side/fifth_puck_zone"))
        self.scales_zone = np.array(rospy.get_param("secondary_robot/purple_side/scales_zone"))
        self.sixth_puck = np.array(rospy.get_param("secondary_robot/purple_side/sixth_puck_zone"))
        self.seventh_puck = np.array(rospy.get_param("secondary_robot/purple_side/seventh_puck_zone"))
        self.eighth_puck = np.array(rospy.get_param("secondary_robot/purple_side/eighth_puck_zone"))
        self.nineth_puck = np.array(rospy.get_param("secondary_robot/purple_side/nineth_puck_zone"))
        self.start_zone = np.array(rospy.get_param("secondary_robot/purple_side/start_zone"))
        self.redium_zone_first = np.array(rospy.get_param("secondary_robot/purple_side/redium_zone_first"))
        self.redium_zone_second = np.array(rospy.get_param("secondary_robot/purple_side/redium_zone_second"))
        self.redium_zone_third = np.array(rospy.get_param("secondary_robot/purple_side/redium_zone_third"))
        self.redium_zone_forth = np.array(rospy.get_param("secondary_robot/purple_side/redium_zone_forth"))

        first_puck = bt.SequenceWithMemoryNode([
            bt.ParallelWithMemoryNode([
                bt_ros.MoveLineToPoint(self.first_puck, "move_client"),
                # bt_ros.SetManipulatortoWall("manipulator_client")
                bt_ros.SetToWall_ifReachedGoal((0.2, 1.6, 0), "manipulator_client")
            ], threshold=2),
            bt.FallbackWithMemoryNode([
                bt_ros.StartTakeWallPuck("manipulator_client"),
                bt.SequenceWithMemoryNode([
                    bt_ros.MoveLineToPoint(self.first_puck, "move_client"),
                    bt_ros.StartTakeWallPuck("manipulator_client")
                ])
            ]),
            bt_ros.StartTakeWallPuck("manipulator_client"),
            bt.ParallelWithMemoryNode([
                bt.SequenceWithMemoryNode([
                    bt_ros.MoveLineToPoint(self.first_puck + (0, -0.07, 0), "move_client"),
                    bt_ros.MoveLineToPoint(self.first_puck + (-0.1, -0.07, 0), "move_client")
                ]),
                bt_ros.CompleteTakeWallPuck("manipulator_client")
            ], threshold=2)
        ])

        second_puck = bt.SequenceWithMemoryNode([
            bt_ros.MoveLineToPoint(self.second_puck, "move_client"),
            bt_ros.StartTakeWallPuck("manipulator_client"),
            bt.ParallelWithMemoryNode([
                bt.SequenceWithMemoryNode([
                    bt_ros.MoveLineToPoint(self.second_puck + (0, -0.5, 0), "move_client"),
                    bt_ros.MoveLineToPoint(self.second_puck + (0.46, -0.5, 0), "move_client")]),
                bt_ros.CompleteTakeWallPuck("manipulator_client")
            ], threshold=2)
        ])

        third_puck = bt.SequenceWithMemoryNode([
            bt_ros.MoveLineToPoint(self.third_puck, "move_client"),
            bt_ros.StartTakeWallPuck("manipulator_client"),
            bt.ParallelWithMemoryNode([
                bt.SequenceWithMemoryNode([
                    bt_ros.MoveLineToPoint(self.third_puck + (0, -0.04, 0), "move_client"),
                    bt_ros.MoveLineToPoint(self.third_puck + (0.2, -0.04, 0), "move_client")]),
                bt_ros.CompleteTakeWallPuck("manipulator_client")
            ], threshold=2)
        ])

        forth_puck = bt.SequenceWithMemoryNode([
            bt_ros.MoveLineToPoint(self.forth_puck, "move_client"),
            bt_ros.StartTakeWallPuck("manipulator_client"),
            bt.ParallelWithMemoryNode([
                bt.SequenceWithMemoryNode([
                    bt_ros.MoveLineToPoint(self.forth_puck + (0, -0.04, 0), "move_client"),
                    bt_ros.MoveLineToPoint(self.forth_puck + (0.2, -0.04, 0), "move_client")]),
                bt_ros.CompleteTakeWallPuck("manipulator_client")
            ], threshold=2)
        ])

        fifth_puck = bt.SequenceWithMemoryNode([
            bt_ros.MoveLineToPoint(self.fifth_puck, "move_client"),
            bt_ros.StartTakeWallPuck("manipulator_client"),
        ])

        scales = bt.SequenceWithMemoryNode([
            bt.ParallelWithMemoryNode([
                bt_ros.MoveLineToPoint(self.scales_zone + (0, -0.14, 0), "move_client"),
                bt_ros.CompleteCollectLastWall("manipulator_client")
            ], threshold=2),
            bt_ros.MoveLineToPoint(self.scales_zone, "move_client")
        ])

        release_and_back = bt.SequenceWithMemoryNode([
            bt_ros.ReleaseFivePucks("manipulator_client"),
            bt_ros.MoveLineToPoint(self.scales_zone + (0, -0.14, 0), "move_client")])

        sixth_puck = bt.SequenceWithMemoryNode([
            bt.ParallelWithMemoryNode([
                bt_ros.MoveLineToPoint(self.sixth_puck + (0, -0.04, 0), "move_client"),
                bt_ros.MoveLineToPoint(self.sixth_puck + (0, -0.04, 0), "move_client"),
                bt_ros.SetToWall_ifReachedGoal(self.sixth_puck, "manipulator_client")
                # bt_ros.SetManipulatortoWall("manipulator_client")
            ], threshold=2),
            bt_ros.MoveLineToPoint(self.sixth_puck, "move_client"),
            bt_ros.StartTakeWallPuckWithoutGrabber("manipulator_client"),
            bt.ParallelWithMemoryNode([
                bt_ros.MoveLineToPoint(self.redium_zone_first, "move_client"),
                bt_ros.SetManipulatortoGroundDelay("manipulator_client")
            ], threshold=2),
            bt_ros.ReleaseFromManipulator("manipulator_client")
        ])

        seventh_puck = bt.SequenceWithMemoryNode([
            bt.ParallelWithMemoryNode([
                bt_ros.MoveLineToPoint(self.seventh_puck + (0, -0.04, 0), "move_client"),
                bt_ros.SetToWall_ifReachedGoal(self.seventh_puck, "manipulator_client")
                # bt_ros.SetManipulatortoWall("manipulator_client")
            ], threshold=2),
            bt_ros.MoveLineToPoint(self.seventh_puck, "move_client"),
            bt_ros.StartTakeWallPuckWithoutGrabber("manipulator_client"),
            bt.ParallelWithMemoryNode([
                bt_ros.MoveLineToPoint(self.redium_zone_second, "move_client"),
                bt_ros.SetManipulatortoGroundDelay("manipulator_client")
            ], threshold=2),
            bt_ros.ReleaseFromManipulator("manipulator_client")
        ])

        eighth_puck = bt.SequenceWithMemoryNode([
            bt.ParallelWithMemoryNode([
                bt_ros.MoveLineToPoint(self.eighth_puck + (0, -0.04, 0), "move_client"),
                bt_ros.SetToWall_ifReachedGoal(self.eighth_puck, "manipulator_client")
                # bt_ros.SetManipulatortoWall("manipulator_client")
            ], threshold=2),
            bt_ros.MoveLineToPoint(self.eighth_puck, "move_client"),
            bt_ros.StartTakeWallPuckWithoutGrabber("manipulator_client"),
            bt.ParallelWithMemoryNode([
                bt_ros.MoveLineToPoint(self.redium_zone_third, "move_client"),
                bt_ros.SetManipulatortoGroundDelay("manipulator_client")
            ], threshold=2),
            bt_ros.ReleaseFromManipulator("manipulator_client")
        ])

        nineth_puck = bt.SequenceWithMemoryNode([
            bt_ros.MoveLineToPoint(self.nineth_puck + (-0.1, -0.14, 0), "move_client"),
            bt_ros.SetToWall_ifReachedGoal(self.nineth_puck, "manipulator_client"),
            # bt_ros.SetManipulatortoWall("manipulator_client"),
            # bt_ros.MoveLineToPoint(self.nineth_puck + (0, -0.04, 0), "move_client"),
            bt_ros.MoveLineToPoint(self.nineth_puck, "move_client"),
            bt_ros.StartTakeWallPuckWithoutGrabber("manipulator_client"),
            bt.ParallelWithMemoryNode([
                bt_ros.MoveLineToPoint(self.redium_zone_forth, "move_client"),
                bt_ros.SetManipulatortoGroundDelay("manipulator_client")
            ], threshold=2),
            bt_ros.ReleaseFromManipulator("manipulator_client")
        ])

        # start_zone = bt_ros.MoveLineToPoint(self.start_zone, "move_client")

        self.tree = bt.SequenceWithMemoryNode([
            first_puck,

            second_puck,
            third_puck,
            forth_puck,
            fifth_puck,
            scales,
            release_and_back,
            sixth_puck,
            seventh_puck,
            eighth_puck,
            nineth_puck])
            # start_zone])


class SecondaryRobotBT(object):
    def __init__(self, side_status=SideStatus.PURPLE):
        self.move_publisher = rospy.Publisher("navigation/command", String, queue_size=100)
        self.move_client = bt_ros.ActionClient(self.move_publisher)
        rospy.Subscriber("navigation/response", String, self.move_client.response_callback)

        self.manipulator_publisher = rospy.Publisher("manipulator/command", String, queue_size=100)
        self.manipulator_client = bt_ros.ActionClient(self.manipulator_publisher)
        rospy.Subscriber("manipulator/response", String, self.manipulator_client.response_callback)

        self.purple_strategy = PurpleStrategy()
        self.yellow_strategy = YellowStrategy()

        self.side_status = side_status
        if self.side_status == SideStatus.PURPLE:
            self.strategy = self.purple_strategy
        elif self.side_status == SideStatus.YELLOW:
            self.strategy = self.yellow_strategy
        else:
            self.strategy = None

        self.bt = None
        self.bt_timer = None

    def start(self):
        self.bt = bt.Root(self.strategy.tree,
                          action_clients={"move_client": self.move_client, "manipulator_client": self.manipulator_client})

        self.bt_timer = rospy.Timer(rospy.Duration(0.1), self.timer_callback)

    def change_side(self, side):
        if side == SideStatus.PURPLE:
            self.strategy = self.purple_strategy
        elif side == SideStatus.YELLOW:
            self.strategy = self.yellow_strategy
        else:
            self.strategy = None
        self.side_status = side

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
