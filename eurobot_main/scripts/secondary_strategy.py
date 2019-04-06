#!/usr/bin/env python
import numpy as np

import rospy
import tf2_ros
from tf.transformations import euler_from_quaternion

import behavior_tree as bt
import bt_ros
from bt_controller import SideStatus

class Strategy(object):
    def __init__(self):
        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer)
        self.robot_name = rospy.get_param("robot_name")
        self.robot_coordinates = None
        self.pucks_inside = []

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


class VovanStrategy(Strategy):
    def __init__(self, side):
        super(VovanStrategy, self).__init__()

        if side == SideStatus.PURPLE:
            param = "purple_side"
            side_sign = -1
        elif side == SideStatus.YELLOW:
            param = "yellow_side"
            side_sign = 1

        self.first_puck = np.array(rospy.get_param("secondary_robot/" + param + "/first_puck_zone"))
        self.second_puck = np.array(rospy.get_param("secondary_robot/" + param + "/second_puck_zone"))
        self.third_puck = np.array(rospy.get_param("secondary_robot/" + param + "/third_puck_zone"))
        self.forth_puck = np.array(rospy.get_param("secondary_robot/" + param + "/forth_puck_zone"))
        self.fifth_puck = np.array(rospy.get_param("secondary_robot/" + param + "/fifth_puck_zone"))
        self.scales_zone = np.array(rospy.get_param("secondary_robot/" + param + "/scales_zone"))
        self.start_zone = np.array(rospy.get_param("secondary_robot/" + param + "/start_zone"))
        self.sixth_puck = np.array(rospy.get_param("secondary_robot/" + param + "/sixth_puck_zone"))
        self.seventh_puck = np.array(rospy.get_param("secondary_robot/" + param + "/seventh_puck_zone"))
        self.eighth_puck = np.array(rospy.get_param("secondary_robot/" + param + "/eighth_puck_zone"))
        self.nineth_puck = np.array(rospy.get_param("secondary_robot/" + param + "/nineth_puck_zone"))
        self.start_zone = np.array(rospy.get_param("secondary_robot/" + param + "/start_zone"))
        self.redium_zone_first = np.array(rospy.get_param("secondary_robot/" + param + "/redium_zone_first"))
        self.redium_zone_second = np.array(rospy.get_param("secondary_robot/" + param + "/redium_zone_second"))
        self.redium_zone_third = np.array(rospy.get_param("secondary_robot/" + param + "/redium_zone_third"))
        self.redium_zone_forth = np.array(rospy.get_param("secondary_robot/" + param + "/redium_zone_forth"))


        first_puck = bt.FallbackWithMemoryNode([
            bt.SequenceWithMemoryNode([
                bt.ParallelWithMemoryNode([
                    bt_ros.MoveLineToPoint(self.first_puck + (0, -0.05, 0), "move_client"),
                    # bt_ros.SetManipulatortoWall("manipulator_client")
                    bt_ros.SetToWall_ifReachedGoal(self.first_puck + (0, -0.15, 0), "manipulator_client")
                ], threshold=2),
                bt_ros.StartPump("manipulator_client"),
                bt_ros.MoveLineToPoint(self.first_puck, "move_client"),
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
                        bt_ros.MoveLineToPoint(self.first_puck + (side_sign*0.1, -0.07, 0), "move_client")
                    ]),
                    bt_ros.CompleteTakeWallPuck(self.pucks_inside, "grenium", "manipulator_client")
                ], threshold=2)
            ]),
            bt.SequenceWithMemoryNode([
                    bt_ros.StopPump("manipulator_client"),
                    bt_ros.MoveLineToPoint(self.first_puck + (0, -0.07, 0), "move_client"),
                    bt_ros.MoveLineToPoint(self.first_puck + (side_sign*0.1, -0.07, 0), "move_client")
                ])
        ])

        second_puck = bt.FallbackWithMemoryNode([
            bt.SequenceWithMemoryNode([
                bt_ros.StartPump("manipulator_client"),
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
                        bt_ros.MoveLineToPoint(self.second_puck + (side_sign*-0.46, -0.5, 0), "move_client")
                    ]),
                    bt_ros.CompleteTakeWallPuck(self.pucks_inside, "bluemium", "manipulator_client")
                ], threshold=2),
            ]),
            bt.SequenceWithMemoryNode([
                    bt_ros.StopPump("manipulator_client"),
                    bt_ros.MoveLineToPoint(self.second_puck + (0, -0.5, 0), "move_client"),
                    bt_ros.MoveLineToPoint(self.second_puck + (side_sign*-0.46, -0.5, 0), "move_client")
                ])
        ])

        third_puck = bt.FallbackWithMemoryNode([
            bt.SequenceWithMemoryNode([
                bt_ros.StartPump("manipulator_client"),
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
                        bt_ros.MoveLineToPoint(self.third_puck + (side_sign*-0.2, -0.04, 0), "move_client")
                    ]),
                    bt_ros.CompleteTakeWallPuck(self.pucks_inside, "grenium", "manipulator_client")
                ], threshold=2)
            ]),
            bt.SequenceWithMemoryNode([
                    bt_ros.StopPump("manipulator_client"),
                    bt_ros.MoveLineToPoint(self.third_puck + (0, -0.04, 0), "move_client"),
                    bt_ros.MoveLineToPoint(self.third_puck + (side_sign*-0.2, -0.04, 0), "move_client")
                ])
        ])

        forth_puck = bt.FallbackWithMemoryNode([
            bt.SequenceWithMemoryNode([
                bt_ros.StartPump("manipulator_client"),
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
                        bt_ros.MoveLineToPoint(self.forth_puck + (side_sign*-0.2, -0.04, 0), "move_client")
                    ]),
                    bt_ros.CompleteTakeWallPuck(self.pucks_inside, "bluemium", "manipulator_client")
                ], threshold=2)
            ]),
            bt.SequenceWithMemoryNode([
                    bt_ros.StopPump("manipulator_client"),
                    bt_ros.MoveLineToPoint(self.forth_puck + (0, -0.04, 0), "move_client"),
                    bt_ros.MoveLineToPoint(self.forth_puck + (side_sign*-0.2, -0.04, 0), "move_client")
                ])
        ])

        scales = bt.SequenceWithMemoryNode([
            bt_ros.MoveLineToPoint(self.fifth_puck + (0, -0.04, 0), "move_client"),
            bt.ParallelWithMemoryNode([
                bt_ros.MoveLineToPoint(self.scales_zone + (0, -0.14, 0), "move_client"),
                bt_ros.CompleteCollectLastWall(self.pucks_inside, "grenium", "manipulator_client")
            ], threshold=2),
            bt_ros.MoveLineToPoint(self.scales_zone, "move_client")
        ])

        fifth_puck = bt.FallbackWithMemoryNode([
            bt.SequenceWithMemoryNode([
                bt_ros.StartPump("manipulator_client"),
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

        release_and_back = bt_ros.ReleaseAndBack(self.pucks_inside, self.scales_zone, "manipulator_client")

        sixth_puck = bt.FallbackWithMemoryNode([
            bt.SequenceWithMemoryNode([
                bt.ParallelWithMemoryNode([
                    bt_ros.MoveLineToPoint(self.sixth_puck + (0, -0.04, 0), "move_client"),
                    bt_ros.SetToWall_ifReachedGoal(self.sixth_puck, "manipulator_client")
                    # bt_ros.SetManipulatortoWall("manipulator_client")
                ], threshold=2),
                bt_ros.StartPump("manipulator_client"),
                bt_ros.MoveLineToPoint(self.sixth_puck, "move_client"),
                bt.FallbackWithMemoryNode([
                        bt_ros.StartTakeWallPuckWithoutGrabber("manipulator_client"),
                        bt.SequenceWithMemoryNode([
                            bt_ros.MoveLineToPoint(self.sixth_puck + (0, -0.04, 0), "move_client"),
                            bt_ros.MoveLineToPoint(self.sixth_puck, "move_client"),
                            bt_ros.StartTakeWallPuckWithoutGrabber("manipulator_client"),
                        ])
                ]),
                bt.ParallelWithMemoryNode([
                    bt_ros.MoveLineToPoint(self.redium_zone_first, "move_client"),
                    bt_ros.SetManipulatortoGroundDelay("manipulator_client")
                ], threshold=2),
                bt_ros.ReleaseFromManipulator("manipulator_client"),
                bt.ParallelWithMemoryNode([
                    bt_ros.MoveLineToPoint(self.seventh_puck + (0, -0.04, 0), "move_client"),
                    bt_ros.SetToWall_ifReachedGoal(self.seventh_puck, "manipulator_client")
                ], threshold=2)
            ]),

            bt.SequenceWithMemoryNode([
                    bt_ros.StopPump("manipulator_client"),
                    bt_ros.MoveLineToPoint(self.sixth_puck + (0, -0.04, 0), "move_client"),
                    bt.ParallelWithMemoryNode([
                        bt_ros.MoveLineToPoint(self.seventh_puck + (0, -0.04, 0), "move_client"),
                        bt_ros.SetToWall_ifReachedGoal(self.seventh_puck, "manipulator_client")
                    ], threshold=2)
                ])
        ])

        seventh_puck = bt.FallbackWithMemoryNode([
            bt.SequenceWithMemoryNode([
                bt_ros.StartPump("manipulator_client"),
                bt_ros.MoveLineToPoint(self.seventh_puck, "move_client"),
                bt.FallbackWithMemoryNode([
                        bt_ros.StartTakeWallPuckWithoutGrabber("manipulator_client"),
                        bt.SequenceWithMemoryNode([
                            bt_ros.MoveLineToPoint(self.seventh_puck + (0, -0.04, 0), "move_client"),
                            bt_ros.MoveLineToPoint(self.seventh_puck, "move_client"),
                            bt_ros.StartTakeWallPuckWithoutGrabber("manipulator_client"),
                        ])
                ]),
                bt.ParallelWithMemoryNode([
                    bt_ros.MoveLineToPoint(self.redium_zone_second, "move_client"),
                    bt_ros.SetManipulatortoGroundDelay("manipulator_client")
                ], threshold=2),
                bt_ros.ReleaseFromManipulator("manipulator_client"),
                bt.ParallelWithMemoryNode([
                    bt_ros.MoveLineToPoint(self.eighth_puck + (0, -0.04, 0), "move_client"),
                    bt_ros.SetToWall_ifReachedGoal(self.eighth_puck, "manipulator_client")
                ], threshold=2)
            ]),
            bt.SequenceWithMemoryNode([
                    bt_ros.StopPump("manipulator_client"),
                    bt_ros.MoveLineToPoint(self.seventh_puck + (0, -0.04, 0), "move_client"),
                    bt.ParallelWithMemoryNode([
                        bt_ros.MoveLineToPoint(self.eighth_puck + (0, -0.04, 0), "move_client"),
                        bt_ros.SetToWall_ifReachedGoal(self.eighth_puck, "manipulator_client")
                    ], threshold=2)
                ])
        ])

        eighth_puck = bt.FallbackWithMemoryNode([
            bt.SequenceWithMemoryNode([
                bt_ros.StartPump("manipulator_client"),
                bt_ros.MoveLineToPoint(self.eighth_puck, "move_client"),
                bt.FallbackWithMemoryNode([
                        bt_ros.StartTakeWallPuckWithoutGrabber("manipulator_client"),
                        bt.SequenceWithMemoryNode([
                            bt_ros.MoveLineToPoint(self.eighth_puck + (0, -0.04, 0), "move_client"),
                            bt_ros.MoveLineToPoint(self.eighth_puck, "move_client"),
                            bt_ros.StartTakeWallPuckWithoutGrabber("manipulator_client"),
                        ])
                ]),
                bt.ParallelWithMemoryNode([
                    bt_ros.MoveLineToPoint(self.redium_zone_third, "move_client"),
                    bt_ros.SetManipulatortoGroundDelay("manipulator_client")
                ], threshold=2),
                bt_ros.ReleaseFromManipulator("manipulator_client"),
                bt.ParallelWithMemoryNode([
                    bt_ros.MoveLineToPoint(self.nineth_puck + (side_sign*0.1, -0.04, 0), "move_client"),
                    bt_ros.SetToWall_ifReachedGoal(self.nineth_puck, "manipulator_client")
                ], threshold=2)
            ]),
            bt.SequenceWithMemoryNode([
                    bt_ros.StopPump("manipulator_client"),
                    bt_ros.MoveLineToPoint(self.eighth_puck + (0, -0.04, 0), "move_client"),
                    bt.ParallelWithMemoryNode([
                        bt_ros.MoveLineToPoint(self.eighth_puck + (side_sign*0.2, -0.04, 0), "move_client"),
                        bt_ros.SetToWall_ifReachedGoal(self.eighth_puck + (0.2, -0.04, 0), "manipulator_client")
                    ], threshold=2),
                    bt_ros.MoveLineToPoint(self.nineth_puck + (side_sign*0.15, -0.14, 0), "move_client"),
                ])
        ])

        nineth_puck = bt.SequenceWithMemoryNode([
            bt_ros.MoveLineToPoint(self.nineth_puck + (0, -0.04, 0), "move_client"),
            bt_ros.StartPump("manipulator_client"),
            bt_ros.MoveLineToPoint(self.nineth_puck, "move_client"),
            bt.FallbackWithMemoryNode([
                    bt_ros.StartTakeWallPuckWithoutGrabber("manipulator_client"),
                    bt.SequenceWithMemoryNode([
                        bt_ros.MoveLineToPoint(self.nineth_puck + (0, -0.04, 0), "move_client"),
                        bt_ros.MoveLineToPoint(self.nineth_puck, "move_client"),
                        bt_ros.StartTakeWallPuckWithoutGrabber("manipulator_client"),
                    ])
            ]),
            bt.ParallelWithMemoryNode([
                bt_ros.MoveLineToPoint(self.redium_zone_forth, "move_client"),
                bt_ros.SetManipulatortoGroundDelay("manipulator_client")
            ], threshold=2),
            bt_ros.ReleaseFromManipulator("manipulator_client")
        ])

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