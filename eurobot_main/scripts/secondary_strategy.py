#!/usr/bin/env python
import numpy as np

import rospy
import tf2_ros
from tf.transformations import euler_from_quaternion

import behavior_tree as bt
import bt_ros
from bt_controller import SideStatus
from score_controller import ScoreController

class Strategy(object):
    def __init__(self):
        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer)
        self.robot_name = rospy.get_param("robot_name")
        self.robot_coordinates = None
        self.collected_pucks = bt.BTVariable([])
        self.score_master = ScoreController(self.collected_pucks, self.robot_name)

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



    def is_no_pucks(self):
        if len(self.collected_pucks.get()) == 0:
            return bt.Status.SUCCESS
        else: 
            return bt.Status.FAILED

    def is_no_pucks_1(self):
        if len(self.collected_pucks.get()) == 0:
            return bt.Status.SUCCESS
        else: 
            return bt.Status.RUNNING

class HomoStrategy(Strategy):
    def __init__(self, side):
        super(HomoStrategy, self).__init__()

        if side == SideStatus.PURPLE:
            param = "purple_side"
            side_sign = -1
        elif side == SideStatus.YELLOW:
            param = "yellow_side"
            side_sign = 1

        self.sixth_puck = np.array(rospy.get_param("secondary_robot/" + param + "/sixth_puck_zone"))
        self.seventh_puck = np.array(rospy.get_param("secondary_robot/" + param + "/seventh_puck_zone"))
        self.eighth_puck = np.array(rospy.get_param("secondary_robot/" + param + "/eighth_puck_zone"))


        sixth_puck = bt.FallbackWithMemoryNode([
            bt.SequenceWithMemoryNode([
                bt.ParallelWithMemoryNode([
                    bt_ros.MoveLineToPoint(self.sixth_puck + (0, -0.05, 0), "move_client"),
                    bt_ros.SetToWall_ifReachedGoal(self.sixth_puck, "manipulator_client")
                ], threshold=2),
                bt_ros.StartPump("manipulator_client"),
                bt_ros.MoveLineToPoint(self.sixth_puck, "move_client"), 
                bt_ros.TryToPumpWallPuckWithoutGrabber(self.sixth_puck),
                bt.ActionNode(lambda: self.score_master.add("REDIUM")),
                bt_ros.MoveLineToPoint(self.sixth_puck + (0, -0.05, 0), "move_client"),
                bt.ParallelWithMemoryNode([
                    bt_ros.MoveLineToPoint(self.redium_zone_first, "move_client"),
                    bt_ros.SetToGround_ifReachedGoal(self.sixth_puck + (side_sign*0.2, -0.2, 0), "manipulator_client"),
                    bt_ros.PublishScore_ifReachedGoal(self.redium_zone_center, self.score_master, "RED")
                ], threshold=3),
                bt_ros.ReleaseFromManipulator("manipulator_client"),
                bt.ParallelWithMemoryNode([
                    bt_ros.MoveLineToPoint(self.seventh_puck + (0, -0.05, 0), "move_client"),
                    bt_ros.SetToWall_ifReachedGoal(self.seventh_puck, "manipulator_client")
                ], threshold=2)
            ]),
            bt_ros.MoveToNextPuckIfFailedToStartZone(self.sixth_puck, self.seventh_puck)
        ])

        seventh_puck = bt.FallbackWithMemoryNode([
            bt.SequenceWithMemoryNode([
                bt_ros.StartPump("manipulator_client"),
                bt_ros.MoveLineToPoint(self.seventh_puck, "move_client"), 
                bt_ros.TryToPumpWallPuckWithoutGrabber(self.seventh_puck),
                bt.ActionNode(lambda: self.score_master.add("REDIUM")),
                bt_ros.MoveLineToPoint(self.seventh_puck + (0, -0.05, 0), "move_client"),
                bt.ParallelWithMemoryNode([
                    bt_ros.MoveLineToPoint(self.redium_zone_second, "move_client"),
                    bt_ros.SetToGround_ifReachedGoal(self.seventh_puck + (side_sign*0.2, -0.2, 0), "manipulator_client"),
                    bt_ros.PublishScore_ifReachedGoal(self.redium_zone_center, self.score_master, "RED")
                ], threshold=3),
                bt_ros.ReleaseFromManipulator("manipulator_client"),
                bt.ParallelWithMemoryNode([
                    bt_ros.MoveLineToPoint(self.eighth_puck + (0, -0.05, 0), "move_client"),
                    bt_ros.SetToWall_ifReachedGoal(self.eighth_puck, "manipulator_client")
                ], threshold=2)
            ]),
            bt_ros.MoveToNextPuckIfFailedToStartZone(self.seventh_puck, self.eighth_puck)
        ])

        eighth_puck = bt.FallbackWithMemoryNode([
            bt.SequenceWithMemoryNode([
                bt_ros.StartPump("manipulator_client"),
                bt_ros.MoveLineToPoint(self.eighth_puck, "move_client"), 
                bt_ros.TryToPumpWallPuckWithoutGrabber(self.eighth_puck),
                # bt.ParallelWithMemoryNode([
                #     bt_ros.TryToPumpWallPuckWithoutGrabber(self.eighth_puck),
                #     bt_ros.SetSpeedSTM([0.1, 0, 0], 0.8, "stm_client")
                #     # bt_ros.MoveLineToPoint(self.first_puck, "move_client"),
                # ],threshold=1),
                bt.ActionNode(lambda: self.score_master.add("REDIUM")),
                bt_ros.MoveLineToPoint(self.eighth_puck + (0, -0.05, 0), "move_client"),
                bt.ParallelWithMemoryNode([
                    bt_ros.MoveLineToPoint(self.redium_zone_third, "move_client"),
                    bt_ros.SetToGround_ifReachedGoal(self.eighth_puck + (side_sign*0.2, -0.2, 0), "manipulator_client"),
                    # bt_ros.SetManipulatortoGroundDelay("manipulator_client"),
                    bt_ros.PublishScore_ifReachedGoal(self.redium_zone_center, self.score_master, "RED")
                ], threshold=3),
                bt_ros.ReleaseFromManipulator("manipulator_client")
            ])
        ])

        self.tree = bt.SequenceWithMemoryNode([
            sixth_puck,
            seventh_puck,
            eighth_puck
        ])

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
        self.redium_zone_center = np.array(rospy.get_param("secondary_robot/" + param + "/redium_zone_center"))


        first_puck = bt.FallbackWithMemoryNode([
            bt.SequenceWithMemoryNode([
                bt.ParallelWithMemoryNode([
                    bt_ros.MoveLineToPoint(self.first_puck + (0, -0.05, 0), "move_client"),
                    bt_ros.SetToWall_ifReachedGoal(self.first_puck + (0, -0.15, 0), "manipulator_client")
                ], threshold=2),
                bt_ros.StartPump("manipulator_client"),

                # bt_ros.MoveLineToPoint(self.first_puck, "move_client"), 

                bt.ParallelWithMemoryNode([
                    bt_ros.MoveLineToPoint(self.first_puck, "move_client"),
                    bt_ros.CheckLimitSwitchInf("manipulator_client")
                ], threshold=1),

                bt_ros.TryToPumpWallPuck(self.first_puck),
                # bt.ParallelWithMemoryNode([
                #     bt_ros.TryToPumpWallPuck(self.first_puck),
                #     bt_ros.SetSpeedSTM([0.1, 0, 0], 0.8, "stm_client")
                #     # bt_ros.MoveLineToPoint(self.first_puck, "move_client"),
                # ],threshold=1),

                bt.ParallelWithMemoryNode([
                    bt.SequenceWithMemoryNode([
                        bt_ros.MoveLineToPoint(self.first_puck + (side_sign*0.05, -0.1, 0), "move_client"),
                        #bt_ros.MoveLineToPoint(self.first_puck + (side_sign*0.09, -0.1, 0), "move_client")
                    ]),
                    bt.SequenceWithMemoryNode([
                        bt_ros.CompleteTakeWallPuck("manipulator_client"),
                        bt.ActionNode(lambda: self.score_master.add("GREENIUM"))
                    ])
                ], threshold=2)
            ]),
            bt.SequenceWithMemoryNode([
                bt_ros.StopPump("manipulator_client"),
                bt_ros.MoveLineToPoint(self.first_puck + (side_sign*0.05, -0.1, 0), "move_client"),
                bt.ParallelWithMemoryNode([
                    bt_ros.MoveLineToPoint(self.first_puck + (side_sign*0.08, -0.1, 0), "move_client"),
                    bt_ros.SetToWall_ifReachedGoal(self.first_puck + (side_sign*0.07, -0.15, 0), "manipulator_client")
                ], threshold = 2)
            ])

        ])


        second_puck = bt.FallbackWithMemoryNode([
            bt.SequenceWithMemoryNode([
                bt_ros.StartPump("manipulator_client"),
                bt.ParallelNode([
                    bt_ros.MoveLineToPoint(self.second_puck, "move_client"),
                    bt_ros.CheckLimitSwitchInf("manipulator_client")
                ], threshold=1),
                # bt_ros.MoveLineToPoint(self.second_puck, "move_client"), 
                bt_ros.TryToPumpWallPuck(self.second_puck),
                # bt.ParallelWithMemoryNode([
                #     bt_ros.TryToPumpWallPuck(self.second_puck),
                #     bt_ros.SetSpeedSTM([0.1, 0, 0], 2, "stm_client")
                #     # bt_ros.MoveLineToPoint(self.first_puck, "move_client"),
                # ],threshold=1),
                bt.ParallelWithMemoryNode([
                    bt.SequenceWithMemoryNode([
                        bt_ros.MoveLineToPoint(self.second_puck + (side_sign*-0.1, -0.5, 0), "move_client"),
                        bt_ros.MoveLineToPoint(self.third_puck + (0, -0.05, 0), "move_client")
                    ]),
                    bt.SequenceWithMemoryNode([
                        bt_ros.CompleteTakeWallPuck("manipulator_client"),
                        bt.ActionNode(lambda: self.score_master.add("BLUNIUM"))
                    ])
                ], threshold=2),
            ]),
            bt.SequenceWithMemoryNode([
                    bt_ros.StopPump("manipulator_client"),
                    bt_ros.MoveLineToPoint(self.second_puck + (side_sign*-0.1, -0.5, 0), "move_client"),
                    bt_ros.MoveLineToPoint(self.third_puck + (0, -0.05, 0), "move_client"),
                    bt_ros.SetToWall_ifReachedGoal(self.third_puck + (0, -0.15, 0), "manipulator_client")
                ])
        ])

        third_puck = bt.FallbackWithMemoryNode([
            bt.SequenceWithMemoryNode([
                bt_ros.StartPump("manipulator_client"),
                bt.ParallelWithMemoryNode([
                    bt_ros.MoveLineToPoint(self.third_puck, "move_client"),
                    bt_ros.CheckLimitSwitchInf("manipulator_client")
                ], threshold=1),
                # bt_ros.MoveLineToPoint(self.third_puck, "move_client"), 
                bt_ros.TryToPumpWallPuck(self.third_puck),
                # bt.ParallelWithMemoryNode([
                #     bt_ros.TryToPumpWallPuck(self.third_puck),
                #     bt_ros.SetSpeedSTM([0.1, 0, 0], 0.8, "stm_client")
                #     # bt_ros.MoveLineToPoint(self.first_puck, "move_client"),
                # ],threshold=1),
                bt_ros.CompleteTakePuckAndMoveToNext(self.third_puck, self.forth_puck, self.score_master, "GREENIUM")
            ]),
            bt_ros.MoveToNextPuckIfFailedToScales(self.third_puck, self.forth_puck)
        ])

        forth_puck = bt.FallbackWithMemoryNode([
            bt.SequenceWithMemoryNode([
                bt_ros.SetToWall_ifReachedGoal(self.forth_puck + (0, -0.15, 0), "manipulator_client"),
                bt_ros.StartPump("manipulator_client"),
                bt.ParallelWithMemoryNode([
                    bt_ros.MoveLineToPoint(self.forth_puck, "move_client"),
                    bt_ros.CheckLimitSwitchInf("manipulator_client")
                ], threshold=1),
                # bt_ros.MoveLineToPoint(self.forth_puck, "move_client"), 
                bt_ros.TryToPumpWallPuck(self.forth_puck),
                # bt.ParallelWithMemoryNode([
                #     bt_ros.TryToPumpWallPuck(self.forth_puck),
                #     bt_ros.SetSpeedSTM([0.1, 0, 0], 0.8, "stm_client")
                #     # bt_ros.MoveLineToPoint(self.first_puck, "move_client"),
                # ],threshold=1),
                bt_ros.CompleteTakePuckAndMoveToNext(self.forth_puck, self.fifth_puck, self.score_master, "BLUNIUM")
            ]),
            bt_ros.MoveToNextPuckIfFailedToScales(self.forth_puck, self.fifth_puck)
        ])

        scales = bt.SequenceWithMemoryNode([
            bt_ros.MoveLineToPoint(self.fifth_puck + (0, -0.04, 0), "move_client"),
            bt.ParallelWithMemoryNode([
                bt_ros.MoveLineToPoint(self.scales_zone + (0, -0.14, 0), "move_client"),
                bt.SequenceWithMemoryNode([
                    bt_ros.CompleteCollectLastWall("manipulator_client"),
                    bt.ActionNode(lambda: self.score_master.add("GREENIUM"))
                ])
            ], threshold=2),
            bt.ParallelWithMemoryNode([
                bt_ros.MoveLineToPoint(self.scales_zone, "move_client"),
                bt_ros.StepperUp("manipulator_client"),
            ],threshold=2)
        ])

        fifth_puck = bt.FallbackWithMemoryNode([
            bt.SequenceWithMemoryNode([
                bt_ros.StartPump("manipulator_client"),
                bt.ParallelWithMemoryNode([
                    bt_ros.MoveLineToPoint(self.fifth_puck, "move_client"),
                    bt_ros.CheckLimitSwitchInf("manipulator_client")
                ], threshold=1),
                # bt_ros.MoveLineToPoint(self.fifth_puck, "move_client"), 
                bt_ros.TryToPumpWallPuck(self.fifth_puck),
                # bt.ParallelWithMemoryNode([
                #     bt_ros.TryToPumpWallPuck(self.fifth_puck),
                #     bt_ros.SetSpeedSTM([0.1, 0, 0], 0.8, "stm_client")
                #     # bt_ros.MoveLineToPoint(self.first_puck, "move_client"),
                # ],threshold=1),
                scales
            ]),
            bt.SequenceWithMemoryNode([
                bt.ParallelWithMemoryNode([
                    bt_ros.MovingDefault("manipulator_client"),
                    bt_ros.MoveLineToPoint(self.scales_zone + (0, -0.14, 0), "move_client")
                ], threshold=2),
                bt.ParallelWithMemoryNode([
                    bt_ros.MoveLineToPoint(self.scales_zone, "move_client"),
                    bt_ros.StepperUp("manipulator_client"),
                ],threshold=2)
                
            ])
        ])

        unload = bt.SequenceNode([
            bt.FallbackNode([
                bt.ConditionNode(self.is_no_pucks),
                bt.SequenceWithMemoryNode([
                    bt_ros.ReleaseOnePuck("manipulator_client"),
                    bt.ActionNode(lambda: self.score_master.unload("SCALES")),
                ])
            ]),
            bt.ConditionNode(self.is_no_pucks_1)
        ])

        sixth_puck = bt.FallbackWithMemoryNode([
            bt.SequenceWithMemoryNode([
                bt_ros.MoveLineToPoint(self.scales_zone + (0, -0.14, 0), "move_client"),
                bt.ParallelWithMemoryNode([
                    bt_ros.MoveLineToPoint(self.sixth_puck + (0, -0.05, 0), "move_client"),
                    bt_ros.SetToWall_ifReachedGoal(self.sixth_puck, "manipulator_client")
                ], threshold=2),
                bt_ros.StartPump("manipulator_client"),
                bt.ParallelWithMemoryNode([
                    bt_ros.MoveLineToPoint(self.sixth_puck, "move_client"),
                    bt_ros.CheckLimitSwitchInf("manipulator_client")
                ], threshold=1),
                # bt_ros.MoveLineToPoint(self.sixth_puck, "move_client"), 
                bt_ros.TryToPumpWallPuckWithoutGrabber(self.sixth_puck),
                # bt.ParallelWithMemoryNode([
                #     bt_ros.TryToPumpWallPuckWithoutGrabber(self.sixth_puck),
                #     bt_ros.SetSpeedSTM([0.1, 0, 0], 0.8, "stm_client")
                #     # bt_ros.MoveLineToPoint(self.first_puck, "move_client"),
                # ],threshold=1),
                bt.ActionNode(lambda: self.score_master.add("REDIUM")),
                bt_ros.MoveLineToPoint(self.sixth_puck + (0, -0.05, 0), "move_client"),
                bt.ParallelWithMemoryNode([
                    bt_ros.MoveLineToPoint(self.redium_zone_first, "move_client"),
                    bt_ros.SetToGround_ifReachedGoal(self.sixth_puck + (side_sign*0.2, -0.2, 0), "manipulator_client"),
                    # bt_ros.SetManipulatortoGroundDelay("manipulator_client"),
                    bt_ros.PublishScore_ifReachedGoal(self.redium_zone_first, self.score_master, "RED")
                ], threshold=3),
                bt_ros.ReleaseFromManipulator("manipulator_client"),
                bt.ParallelWithMemoryNode([
                    bt_ros.MoveLineToPoint(self.seventh_puck + (0, -0.05, 0), "move_client"),
                    bt_ros.SetToWall_ifReachedGoal(self.seventh_puck, "manipulator_client")
                ], threshold=2)
            ]),
            bt_ros.MoveToNextPuckIfFailedToStartZone(self.sixth_puck, self.seventh_puck)
        ])

        seventh_puck = bt.FallbackWithMemoryNode([
            bt.SequenceWithMemoryNode([
                bt_ros.StartPump("manipulator_client"),
                bt.ParallelWithMemoryNode([
                    bt_ros.MoveLineToPoint(self.seventh_puck, "move_client"),
                    bt_ros.CheckLimitSwitchInf("manipulator_client")
                ], threshold=1),
                # bt_ros.MoveLineToPoint(self.seventh_puck, "move_client"), 
                bt_ros.TryToPumpWallPuckWithoutGrabber(self.seventh_puck),
                # bt.ParallelWithMemoryNode([
                #     bt_ros.TryToPumpWallPuckWithoutGrabber(self.seventh_puck),
                #     bt_ros.SetSpeedSTM([0.1, 0, 0], 0.8, "stm_client")
                #     # bt_ros.MoveLineToPoint(self.first_puck, "move_client"),
                # ],threshold=1),
                bt.ActionNode(lambda: self.score_master.add("REDIUM")),
                bt_ros.MoveLineToPoint(self.seventh_puck + (0, -0.05, 0), "move_client"),
                bt.ParallelWithMemoryNode([
                    bt_ros.MoveLineToPoint(self.redium_zone_second, "move_client"),
                    bt_ros.SetToGround_ifReachedGoal(self.seventh_puck + (side_sign*0.2, -0.2, 0), "manipulator_client"),
                    # bt_ros.SetManipulatortoGroundDelay("manipulator_client"),
                    bt_ros.PublishScore_ifReachedGoal(self.redium_zone_second, self.score_master, "RED")
                ], threshold=3),
                bt_ros.ReleaseFromManipulator("manipulator_client"),
                bt.ParallelWithMemoryNode([
                    bt_ros.MoveLineToPoint(self.eighth_puck + (0, -0.05, 0), "move_client"),
                    bt_ros.SetToWall_ifReachedGoal(self.eighth_puck, "manipulator_client")
                ], threshold=2)
            ]),
            bt_ros.MoveToNextPuckIfFailedToStartZone(self.seventh_puck, self.eighth_puck)
        ])

        eighth_puck = bt.FallbackWithMemoryNode([
            bt.SequenceWithMemoryNode([
                bt_ros.StartPump("manipulator_client"),
                bt.ParallelWithMemoryNode([
                    bt_ros.MoveLineToPoint(self.eighth_puck, "move_client"),
                    bt_ros.CheckLimitSwitchInf("manipulator_client")
                ], threshold=1),
                # bt_ros.MoveLineToPoint(self.eighth_puck, "move_client"), 
                bt_ros.TryToPumpWallPuckWithoutGrabber(self.eighth_puck),
                # bt.ParallelWithMemoryNode([
                #     bt_ros.TryToPumpWallPuckWithoutGrabber(self.eighth_puck),
                #     bt_ros.SetSpeedSTM([0.1, 0, 0], 0.8, "stm_client")
                #     # bt_ros.MoveLineToPoint(self.first_puck, "move_client"),
                # ],threshold=1),
                bt.ActionNode(lambda: self.score_master.add("REDIUM")),
                bt_ros.MoveLineToPoint(self.eighth_puck + (0, -0.05, 0), "move_client"),
                bt.ParallelWithMemoryNode([
                    bt_ros.MoveLineToPoint(self.redium_zone_third, "move_client"),
                    bt_ros.SetToGround_ifReachedGoal(self.eighth_puck + (side_sign*0.2, -0.2, 0), "manipulator_client"),
                    # bt_ros.SetManipulatortoGroundDelay("manipulator_client"),
                    bt_ros.PublishScore_ifReachedGoal(self.redium_zone_third, self.score_master, "RED")
                ], threshold=3),
                bt_ros.ReleaseFromManipulator("manipulator_client"),
                bt.ParallelWithMemoryNode([
                    bt_ros.MoveLineToPoint(self.nineth_puck + (side_sign*0.1, -0.05, 0), "move_client"),
                    bt_ros.SetToWall_ifReachedGoal(self.nineth_puck, "manipulator_client")
                ], threshold=2)
            ]),
            bt.SequenceWithMemoryNode([
                    bt_ros.StopPump("manipulator_client"),
                    bt_ros.MoveLineToPoint(self.eighth_puck + (0, -0.05, 0), "move_client"),
                    bt.ParallelWithMemoryNode([
                        bt_ros.MoveLineToPoint(self.eighth_puck + (side_sign*0.2, -0.05, 0), "move_client"),
                        bt_ros.SetToWall_ifReachedGoal(self.eighth_puck + (side_sign*0.2, -0.05, 0), "manipulator_client")
                    ], threshold=2),
                    bt_ros.MoveLineToPoint(self.nineth_puck + (side_sign*0.15, -0.14, 0), "move_client"),
                ])
        ])

        nineth_puck = bt.SequenceWithMemoryNode([
            bt_ros.MoveLineToPoint(self.nineth_puck + (0, -0.05, 0), "move_client"),
            bt_ros.StartPump("manipulator_client"),
            bt.ParallelWithMemoryNode([
                    bt_ros.MoveLineToPoint(self.nineth_puck, "move_client"),
                    bt_ros.CheckLimitSwitchInf("manipulator_client")
                ], threshold=1),
            # bt_ros.MoveLineToPoint(self.nineth_puck, "move_client"),
            bt_ros.TryToPumpWallPuckWithoutGrabber(self.nineth_puck),
                # bt.ParallelWithMemoryNode([
                #     bt_ros.TryToPumpWallPuckWithoutGrabber(self.nineth_puck),
                #     bt_ros.SetSpeedSTM([0.1, 0, 0], 0.8, "stm_client")
                #     # bt_ros.MoveLineToPoint(self.first_puck, "move_client"),
                # ],threshold=1),
            bt.ActionNode(lambda: self.score_master.add("REDIUM")),
            bt.ParallelWithMemoryNode([
                bt_ros.MoveLineToPoint(self.redium_zone_forth, "move_client"),
                bt_ros.SetToGround_ifReachedGoal(self.nineth_puck + (side_sign*0.2, -0.2, 0), "manipulator_client"),
                # bt_ros.SetManipulatortoGroundDelay("manipulator_client"),
                bt_ros.PublishScore_ifReachedGoal(self.redium_zone_forth, self.score_master, "RED")
            ], threshold=3),
            bt_ros.ReleaseFromManipulator("manipulator_client"),
            bt.ActionNode(lambda: self.score_master.unload("RED")),
        ])

        self.tree = bt.SequenceWithMemoryNode([
            first_puck,
            second_puck,
            third_puck,
            forth_puck,
            fifth_puck,            
            unload,
            sixth_puck,
            seventh_puck,
            eighth_puck,
            nineth_puck])
