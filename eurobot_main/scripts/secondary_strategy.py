#!/usr/bin/env python
import numpy as np

import rospy
import tf2_ros
from tf.transformations import euler_from_quaternion

import behavior_tree as bt
import bt_ros
from bt_controller import SideStatus
from score_controller import ScoreController
from tactics_math import get_color, yolo_parse_pucks, initial_parse_pucks
from visualization_msgs.msg import MarkerArray

class Strategy(object):
    def __init__(self, side):
        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer)
        self.robot_name = rospy.get_param("robot_name")
        self.robot_coordinates = None
        self.collected_pucks = bt.BTVariable([])
        self.chaos_radius = rospy.get_param("chaos_radius")
        self.purple_chaos_center = rospy.get_param("main_robot" + "/" + "purple_side" + "/chaos_center")
        self.yellow_chaos_center = rospy.get_param("main_robot" + "/" + "yellow_side" + "/chaos_center")
        self.purple_cells_area = rospy.get_param("main_robot" + "/" + "purple_side" + "/purple_cells_area")
        self.yellow_cells_area = rospy.get_param("main_robot" + "/" + "yellow_side" + "/yellow_cells_area")
        self.final_search_area = rospy.get_param("final_search_area")
        self.visible_pucks_on_field = bt.BTVariable(np.array([]))  # (x, y, id, r, g, b)

        self.is_robot_started = bt.BTVariable(False)
        self.our_pucks_rgb = bt.BTVariable(np.array([[0, 0, 0, 0, 0, 0],
                                                     [0, 0, 1, 0, 0, 0],
                                                     [0, 0, 2, 0, 0, 0]]))  # (x, y, id, r, g, b)

        self.side = side
        self.score_master = ScoreController(self.collected_pucks, self.robot_name)

        if side == SideStatus.PURPLE:
            self.color_side = "purple_side"
            self.opponent_side = "yellow_side"
            self.sign = -1
        elif side == SideStatus.YELLOW:
            self.color_side = "yellow_side"
            self.opponent_side = "purple_side"
            self.sign = 1

        self.strategy_status_subscriber = rospy.Subscriber("/pucks", MarkerArray, self.parse_pucks)

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

    def parse_pucks(self, data):
        # [(0.95, 1.1, 3, 0, 0, 1), ...] - blue, id=3  IDs are not guaranteed to be the same from frame to frame
        # red (1, 0, 0)
        # green (0, 1, 0)
        # blue (0, 0, 1)
        # rospy.loginfo(data)

        try:
            new_observation_pucks = [[marker.pose.position.x,
                                      marker.pose.position.y,
                                      marker.id,
                                      marker.color.r,
                                      marker.color.g,
                                      marker.color.b] for marker in data.markers]

            # Updating all pucks from scratch with each new observation if robots are still waiting.
            # When robots start, there occurs possibility that camera just doesn't see some of pucks or they are already collected:
            # in this case function compare_to_update_or_ignore is used

            if self.is_robot_started.get() is False:

                __, __, purple_pucks_rgb, yellow_pucks_rgb = initial_parse_pucks(
                                                            new_observation_pucks,
                                                            self.purple_chaos_center,
                                                            self.yellow_chaos_center,
                                                            self.chaos_radius,
                                                            self.purple_cells_area,
                                                            self.yellow_cells_area)


                if self.color_side == "purple_side":
                    if len(purple_pucks_rgb) == 3:
                        self.our_pucks_rgb.set(purple_pucks_rgb)

                elif self.color_side == "yellow_side":
                    if len(yellow_pucks_rgb) == 3:
                        self.our_pucks_rgb.set(yellow_pucks_rgb)

            if self.is_robot_started.get() is True:
                lost_pucks = yolo_parse_pucks(new_observation_pucks,
                                              self.final_search_area)
                self.visible_pucks_on_field.set(lost_pucks)

        except Exception:  # FIXME
            rospy.loginfo("list index out of range - no visible pucks on the field ")

    def is_observed(self):
        # rospy.loginfo("is_observed")
        # rospy.loginfo(self.visible_pucks_on_field.get())
        if len(self.our_pucks_rgb.get()) > 0:
            rospy.loginfo('pucks near cells observed')
            return bt.Status.SUCCESS
        else:
            rospy.loginfo('no pucks found')
            return bt.Status.FAILED

    def is_last_puck_inside(self):
        if len(self.collected_pucks.get()) == 1:
            return bt.Status.SUCCESS
        else:
            return bt.Status.FAILED

    def is_last_puck_inside_1(self):
        if len(self.collected_pucks.get()) == 1:
            return bt.Status.SUCCESS
        else:
            return bt.Status.RUNNING

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
    def __init__(self):
        super(HomoStrategy, self).__init__(side)

        if self.side == SideStatus.PURPLE:
            param = "purple_side"
            side_sign = -1
        elif self.side == SideStatus.YELLOW:
            param = "yellow_side"
            side_sign = 1

        self.sixth_puck = np.array(rospy.get_param("secondary_robot/" + param + "/sixth_puck_zone"))
        self.seventh_puck = np.array(rospy.get_param("secondary_robot/" + param + "/seventh_puck_zone"))
        self.eighth_puck = np.array(rospy.get_param("secondary_robot/" + param + "/eighth_puck_zone"))
        self.start_zone = np.array(rospy.get_param("secondary_robot/" + param + "/start_zone"))
        self.redium_zone_first = np.array(rospy.get_param("secondary_robot/" + param + "/redium_zone_first"))
        self.redium_zone_second = np.array(rospy.get_param("secondary_robot/" + param + "/redium_zone_second"))
        self.redium_zone_third = np.array(rospy.get_param("secondary_robot/" + param + "/redium_zone_third"))
        self.redium_zone_forth = np.array(rospy.get_param("secondary_robot/" + param + "/redium_zone_forth"))
        self.redium_zone_center = np.array(rospy.get_param("secondary_robot/" + param + "/redium_zone_center"))

        sixth_puck = bt.FallbackWithMemoryNode([
            bt.SequenceWithMemoryNode([
                bt_ros.MoveLineToPoint(self.start_zone + (0, 0, -side_sign * 1.57), "move_client"),
                bt.ParallelWithMemoryNode([
                    bt_ros.MoveLineToPoint(self.sixth_puck + (0, -0.05, 0), "move_client"),
                    bt_ros.SetToWall_ifReachedGoal(self.sixth_puck, "manipulator_client")
                ], threshold=2),
                bt_ros.StartPump("manipulator_client"),
                bt_ros.MoveLineToPoint(self.sixth_puck, "move_client"),
                bt_ros.TryToPumpWallPuckWithoutGrabber(self.sixth_puck),
                bt.ActionNode(lambda: self.score_master.add("REDIUM")),
                bt_ros.MoveLineToPoint(self.sixth_puck + (0, -0.1, 0), "move_client"),
                bt_ros.SetManipulatortoGround("manipulator_client"),
                bt_ros.MoveLineToPoint(self.sixth_puck + (0, -0.1, -3.14), "move_client"),
                bt.ParallelWithMemoryNode([
                    bt_ros.MoveLineToPoint(self.redium_zone_first, "move_client"),
                    bt_ros.PublishScore_ifReachedGoal(self.redium_zone_center, self.score_master, "RED")
                ], threshold=2),
                bt_ros.ReleaseFromManipulator("manipulator_client"),
                bt_ros.MoveLineToPoint(self.redium_zone_first + (0, 0, -side_sign * 1.57), "move_client"),
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
                bt_ros.MoveLineToPoint(self.seventh_puck + (0, -0.1, 0), "move_client"),
                bt_ros.SetManipulatortoGround("manipulator_client"),
                bt_ros.MoveLineToPoint(self.seventh_puck + (0, -0.1, -3.14), "move_client"),
                bt.ParallelWithMemoryNode([
                    bt_ros.MoveLineToPoint(self.redium_zone_second, "move_client"),
                    bt_ros.PublishScore_ifReachedGoal(self.redium_zone_center, self.score_master, "RED")
                ], threshold=2),
                bt_ros.ReleaseFromManipulator("manipulator_client"),
                bt_ros.MoveLineToPoint(self.redium_zone_second + (0, 0, -side_sign * 1.57), "move_client"),
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
                bt.ActionNode(lambda: self.score_master.add("REDIUM")),
                bt_ros.MoveLineToPoint(self.eighth_puck + (0, -0.1, 0), "move_client"),
                bt_ros.SetManipulatortoGround("manipulator_client"),
                bt_ros.MoveLineToPoint(self.eighth_puck + (0, -0.1, -3.14), "move_client"),
                bt.ParallelWithMemoryNode([
                    bt_ros.MoveLineToPoint(self.redium_zone_third, "move_client"),
                    bt_ros.PublishScore_ifReachedGoal(self.redium_zone_center, self.score_master, "RED")
                ], threshold=2),
                bt_ros.ReleaseFromManipulator("manipulator_client")
            ])
        ])

        self.tree = bt.SequenceWithMemoryNode([
            sixth_puck,
            seventh_puck,
            eighth_puck
        ])


class SemaPidrStrategy(Strategy):
    def __init__(self, side):
        super(SemaPidrStrategy, self).__init__(side)

        if side == SideStatus.PURPLE:
            param = "purple_side"
            side_sign = -1
        elif side == SideStatus.YELLOW:
            param = "yellow_side"
            side_sign = 1

        self.sixth_puck = np.array(rospy.get_param("secondary_robot/" + param + "/sixth_puck_zone"))
        self.seventh_puck = np.array(rospy.get_param("secondary_robot/" + param + "/seventh_puck_zone"))
        self.eighth_puck = np.array(rospy.get_param("secondary_robot/" + param + "/eighth_puck_zone"))
        self.start_zone = np.array(rospy.get_param("secondary_robot/" + param + "/start_zone"))
        self.redium_zone_first = np.array(rospy.get_param("secondary_robot/" + param + "/redium_zone_first"))
        self.redium_zone_second = np.array(rospy.get_param("secondary_robot/" + param + "/redium_zone_second"))
        self.redium_zone_third = np.array(rospy.get_param("secondary_robot/" + param + "/redium_zone_third"))
        self.redium_zone_forth = np.array(rospy.get_param("secondary_robot/" + param + "/redium_zone_forth"))
        self.redium_zone_center = np.array(rospy.get_param("secondary_robot/" + param + "/redium_zone_center"))
        self.reflected_last_zone = np.array(rospy.get_param("secondary_robot/" + param + "/reflected_last_zone"))

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
                bt_ros.MoveLineToPoint(self.start_zone + (0, 0, -side_sign * 1.57), "move_client"),
                bt.ParallelWithMemoryNode([
                    bt_ros.MoveLineToPoint(self.sixth_puck + (0, -0.05, 0), "move_client"),
                    bt_ros.SetToWall_ifReachedGoal(self.sixth_puck, "manipulator_client")
                ], threshold=2),
                bt_ros.StartPump("manipulator_client"),
                bt_ros.MoveLineToPoint(self.sixth_puck, "move_client"),
                bt_ros.TryToPumpWallPuck(self.sixth_puck),
                bt.ActionNode(lambda: self.score_master.add("REDIUM")),
                bt_ros.MoveLineToPoint(self.sixth_puck + (0, -0.1, 0), "move_client"),
                bt_ros.CompleteTakeWallPuck("manipulator_client"),
                bt_ros.MoveLineToPoint(self.seventh_puck + (0, -0.05, 0), "move_client")
            ])
        ])

        seventh_puck = bt.FallbackWithMemoryNode([
            bt.SequenceWithMemoryNode([
                bt_ros.MoveLineToPoint(self.seventh_puck, "move_client"),
                bt_ros.TryToPumpWallPuck(self.seventh_puck),
                bt.ActionNode(lambda: self.score_master.add("GREENIUM")),
                bt_ros.MoveLineToPoint(self.seventh_puck + (0, -0.1, 0), "move_client"),
                bt_ros.CompleteTakeWallPuck("manipulator_client"),
                bt_ros.MoveLineToPoint(self.eighth_puck + (0, -0.05, 0), "move_client")
            ])
        ])

        eighth_puck = bt.FallbackWithMemoryNode([
            bt.SequenceWithMemoryNode([
                bt_ros.StartPump("manipulator_client"),
                bt_ros.MoveLineToPoint(self.eighth_puck, "move_client"),
                bt_ros.TryToPumpWallPuck(self.eighth_puck),
                bt.ActionNode(lambda: self.score_master.add("BLUNIUM")),
                bt_ros.MoveLineToPoint(self.eighth_puck + (0, -0.1, 0), "move_client"),
                bt_ros.CompleteTakeWallPuck("manipulator_client"),
                bt_ros.MoveLineToPoint(self.reflected_last_zone, "move_client")
            ])
        ])

        self.tree = bt.SequenceWithMemoryNode([
            sixth_puck,
            seventh_puck,
            eighth_puck,
            bt_ros.SetReleaserSpeedToLow("manipulator_client"),
            bt_ros.StepperUp("manipulator_client"),
            bt_ros.RightMoustacheDown("manipulator_client"),
            unload
        ])


class ReflectedVovanStrategy(Strategy):
    def __init__(self, side):
        super(ReflectedVovanStrategy, self).__init__(side)

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
        self.reflected_last_zone = np.array(rospy.get_param("secondary_robot/" + param + "/reflected_last_zone"))
        self.third_ground_puck = np.array(rospy.get_param("secondary_robot/" + param + "/third_ground_puck"))
        self.second_ground_puck = np.array(rospy.get_param("secondary_robot/" + param + "/second_ground_puck"))

        first_puck = bt.FallbackWithMemoryNode([
            bt.SequenceWithMemoryNode([
                bt.SequenceWithMemoryNode([
                    bt_ros.MoveLineToPoint(self.third_ground_puck + (side_sign * 0.07, 0, 0), "move_client"),
                    bt_ros.MoveLineToPoint(self.third_ground_puck, "move_client"),
                    bt.FallbackWithMemoryNode([
                        bt.ParallelWithMemoryNode([
                            bt_ros.MoveLineToPoint(self.fifth_puck + (0, -0.08, 0), "move_client"),
                            bt.SequenceWithMemoryNode([
                                bt_ros.StartCollectGroundCheck("manipulator_client"),
                                bt_ros.CompleteCollectGround("manipulator_client"),
                                bt.ActionNode(lambda: self.score_master.add(get_color(self.our_pucks_rgb.get()[2])))
                            ]),
                        ], threshold=2),
                        bt_ros.SetManipulatortoWall("manipulator_client"),        
                        # bt_ros.SetToWall_ifReachedGoal(self.fifth_puck + (0, -0.05, 0), "manipulator_client")
                    ]),
                ]),
                bt_ros.SetManipulatortoWall("manipulator_client"),
                bt_ros.StartPump("manipulator_client"),
                bt.ParallelWithMemoryNode([
                    bt_ros.MoveLineToPoint(self.fifth_puck, "move_client"),
                    bt_ros.CheckLimitSwitchInf("manipulator_client")
                ], threshold=1),
                bt_ros.TryToPumpWallPuck(self.fifth_puck),
                bt_ros.CompleteTakePuckAndMoveToNext(self.fifth_puck, self.forth_puck, self.score_master, "GREENIUM")
            ]),
            bt_ros.MoveToNextPuckIfFailedToScales(self.fifth_puck, self.forth_puck)
        ])

        second_puck = bt.FallbackWithMemoryNode([
            bt.SequenceWithMemoryNode([
                bt_ros.SetToWall_ifReachedGoal(self.forth_puck + (0, -0.15, 0), "manipulator_client"),
                bt_ros.StartPump("manipulator_client"),
                bt.ParallelWithMemoryNode([
                    bt_ros.MoveLineToPoint(self.forth_puck, "move_client"),
                    bt_ros.CheckLimitSwitchInf("manipulator_client")
                ], threshold=1),
                bt_ros.TryToPumpWallPuck(self.forth_puck),
                bt_ros.CompleteTakePuckAndMoveToNext(self.forth_puck, self.third_puck, self.score_master, "BLUNIUM")
            ]),
            bt_ros.MoveToNextPuckIfFailedToScales(self.forth_puck, self.third_puck)
        ])

        third_puck = bt.FallbackWithMemoryNode([
            bt.SequenceWithMemoryNode([
                bt_ros.StartPump("manipulator_client"),
                bt.ParallelWithMemoryNode([
                    bt_ros.MoveLineToPoint(self.third_puck, "move_client"),
                    bt_ros.CheckLimitSwitchInf("manipulator_client")
                ], threshold=1),
                bt_ros.TryToPumpWallPuck(self.third_puck),
                bt.ParallelWithMemoryNode([
                    bt.SequenceWithMemoryNode([
                        # bt_ros.MoveLineToPoint(self.third_puck + (side_sign * 0.05, -0.1, 0), "move_client"),                        
                        bt_ros.MoveLineToPoint(self.third_puck + (side_sign * 0.25, -0.1, 0), "move_client"),
                        bt_ros.StopPump("manipulator_client"),
                    ]),
                    bt.SequenceWithMemoryNode([
                        bt_ros.CompleteTakeWallPuck("manipulator_client"),
                        bt.ActionNode(lambda: self.score_master.add("GREENIUM"))
                    ])
                ], threshold=2)
            ]),
            bt.SequenceWithMemoryNode([
                bt_ros.StopPump("manipulator_client"),
                bt_ros.MoveLineToPoint(self.third_puck + (side_sign * 0.25, -0.1, 0), "move_client")
                # bt_ros.MoveLineToPoint(self.third_puck + (side_sign*0.05, -0.1, 0), "move_client"),
                # bt.ParallelWithMemoryNode([
                #     bt_ros.MoveLineToPoint(self.fifth_puck + (side_sign*0.2, -0.1, 0), "move_client"),
                #     bt_ros.SetToWall_ifReachedGoal(self.fifth_puck + (side_sign*0.2, -0.15, 0), "manipulator_client")
                # ], threshold = 2)
            ])

        ])

        forth_puck = bt.FallbackWithMemoryNode([
            bt.SequenceWithMemoryNode([
                bt.ParallelWithMemoryNode([
                    bt_ros.MoveLineToPoint(self.first_puck + (0, -0.05, 0), "move_client"),
                    bt_ros.SetToWall_ifReachedGoal(self.first_puck + (0, -0.15, 0), "manipulator_client")
                ], threshold=2),
                bt_ros.StartPump("manipulator_client"),
                bt.ParallelWithMemoryNode([
                    bt_ros.MoveLineToPoint(self.first_puck, "move_client"),
                    bt_ros.CheckLimitSwitchInf("manipulator_client")
                ], threshold=1),
                bt_ros.TryToPumpWallPuck(self.first_puck),
                bt.ParallelWithMemoryNode([
                    bt.SequenceWithMemoryNode([
                        bt_ros.MoveLineToPoint(self.first_puck + (side_sign * 0.05, -0.08, 0), "move_client"),
                    ]),
                    bt.SequenceWithMemoryNode([
                        bt_ros.CompleteTakeWallPuck("manipulator_client"),
                        bt.ActionNode(lambda: self.score_master.add("GREENIUM"))
                    ])
                ], threshold=2)
            ]),
            bt.SequenceWithMemoryNode([
                bt_ros.StopPump("manipulator_client"),
                bt_ros.MoveLineToPoint(self.first_puck + (side_sign * 0.05, -0.1, 0), "move_client"),
                bt.ParallelWithMemoryNode([
                    bt_ros.MoveLineToPoint(self.first_puck + (side_sign * 0.08, -0.1, 0), "move_client"),
                    bt_ros.SetToWall_ifReachedGoal(self.first_puck + (side_sign * 0.07, -0.15, 0), "manipulator_client")
                ], threshold=2)
            ])

        ])

        fifth_puck = bt.FallbackWithMemoryNode([
            bt.SequenceWithMemoryNode([
                bt_ros.StartPump("manipulator_client"),
                bt.ParallelNode([
                    bt_ros.MoveLineToPoint(self.second_puck, "move_client"),
                    bt_ros.CheckLimitSwitchInf("manipulator_client")
                ], threshold=1),
                bt_ros.TryToPumpWallPuck(self.second_puck),
                bt.ParallelWithMemoryNode([
                    bt.SequenceWithMemoryNode([
                        bt_ros.MoveLineToPoint(self.second_puck + (side_sign * -0.2, -0.45, 0), "move_client")
                    ]),
                    bt.SequenceWithMemoryNode([
                        bt_ros.CompleteCollectLastWall("manipulator_client"),
                        bt.ActionNode(lambda: self.score_master.add("BLUNIUM"))
                    ])
                ], threshold=2),
                bt_ros.StopPump("manipulator_client"),
                bt_ros.MoveLineToPoint(self.scales_zone + (0, -0.1, 0), "move_client"),
                bt.ParallelWithMemoryNode([
                    bt_ros.MoveLineToPoint(self.scales_zone, "move_client"),
                    bt_ros.StepperUp("manipulator_client"),
                ], threshold=2)
            ]),
            bt.SequenceWithMemoryNode([
                bt_ros.StopPump("manipulator_client"),
                bt_ros.MoveLineToPoint(self.second_puck + (side_sign * -0.1, -0.5, 0), "move_client"),
                bt_ros.MoveLineToPoint(self.scales_zone + (0, -0.14, 0), "move_client"),
                bt.ParallelWithMemoryNode([
                    bt_ros.MoveLineToPoint(self.scales_zone, "move_client"),
                    bt_ros.StepperUp("manipulator_client"),
                ], threshold=2)
            ])
        ])

        unload = bt.SequenceNode([
            bt.FallbackNode([
                bt.ConditionNode(self.is_last_puck_inside),
                bt.SequenceWithMemoryNode([
                    bt_ros.ReleaseOnePuck("manipulator_client"),
                    bt.ActionNode(lambda: self.score_master.unload("SCALES")),
                ])
            ]),
            bt.ConditionNode(self.is_last_puck_inside_1)
        ])

        sixth_puck = bt.FallbackWithMemoryNode([
            bt.SequenceWithMemoryNode([
                bt.ParallelWithMemoryNode([
                    bt_ros.MoveLineToPoint(self.sixth_puck + (0, -0.08, 0), "move_client"),
                    bt_ros.SetToWall_ifReachedGoal(self.sixth_puck, "manipulator_client")
                ], threshold=2),
                bt_ros.StartPump("manipulator_client"),
                bt_ros.MoveLineToPoint(self.sixth_puck, "move_client"),
                bt_ros.TryToPumpWallPuck(self.sixth_puck),
                bt_ros.CompleteTakePuckAndMoveToNext(self.sixth_puck, self.seventh_puck, self.score_master, "REDIUM")
            ])
        ])

        seventh_puck = bt.FallbackWithMemoryNode([
            bt.SequenceWithMemoryNode([
                bt_ros.StartPump("manipulator_client"),
                bt_ros.MoveLineToPoint(self.seventh_puck, "move_client"),
                bt_ros.TryToPumpWallPuck(self.seventh_puck),
                bt_ros.CompleteTakePuckAndMoveToNext(self.seventh_puck, self.eighth_puck, self.score_master, "REDIUM")
            ])
        ])

        eighth_puck = bt.FallbackWithMemoryNode([
            bt.SequenceWithMemoryNode([
                bt_ros.StartPump("manipulator_client"),
                bt_ros.MoveLineToPoint(self.eighth_puck, "move_client"),
                bt_ros.TryToPumpWallPuck(self.eighth_puck),
                bt.ActionNode(lambda: self.score_master.add("REDIUM")),

                bt.ParallelWithMemoryNode([
                    bt.SequenceWithMemoryNode([
                        bt_ros.MoveLineToPoint(self.third_puck + (side_sign * 0.4, -0.1, 0), "move_client"),
                        bt_ros.MoveLineToPoint(self.nineth_puck + (0, -0.05, 0), "move_client"),
                    ]),
                    bt_ros.CompleteTakeWallPuck("manipulator_client"),
                ], threshold=2),
            ])
        ])

        nineth_puck = bt.SequenceWithMemoryNode([
            bt_ros.StartPump("manipulator_client"),
            bt.ParallelWithMemoryNode([
                bt_ros.MoveLineToPoint(self.nineth_puck, "move_client"),
                bt_ros.CheckLimitSwitchInf("manipulator_client")
            ], threshold=1),
            bt_ros.TryToPumpWallPuck(self.nineth_puck),
            bt.ActionNode(lambda: self.score_master.add("REDIUM")),
            bt.ParallelWithMemoryNode([
                bt_ros.MoveLineToPoint(self.reflected_last_zone, "move_client"),
                bt_ros.CompleteCollectLastWall("manipulator_client")
            ], threshold=2)
        ])

        unload_to_red = bt.SequenceNode([
            bt.FallbackNode([
                bt.ConditionNode(self.is_no_pucks),
                bt.SequenceWithMemoryNode([
                    bt_ros.ReleaseOnePuck("manipulator_client"),
                    bt.ActionNode(lambda: self.score_master.unload("RED")),
                ])
            ]),
            bt.ConditionNode(self.is_no_pucks_1)
        ])

        leaving_red_zone = bt.SequenceNode([
            bt_ros.MoveLineToPoint(self.reflected_last_zone + (-0.2, 0, 0), "move_client"),
            bt_ros.RightMoustacheDefault("manipulator_client"),
        ])

        second_ground_puck = bt.FallbackWithMemoryNode([
                bt.SequenceWithMemoryNode([
                    bt_ros.MoveLineToPoint(self.second_ground_puck + (side_sign * 0.07, 0, 0), "move_client"),
                    bt_ros.MoveLineToPoint(self.second_ground_puck, "move_client"),
                    bt_ros.StartCollectGroundCheck("manipulator_client"),
                    bt.ActionNode(lambda: self.score_master.add(get_color(self.our_pucks_rgb.get()[1]))),
                    bt.ParallelWithMemoryNode([
                        bt_ros.MoveLineToPoint(self.redium_zone_forth + (0, 0, -1.57), "move_client"),
                        bt_ros.PublishScore_ifReachedGoal(self.redium_zone_forth + (0, 0, -1.57), self.score_master, "RED")
                    ], threshold=2),
                    bt_ros.ReleaseFromManipulator("manipulator_client"),
                    bt_ros.SetManipulatortoUp("manipulator_client")
            ])
        ])

        self.tree = bt.SequenceWithMemoryNode([
            first_puck,
            second_puck,
            third_puck,
            forth_puck,
            fifth_puck,
            unload,
            bt_ros.StepperDown("manipulator_client"),
            bt_ros.SetReleaserSpeedToLow("manipulator_client"),
            sixth_puck,
            seventh_puck,
            eighth_puck,
            nineth_puck,
            bt_ros.StepperUp("manipulator_client"),
            bt_ros.RightMoustacheDown("manipulator_client"),
            unload_to_red,
            leaving_red_zone,
            second_ground_puck])


class VovanStrategy(Strategy):
    def __init__(self, side):
        super(VovanStrategy, self).__init__(side)

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
                bt.ParallelWithMemoryNode([
                    bt_ros.MoveLineToPoint(self.first_puck, "move_client"),
                    bt_ros.CheckLimitSwitchInf("manipulator_client")
                ], threshold=1),
                bt_ros.TryToPumpWallPuck(self.first_puck),
                bt.ParallelWithMemoryNode([
                    bt.SequenceWithMemoryNode([
                        bt_ros.MoveLineToPoint(self.first_puck + (side_sign * 0.05, -0.1, 0), "move_client"),
                    ]),
                    bt.SequenceWithMemoryNode([
                        bt_ros.CompleteTakeWallPuck("manipulator_client"),
                        bt.ActionNode(lambda: self.score_master.add("GREENIUM"))
                    ])
                ], threshold=2)
            ]),
            bt.SequenceWithMemoryNode([
                bt_ros.StopPump("manipulator_client"),
                bt_ros.MoveLineToPoint(self.first_puck + (side_sign * 0.05, -0.1, 0), "move_client"),
                bt.ParallelWithMemoryNode([
                    bt_ros.MoveLineToPoint(self.first_puck + (side_sign * 0.08, -0.1, 0), "move_client"),
                    bt_ros.SetToWall_ifReachedGoal(self.first_puck + (side_sign * 0.07, -0.15, 0), "manipulator_client")
                ], threshold=2)
            ])

        ])

        second_puck = bt.FallbackWithMemoryNode([
            bt.SequenceWithMemoryNode([
                bt_ros.StartPump("manipulator_client"),
                bt.ParallelNode([
                    bt_ros.MoveLineToPoint(self.second_puck, "move_client"),
                    bt_ros.CheckLimitSwitchInf("manipulator_client")
                ], threshold=1),
                bt_ros.TryToPumpWallPuck(self.second_puck),
                bt.ParallelWithMemoryNode([
                    bt.SequenceWithMemoryNode([
                        bt_ros.MoveLineToPoint(self.second_puck + (side_sign * -0.1, -0.5, 0), "move_client"),
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
                bt_ros.MoveLineToPoint(self.second_puck + (side_sign * -0.1, -0.5, 0), "move_client"),
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
                bt_ros.TryToPumpWallPuck(self.third_puck),
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
                bt_ros.TryToPumpWallPuck(self.forth_puck),
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
            ], threshold=2)
        ])

        fifth_puck = bt.FallbackWithMemoryNode([
            bt.SequenceWithMemoryNode([
                bt_ros.StartPump("manipulator_client"),
                bt.ParallelWithMemoryNode([
                    bt_ros.MoveLineToPoint(self.fifth_puck, "move_client"),
                    bt_ros.CheckLimitSwitchInf("manipulator_client")
                ], threshold=1),
                bt_ros.TryToPumpWallPuck(self.fifth_puck),
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
                ], threshold=2)

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
                # bt_ros.MoveLineToPoint(self.scales_zone + (0, -0.14, 0), "move_client"),
                bt.ParallelWithMemoryNode([
                    bt_ros.MoveLineToPoint(self.sixth_puck + (0, -0.08, 0), "move_client"),
                    bt_ros.SetToWall_ifReachedGoal(self.sixth_puck, "manipulator_client", threshold=0.15)
                ], threshold=2),
                bt_ros.StartPump("manipulator_client"),
                bt.ParallelWithMemoryNode([
                    bt_ros.MoveLineToPoint(self.sixth_puck, "move_client"),
                    bt_ros.CheckLimitSwitchInf("manipulator_client")
                ], threshold=1),
                bt_ros.TryToPumpWallPuckWithoutGrabber(self.sixth_puck),
                bt.ActionNode(lambda: self.score_master.add("REDIUM")),
                bt_ros.MoveLineToPoint(self.sixth_puck + (0, -0.05, 0), "move_client"),
                bt.ParallelWithMemoryNode([
                    bt_ros.MoveLineToPoint(self.redium_zone_first, "move_client"),
                    bt_ros.SetToGround_ifReachedGoal(self.sixth_puck + (side_sign * 0.2, -0.2, 0),
                                                     "manipulator_client"),
                    bt_ros.PublishScore_ifReachedGoal(self.redium_zone_first, self.score_master, "RED")
                ], threshold=3),
                bt_ros.ReleaseFromManipulator("manipulator_client"),
                bt.ParallelWithMemoryNode([
                    bt_ros.MoveLineToPoint(self.seventh_puck + (0, -0.05, 0), "move_client"),
                    bt_ros.SetManipulatortoUp("manipulator_client"),
                    bt_ros.SetToWall_ifReachedGoal(self.seventh_puck, "manipulator_client")
                ], threshold=3)
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
                bt_ros.TryToPumpWallPuckWithoutGrabber(self.seventh_puck),
                bt.ActionNode(lambda: self.score_master.add("REDIUM")),
                bt_ros.MoveLineToPoint(self.seventh_puck + (0, -0.05, 0), "move_client"),
                bt.ParallelWithMemoryNode([
                    bt_ros.MoveLineToPoint(self.redium_zone_second, "move_client"),
                    bt_ros.SetToGround_ifReachedGoal(self.seventh_puck + (side_sign * 0.2, -0.2, 0),
                                                     "manipulator_client"),
                    bt_ros.PublishScore_ifReachedGoal(self.redium_zone_second, self.score_master, "RED")
                ], threshold=3),
                bt_ros.ReleaseFromManipulator("manipulator_client"),
                bt.ParallelWithMemoryNode([
                    bt_ros.MoveLineToPoint(self.eighth_puck + (0, -0.05, 0), "move_client"),
                    bt_ros.SetManipulatortoUp("manipulator_client"),
                    bt_ros.SetToWall_ifReachedGoal(self.eighth_puck, "manipulator_client")
                ], threshold=3)
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
                bt_ros.TryToPumpWallPuckWithoutGrabber(self.eighth_puck),
                bt.ActionNode(lambda: self.score_master.add("REDIUM")),
                bt_ros.MoveLineToPoint(self.eighth_puck + (0, -0.05, 0), "move_client"),
                bt.ParallelWithMemoryNode([
                    bt_ros.MoveLineToPoint(self.redium_zone_third, "move_client"),
                    bt_ros.SetToGround_ifReachedGoal(self.eighth_puck + (side_sign * 0.2, -0.2, 0),
                                                     "manipulator_client"),
                    bt_ros.PublishScore_ifReachedGoal(self.redium_zone_third, self.score_master, "RED")
                ], threshold=3),
                bt_ros.ReleaseFromManipulator("manipulator_client"),
                bt.ParallelWithMemoryNode([
                    bt_ros.MoveLineToPoint(self.nineth_puck + (0, -0.05, 0), "move_client"),
                    bt_ros.SetManipulatortoUp("manipulator_client"),
                    bt_ros.SetToWall_ifReachedGoal(self.nineth_puck, "manipulator_client")
                ], threshold=3)
            ]),
            bt.SequenceWithMemoryNode([
                bt_ros.StopPump("manipulator_client"),
                bt_ros.SetManipulatortoUp("manipulator_client"),
                bt_ros.MoveLineToPoint(self.eighth_puck + (0, -0.05, 0), "move_client"),
                bt.ParallelWithMemoryNode([
                    bt_ros.MoveLineToPoint(self.eighth_puck + (side_sign * 0.2, -0.05, 0), "move_client"),
                    bt_ros.SetToWall_ifReachedGoal(self.eighth_puck + (side_sign * 0.2, -0.05, 0), "manipulator_client")
                ], threshold=2),
                bt_ros.MoveLineToPoint(self.nineth_puck + (0, -0.05, 0), "move_client"),
            ])
        ])

        nineth_puck = bt.SequenceWithMemoryNode([
            bt_ros.StartPump("manipulator_client"),
            bt.ParallelWithMemoryNode([
                bt_ros.MoveLineToPoint(self.nineth_puck, "move_client"),
                bt_ros.CheckLimitSwitchInf("manipulator_client")
            ], threshold=1),
            bt_ros.TryToPumpWallPuckWithoutGrabber(self.nineth_puck),
            bt.ActionNode(lambda: self.score_master.add("REDIUM")),
            bt.ParallelWithMemoryNode([
                bt_ros.MoveLineToPoint(self.redium_zone_forth, "move_client"),
                bt_ros.SetToGround_ifReachedGoal(self.nineth_puck + (side_sign * 0.2, -0.2, 0), "manipulator_client"),
                bt_ros.PublishScore_ifReachedGoal(self.redium_zone_center, self.score_master, "RED")
            ], threshold=3),
            bt_ros.ReleaseFromManipulator("manipulator_client"),
            bt_ros.SetManipulatortoUp("manipulator_client"),
            bt.ActionNode(lambda: self.score_master.unload("RED"))
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


class DefenseStrategy(Strategy):
    def __init__(self, side):
        super(DefenseStrategy, self).__init__(side)

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
                    bt_ros.MoveLineToPoint(self.fifth_puck_zone + (0, -0.05, 0), "move_client"),
                    bt_ros.SetToWall_ifReachedGoal(self.fifth_puck_zone + (0, -0.15, 0), "manipulator_client")
                ], threshold=2),
                bt_ros.StartPump("manipulator_client"),
                bt.ParallelWithMemoryNode([
                    bt_ros.MoveLineToPoint(self.fifth_puck_zone, "move_client"),
                    bt_ros.CheckLimitSwitchInf("manipulator_client")
                ], threshold=1),
                bt_ros.TryToPumpWallPuck(self.fifth_puck_zone),
                bt.ParallelWithMemoryNode([
                    bt.SequenceWithMemoryNode([
                        bt_ros.MoveLineToPoint(self.fifth_puck_zone + (side_sign * 0.05, -0.1, 0), "move_client"),
                    ]),
                    bt.SequenceWithMemoryNode([
                        bt_ros.CompleteTakeWallPuck("manipulator_client"),
                        bt.ActionNode(lambda: self.score_master.add("GREENIUM"))
                    ])
                ], threshold=2)
            ]),
            bt.SequenceWithMemoryNode([
                bt_ros.StopPump("manipulator_client"),
                bt_ros.MoveLineToPoint(self.fifth_puck_zone + (side_sign * 0.05, -0.1, 0), "move_client"),
                bt.ParallelWithMemoryNode([
                    bt_ros.MoveLineToPoint(self.fifth_puck_zone + (side_sign * 0.08, -0.1, 0), "move_client"),
                    bt_ros.SetToWall_ifReachedGoal(self.fifth_puck_zone + (side_sign * 0.07, -0.15, 0),
                                                   "manipulator_client")
                ], threshold=2)
            ])

        ])

        second_puck = bt.FallbackWithMemoryNode([
            bt.SequenceWithMemoryNode([
                bt_ros.StartPump("manipulator_client"),
                bt.ParallelNode([
                    bt_ros.MoveLineToPoint(self.second_puck, "move_client"),
                    bt_ros.CheckLimitSwitchInf("manipulator_client")
                ], threshold=1),
                bt_ros.TryToPumpWallPuck(self.second_puck),
                bt.ParallelWithMemoryNode([
                    bt.SequenceWithMemoryNode([
                        bt_ros.MoveLineToPoint(self.second_puck + (side_sign * -0.1, -0.5, 0), "move_client"),
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
                bt_ros.MoveLineToPoint(self.second_puck + (side_sign * -0.1, -0.5, 0), "move_client"),
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
                bt_ros.TryToPumpWallPuck(self.third_puck),
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
                bt_ros.TryToPumpWallPuck(self.forth_puck),
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
            ], threshold=2)
        ])

        fifth_puck = bt.FallbackWithMemoryNode([
            bt.SequenceWithMemoryNode([
                bt_ros.StartPump("manipulator_client"),
                bt.ParallelWithMemoryNode([
                    bt_ros.MoveLineToPoint(self.fifth_puck, "move_client"),
                    bt_ros.CheckLimitSwitchInf("manipulator_client")
                ], threshold=1),
                bt_ros.TryToPumpWallPuck(self.fifth_puck),
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
                ], threshold=2)

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
                # bt_ros.MoveLineToPoint(self.scales_zone + (0, -0.14, 0), "move_client"),
                bt.ParallelWithMemoryNode([
                    bt_ros.MoveLineToPoint(self.sixth_puck + (0, -0.08, 0), "move_client"),
                    bt_ros.SetToWall_ifReachedGoal(self.sixth_puck, "manipulator_client", threshold=0.15)
                ], threshold=2),
                bt_ros.StartPump("manipulator_client"),
                bt.ParallelWithMemoryNode([
                    bt_ros.MoveLineToPoint(self.sixth_puck, "move_client"),
                    bt_ros.CheckLimitSwitchInf("manipulator_client")
                ], threshold=1),
                bt_ros.TryToPumpWallPuckWithoutGrabber(self.sixth_puck),
                bt.ActionNode(lambda: self.score_master.add("REDIUM")),
                bt_ros.MoveLineToPoint(self.sixth_puck + (0, -0.05, 0), "move_client"),
                bt.ParallelWithMemoryNode([
                    bt_ros.MoveLineToPoint(self.redium_zone_first, "move_client"),
                    bt_ros.SetToGround_ifReachedGoal(self.sixth_puck + (side_sign * 0.2, -0.2, 0),
                                                     "manipulator_client"),
                    bt_ros.PublishScore_ifReachedGoal(self.redium_zone_first, self.score_master, "RED")
                ], threshold=3),
                bt_ros.ReleaseFromManipulator("manipulator_client"),
                bt.ParallelWithMemoryNode([
                    bt_ros.MoveLineToPoint(self.seventh_puck + (0, -0.05, 0), "move_client"),
                    bt_ros.SetManipulatortoUp("manipulator_client"),
                    bt_ros.SetToWall_ifReachedGoal(self.seventh_puck, "manipulator_client")
                ], threshold=3)
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
                bt_ros.TryToPumpWallPuckWithoutGrabber(self.seventh_puck),
                bt.ActionNode(lambda: self.score_master.add("REDIUM")),
                bt_ros.MoveLineToPoint(self.seventh_puck + (0, -0.05, 0), "move_client"),
                bt.ParallelWithMemoryNode([
                    bt_ros.MoveLineToPoint(self.redium_zone_second, "move_client"),
                    bt_ros.SetToGround_ifReachedGoal(self.seventh_puck + (side_sign * 0.2, -0.2, 0),
                                                     "manipulator_client"),
                    bt_ros.PublishScore_ifReachedGoal(self.redium_zone_second, self.score_master, "RED")
                ], threshold=3),
                bt_ros.ReleaseFromManipulator("manipulator_client"),
                bt.ParallelWithMemoryNode([
                    bt_ros.MoveLineToPoint(self.eighth_puck + (0, -0.05, 0), "move_client"),
                    bt_ros.SetManipulatortoUp("manipulator_client"),
                    bt_ros.SetToWall_ifReachedGoal(self.eighth_puck, "manipulator_client")
                ], threshold=3)
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
                bt_ros.TryToPumpWallPuckWithoutGrabber(self.eighth_puck),
                bt.ActionNode(lambda: self.score_master.add("REDIUM")),
                bt_ros.MoveLineToPoint(self.eighth_puck + (0, -0.05, 0), "move_client"),
                bt.ParallelWithMemoryNode([
                    bt_ros.MoveLineToPoint(self.redium_zone_third, "move_client"),
                    bt_ros.SetToGround_ifReachedGoal(self.eighth_puck + (side_sign * 0.2, -0.2, 0),
                                                     "manipulator_client"),
                    bt_ros.PublishScore_ifReachedGoal(self.redium_zone_third, self.score_master, "RED")
                ], threshold=3),
                bt_ros.ReleaseFromManipulator("manipulator_client"),
                bt.ParallelWithMemoryNode([
                    bt_ros.MoveLineToPoint(self.nineth_puck + (0, -0.05, 0), "move_client"),
                    bt_ros.SetManipulatortoUp("manipulator_client"),
                    bt_ros.SetToWall_ifReachedGoal(self.nineth_puck, "manipulator_client")
                ], threshold=3)
            ]),
            bt.SequenceWithMemoryNode([
                bt_ros.StopPump("manipulator_client"),
                bt_ros.SetManipulatortoUp("manipulator_client"),
                bt_ros.MoveLineToPoint(self.eighth_puck + (0, -0.05, 0), "move_client"),
                bt.ParallelWithMemoryNode([
                    bt_ros.MoveLineToPoint(self.eighth_puck + (side_sign * 0.2, -0.05, 0), "move_client"),
                    bt_ros.SetToWall_ifReachedGoal(self.eighth_puck + (side_sign * 0.2, -0.05, 0), "manipulator_client")
                ], threshold=2),
                bt_ros.MoveLineToPoint(self.nineth_puck + (0, -0.05, 0), "move_client"),
            ])
        ])

        nineth_puck = bt.SequenceWithMemoryNode([
            bt_ros.StartPump("manipulator_client"),
            bt.ParallelWithMemoryNode([
                bt_ros.MoveLineToPoint(self.nineth_puck, "move_client"),
                bt_ros.CheckLimitSwitchInf("manipulator_client")
            ], threshold=1),
            bt_ros.TryToPumpWallPuckWithoutGrabber(self.nineth_puck),
            bt.ActionNode(lambda: self.score_master.add("REDIUM")),
            bt.ParallelWithMemoryNode([
                bt_ros.MoveLineToPoint((0.25, 0.4, 1.57), "move_client"),
                bt_ros.SetToGround_ifReachedGoal(self.nineth_puck + (side_sign * 0.2, -0.2, 0), "manipulator_client"),
                bt_ros.PublishScore_ifReachedGoal(self.redium_zone_center, self.score_master, "RED")
            ], threshold=3),
            bt_ros.ReleaseFromManipulator("manipulator_client"),
            bt_ros.SetManipulatortoUp("manipulator_client"),
            bt.ActionNode(lambda: self.score_master.unload("RED"))
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

