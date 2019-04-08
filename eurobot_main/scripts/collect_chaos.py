#!/usr/bin/env python
import rospy
import behavior_tree as bt
import bt_ros
from std_msgs.msg import String
from bt_controller import SideStatus, BTController
from score_controller import ScoreController

import tf2_ros
from tf.transformations import euler_from_quaternion
from threading import Lock
from tactics_math import *
from core_functions import cvt_local2global
from visualization_msgs.msg import MarkerArray


class CollectChaosPucks(bt.FallbackNode):
    def __init__(self, move_client, manipulator_client):

        # Init parameters
        self.known_chaos_pucks = bt.BTVariable(np.array([]))
        self.is_observed = bt.BTVariable(False)

        self.sorted_chaos_landings = np.array([])
        self.known_chaos_pucks = np.array([])  # (x, y, id, r, g, b)

        self.active_goal = None
        self.goal_landing = None

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

        rospy.Subscriber("/pucks", MarkerArray, self.pucks_callback, queue_size=1)

    def pucks_callback(self, data):
        self.known_chaos_pucks.set(data)  # Action
        self.is_observed.set(True)



        if len(self.known_chaos_pucks) == 0:
            new_observation_pucks = [[marker.pose.position.x, marker.pose.position.y, marker.id, marker.color.r, marker.color.g, marker.color.b] for marker in data.markers]
            # [(0.95, 1.1, 3, 0, 0, 1), ...] - blue, id=3  IDs are not guaranteed to be the same from frame to frame
            print('TN -- new_observation_pucks')
            print(new_observation_pucks)

            try:
                self.known_chaos_pucks = np.array(new_observation_pucks)
                print("known")
                print(self.known_chaos_pucks)
            except Exception:  # FIXME
                print("list index out of range - no visible pucks on the field ")



        rospy.loginfo("Got pucks observation:")
        rospy.loginfo(self.known_chaos_pucks.get())

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

    def is_chaos_observed(self):  # Condition
        if self.is_observed.get():
            return bt.Status.SUCCESS
        else:
            return bt.Status.RUNNING






    while not self.update_coords():
        rospy.sleep(0.05)



        # fixme this
    def calculate_pucks_configuration(self, robot_coords, known_chaos_pucks, critical_angle):

        self.known_chaos_pucks = sort_wrt_robot(robot_coords, known_chaos_pucks)  # [(0.95, 1.1, 3, 0, 0, 1), ...]

        if len(self.known_chaos_pucks) >= 3:
            is_hull_safe_to_approach, coords_sorted_by_angle = sort_by_inner_angle_and_check_if_safe(robot_coords, known_chaos_pucks, critical_angle)

            if is_hull_safe_to_approach:  # only sharp angles
                print("hull is SAFE to approach, sorted wrt robot")

            if not is_hull_safe_to_approach:
                known_chaos_pucks = coords_sorted_by_angle  # calc vert-angle, sort by angle, return vertices (sorted)
                print("hull is not safe to approach, sorted by angle")

        # when we finally sorted them, chec if one of them is blue. If so, roll it
        if len(self.known_chaos_pucks) > 1 and all(self.known_chaos_pucks[0][3:6] == [0, 0, 1]):
            self.known_chaos_pucks = np.roll(self.known_chaos_pucks, -1, axis=0)  # so blue becomes last one to collect
            print("blue rolled")



    # fixme need to add self
    # fixme this
    def calculate_landings(self, robot_coords, coordinates, approach_vec, scale_factor, approach_dist):
        coords = coordinates[:, :2]
        if len(coords) == 1:
            landings = calculate_closest_landing_to_point(robot_coords, coords, approach_vec)  # Should be [(x, y, theta), ...]
        else:
            landings = unleash_power_of_geometry(coords, scale_factor, approach_dist)
        return landings





    self.goal_landing = self.sorted_chaos_landings[0]

    prelanding = cvt_local2global(self.drive_back_vec, self.goal_landing)
    self.active_goal = prelanding



    # calculate and move to first
    move_to_chaos = bt.SequenceWithMemoryNode([
                        bt.ActionNode(lambda: self.calculate_pucks_configuration(self.robot_coords, self.known_chaos_pucks, self.critical_angle)),
                        bt.ActionNode(lambda: self.calculate_landings(self.robot_coords, self.known_chaos_pucks, self.approach_vec, self.scale_factor, self.approach_dist)),
                        bt_ros.MoveLineToPoint(self.nearest_prelanding, "move_client"),
                        bt_ros.MoveLineToPoint(landings[0], "move_client"),
                    ]),

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
                                bt_ros.MoveLineToPoint(drive_back_point, "move_client"), # FIXME make it closer

                                # calc new landing
                                bt.ActionNode(self.calculate_pucks_configuration),
                                bt.ActionNode(self.calculate_landings),

                                # drive back, collect and move to new prelanding
                                bt.ParallelWithMemoryNode([
                                    bt_ros.CompleteCollectGround("manipulator_client")
                                    bt.ActionNode(lambda: self.score_master.add("BLUNIUM")),  # FIXME get color from camera
                                    bt_ros.MoveLineToPoint(self.nearest_prelanding, "move_client"),
                                ], threshold=3),

                                bt_ros.MoveLineToPoint(landings[0], "move_client"),
                            ]),
                        ]),
                        bt.ConditionNode(lambda: self.is_chaos_collected_completely1())
                    ])



    def update_coords(self):
        try:
            trans = self.tfBuffer.lookup_transform('map', self.robot_name, rospy.Time())
            q = [trans.transform.rotation.x,
                 trans.transform.rotation.y,
                 trans.transform.rotation.z,
                 trans.transform.rotation.w]
            angle = euler_from_quaternion(q)[2] % (2 * np.pi)

            self.robot_coords = np.array([trans.transform.translation.x, trans.transform.translation.y, angle])
            # rospy.loginfo("TN: Robot coords:\t" + str(self.robot_coords))
            return True
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as msg:
            rospy.logwarn(str(msg))
            return False


        # # Make BT
        # super(CollectChaosPucks, self).__init__([
        #     bt.ConditionNode(self.is_chaos_collected_completely),
        #
        #     bt.SequenceNode([
        #         bt.SequenceWithMemoryNode([
        #             bt.FallbackNode([
        #                 bt.ActionNode(self.get_chaos_observation_from_camera)  # FIXME
        #             ]),
        #             bt.ConditionNode(self.is_chaos_observed), # return RUNNING when not observed
        #
        #             bt.FallbackWithMemoryNode([
        #                 bt.ConditionNode(self.is_not_first_puck),
        #                 bt.SequenceWithMemoryNode([
        #                     bt.ActionNode(self.calculate_pucks_configuration),
        #                     bt.ActionNode(self.calculate_landings),
        #                     bt_ros.MoveLineToPoint(prelanding, "move_client"),  # approach_nearest_prelanding
        #                     bt_ros.MoveLineToPoint(landings[0], "move_client"),
        #                 ]),
        #
        #             ]),
        #
        #             bt_ros.StartCollectGround("manipulator_client"),  # self.start_suck()
        #
        #             bt.FallbackWithMemoryNode([
        #                 # bt.ConditionNode(self.is_safe_away_from_other_pucks_to_suck),
        #                 bt.SequenceNode([
        #                     bt_ros.MoveLineToPoint(drive_back_point, "move_client")  # self.drive_back()
        #                 ])
        #             ]),
        #
        #             bt.ParallelNode([
        #                 bt_ros.CompleteCollectGround("manipulator_client"),
        #
        #                 bt.FallbackWithMemoryNode([
        #                     bt.ConditionNode(self.is_last_puck),
        #                     bt.SequenceWithMemoryNode([
        #                         bt.ActionNode(self.calculate_pucks_configuration),
        #                         bt.ActionNode(self.calculate_landings)
        #                         bt_ros.MoveArcToPoint(prelanding, "move_client"),  # approach_nearest_prelanding
        #                         bt_ros.MoveLineToPoint(landings[0], "move_client"),  # approach_nearest_landing
        #                     ])
        #                 ])
        #             ], threshold=2),  # FIXME
        #         ]),
        #
        #         bt.ConditionNode(lambda: bt.Status.RUNNING),
        #     ])
        # ])
