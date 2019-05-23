#!/usr/bin/env python
import rospy
import eurobot_main.scripts.behavior_tree as bt
import eurobot_main.scripts.bt_ros
from std_msgs.msg import String
from eurobot_main.scripts.bt_controller import SideStatus, BTController
from eurobot_main.scripts.score_controller import ScoreController

import tf2_ros
from tf.transformations import euler_from_quaternion
from threading import Lock
from tactics_math import *
from core_functions import cvt_local2global
from visualization_msgs.msg import MarkerArray


class CollectChaos:
    def __init__(self, move_client, manipulator_client):
        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer)

        self.robot_name = rospy.get_param("robot_name")

        # Init parameters
        self.known_chaos_pucks = bt.BTVariable(np.array([])) # (x, y, id, r, g, b)
        self.sorted_chaos_landings = bt.BTVariable(np.array([]))
        self.is_observed = bt.BTVariable(False)

        self.nearest_landing = None
        self.nearest_PRElanding = None
        self.main_coords = None
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

        # # calculate and move to first
        # move_to_chaos = bt.SequenceWithMemoryNode([
        #                     bt.ActionNode(self.update_robot_coords),  # FIXME check with Alexey
        #                     bt.ActionNode(self.calculate_pucks_configuration),
        #                     bt.ActionNode(self.calculate_landings),
        #                     bt.ActionNode(self.choose_new_landing),
        #                     bt.ActionNode(self.calculate_prelanding),
        #
        #                     bt_ros.MoveLineToPoint(self.nearest_PRElanding, "move_client"),
        #                     bt_ros.MoveLineToPoint(self.nearest_landing, "move_client"),
        #                 ])
        #
        # collect_chaos = bt.SequenceWithMemoryNode([
        #                     bt.FallbackNode([
        #                         # completely?
        #                         bt.ConditionNode(self.is_chaos_collected_completely),
        #
        #                         # if last puck, just collect it, return success and get fuck out of here
        #                         bt.SequenceWithMemoryNode([
        #                             bt.ConditionNode(self.is_last_puck),
        #                             bt.SequenceWithMemoryNode([
        #                                 bt_ros.StartCollectGround("manipulator_client"),
        #                                 bt.ActionNode(lambda: self.score_master.add("BLUNIUM")),  # FIXME get color from camera
        #                                 bt_ros.CompleteCollectGround("manipulator_client")
        #                             ])
        #                         ]),
        #
        #                         # calc config and start collect
        #                         bt.SequenceWithMemoryNode([
        #                             bt_ros.StartCollectGround("manipulator_client"),
        #                             bt.ActionNode(self.calculate_drive_back_point),
        #                             bt_ros.MoveLineToPoint(self.drive_back_point, "move_client"), # FIXME make it closer
        #
        #                             # calc new landing
        #                             bt.ActionNode(self.calculate_pucks_configuration),
        #                             bt.ActionNode(self.calculate_landings),
        #
        #                             # drive back, collect and move to new prelanding
        #                             bt.ParallelWithMemoryNode([
        #                                 bt_ros.CompleteCollectGround("manipulator_client"),
        #                                 bt.ActionNode(lambda: self.score_master.add("BLUNIUM")),  # FIXME get color from camera
        #                                 bt_ros.MoveLineToPoint(self.nearest_PRElanding, "move_client"),
        #                             ], threshold=3),
        #
        #                             bt_ros.MoveLineToPoint(self.nearest_landing, "move_client"),
        #                         ]),
        #                     ]),
        #                     bt.ConditionNode(lambda: self.is_chaos_collected_completely1())
        #                 ])

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

    # def update_robot_coords(self):
    #     try:
    #         trans = self.tfBuffer.lookup_transform('map', self.robot_name, rospy.Time())
    #         q = [trans.transform.rotation.x,
    #              trans.transform.rotation.y,
    #              trans.transform.rotation.z,
    #              trans.transform.rotation.w]
    #         angle = euler_from_quaternion(q)[2] % (2 * np.pi)
    #
    #         self.robot_coords = np.array([trans.transform.translation.x, trans.transform.translation.y, angle])
    #         # rospy.loginfo("TN: Robot coords:\t" + str(self.robot_coords))
    #         # return True
    #     except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as msg:
    #         rospy.logwarn(str(msg))
    #         # return False


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
