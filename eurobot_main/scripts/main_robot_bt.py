#!/usr/bin/env python

import rospy
import numpy as np
import behavior_tree as bt
import bt_ros
import tf2_ros
from bt_controller import SideStatus, BTController
from std_msgs.msg import String

from tf.transformations import euler_from_quaternion
from core_functions import *
from tactics_math import *
from score_controller import ScoreController
from visualization_msgs.msg import MarkerArray
from shapely.geometry import Point
from shapely.geometry.polygon import Polygon


class MainRobotBT(object):
    # noinspection PyTypeChecker
    def __init__(self):
        self.move_publisher = rospy.Publisher("navigation/command", String, queue_size=100)
        self.manipulator_publisher = rospy.Publisher("manipulator/command", String, queue_size=100)
        self.stm_publisher = rospy.Publisher("stm/command", String, queue_size=100)

        self.move_client = bt_ros.ActionClient(self.move_publisher)
        self.manipulator_client = bt_ros.ActionClient(self.manipulator_publisher)
        self.stm_client = bt_ros.ActionClient(self.stm_publisher)

        self.side_status = None
        self.strategy = None

        self.bt = None
        self.bt_timer = None

        rospy.Subscriber("navigation/response", String, self.move_client.response_callback)
        rospy.Subscriber("manipulator/response", String, self.manipulator_client.response_callback)

    def start(self):

        self.bt = bt.Root(self.strategy.tree,
                          action_clients={"move_client": self.move_client,
                                          "manipulator_client": self.manipulator_client,
                                          "stm_client": self.stm_client})

        self.bt_timer = rospy.Timer(rospy.Duration(0.1), self.timer_callback)

    def change_side(self, side):
        self.side_status = side
        # self.strategy = Combobombo(self.side_status)
        self.strategy = SberStrategy(self.side_status)
        # self.strategy = OptimalStrategy(self.side_status)
        # self.strategy = BlindStrategy(self.side_status)

    def timer_callback(self, event):
        status = self.bt.tick()
        if status != bt.Status.RUNNING:
            self.bt_timer.shutdown()
        # print("============== BT LOG ================")
        self.bt.log(0)


class StrategyConfig(object):
    def __init__(self, side):

        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer)

        self.robot_name = rospy.get_param("robot_name")
        self.coords_threshold = rospy.get_param("coords_threshold")

        self.purple_chaos_center = rospy.get_param(self.robot_name + "/" + "purple_side" + "/chaos_center")
        self.yellow_chaos_center = rospy.get_param(self.robot_name + "/" + "yellow_side" + "/chaos_center")

        if side == SideStatus.PURPLE:
            self.color_side = "purple_side"
            self.opponent_side = "yellow_side"
            self.sign = -1
            self.our_chaos_center = self.purple_chaos_center
            self.opponent_chaos_center = self.yellow_chaos_center
        elif side == SideStatus.YELLOW:
            self.color_side = "yellow_side"
            self.opponent_side = "purple_side"
            self.sign = 1
            self.our_chaos_center = self.yellow_chaos_center
            self.opponent_chaos_center = self.purple_chaos_center

        self.VPAD = rospy.get_param("vertical_pucks_approach_dist")
        self.HPAD = np.array(rospy.get_param("horiz_pucks_approach_dist"))  # 0.127 meters, distance from robot to puck where robot will try to grab it
        self.delta = rospy.get_param("approach_delta")  # FIXME

        self.gnd_spacing = rospy.get_param("ground_spacing_dist")
        self.robot_outer_radius = rospy.get_param("robot_outer_radius")
        self.stick_len = rospy.get_param("stick_len")

        self.my_chaos_pucks = bt.BTVariable(np.array([]))  # (x, y, id, r, g, b)
        self.opponent_chaos_pucks = bt.BTVariable(np.array([]))  # (x, y, id, r, g, b)
        self.our_pucks_rgb = bt.BTVariable(np.array([]))  # (x, y, id, r, g, b)
        self.my_collected_chaos = bt.BTVariable(np.array([]))  # (x, y, id, r, g, b)

        self.incoming_puck_color = bt.BTVariable(None)
        self.collected_pucks = bt.BTVariable(np.array([]))

        self.is_main_robot_started = bt.BTVariable(False)
        self.is_our_chaos_observed_flag = bt.BTVariable(False)
        self.is_opponent_chaos_observed_flag = bt.BTVariable(False)

        self.is_secondary_responding = False

        self.secondary_coords = np.array([0, 0, 0])
        self.main_coords = None

        self.score_master = ScoreController(self.collected_pucks, self.robot_name)

        self.red_cell_puck = rospy.get_param(self.robot_name + "/" + self.color_side + "/red_cell_puck")
        self.blunium = rospy.get_param(self.robot_name + "/" + self.color_side + "/blunium")
        self.goldenium = rospy.get_param(self.robot_name + "/" + self.color_side + "/goldenium")
        self.scales_area = np.array(rospy.get_param(self.robot_name + "/" + self.color_side + "/scales_area"))
        self.chaos_radius = rospy.get_param("chaos_radius")

        self.my_chaos_area = rospy.get_param(self.robot_name + "/" + self.color_side + "/area_to_search_chaos")
        self.opponent_chaos_area = rospy.get_param(self.robot_name + "/" + self.opponent_side + "/area_to_search_chaos")

        self.purple_cells_area = rospy.get_param(self.robot_name + "/" + "purple_side" + "/purple_cells_area")
        self.yellow_cells_area = rospy.get_param(self.robot_name + "/" + "yellow_side" + "/yellow_cells_area")

        self.green_cell_puck = np.array([self.red_cell_puck[0], self.red_cell_puck[1] + self.gnd_spacing])
        self.blue_cell_puck = np.array([self.red_cell_puck[0], self.red_cell_puck[1] + 2 * self.gnd_spacing])

        self.first_puck_landing = np.array([self.red_cell_puck[0] + self.sign * self.HPAD - self.sign * self.delta,
                                            self.red_cell_puck[1],
                                            1.57 + self.sign * 1.57])  # y/p 3.14 / 0

        self.first_puck_landing_finish = np.array([self.red_cell_puck[0],
                                                    self.red_cell_puck[1] - 0.04,
                                                    1.57])

        self.second_puck_landing = np.array([self.red_cell_puck[0],
                                             self.red_cell_puck[1] + self.gnd_spacing - self.HPAD + self.delta,
                                             1.57])

        self.third_puck_landing = np.array([self.red_cell_puck[0],
                                            self.red_cell_puck[1] + 2 * self.gnd_spacing - self.HPAD + self.delta,
                                            1.57])

        self.third_puck_rotate_pose = np.array([self.our_chaos_center[0],
                                                self.our_chaos_center[1] - 0.3,
                                                -1.57 - self.sign * 0.785])  # y/p -2.35 / -0.78

        self.blunium_prepose = np.array([self.blunium[0] + self.sign * 0.07,
                                         self.blunium[1] + 0.35,
                                         -0.52])

        self.blunium_collect_PREpos = np.array([self.blunium[0],
                                                self.blunium[1] + 0.35,
                                                -1.57])

        self.blunium_collect_pos = np.array([self.blunium[0],
                                             self.blunium[1] + self.VPAD,  # 0.185,  # FIXME move 0.185 in params
                                             self.blunium_collect_PREpos[2]])

        self.blunium_collect_pos_side = np.array([self.blunium[0] + self.sign * 0.03,
                                                    self.blunium_collect_pos[1],
                                                    self.blunium_collect_PREpos[2]])

        self.blunium_start_push_pose = np.array([self.blunium_prepose[0],
                                                 self.blunium[1] + self.robot_outer_radius,
                                                 self.blunium_prepose[2]])

        self.blunium_end_push_pose = np.array([self.blunium_start_push_pose[0] - self.sign * 0.08,
                                               self.blunium_start_push_pose[1],
                                               self.blunium_start_push_pose[2]])

        self.blunium_get_back_pose = np.array([self.blunium_end_push_pose[0],
                                               self.blunium_end_push_pose[1] + 0.1,
                                               self.blunium_end_push_pose[2]])

        self.accelerator_PREunloading_pos = np.array([self.blunium_end_push_pose[0] - self.sign * 0.22,  # 0.22
                                                       self.blunium[1] + 0.13,
                                                       0.56])

        self.goldenium_1_PREgrab_pos = np.array([self.goldenium[0],
                                               self.goldenium[1] + 0.35,
                                               self.accelerator_PREunloading_pos[2]])

        self.goldenium_2_PREgrab_pos = np.array([self.goldenium[0],
                                               self.goldenium[1] + 0.28,
                                               -1.57])

        self.goldenium_grab_pos = np.array([self.goldenium[0],
                                            self.goldenium[1] + self.VPAD,  # 0.185
                                            self.goldenium_2_PREgrab_pos[2]])

        self.goldenium_back_pose = np.array([self.goldenium[0],
                                            self.goldenium_grab_pos[1] + 0.09,
                                            self.goldenium_2_PREgrab_pos[2]])

        self.goldenium_back_rot_pose = np.array([self.goldenium[0],
                                                 self.goldenium_back_pose[1],
                                                 1.57 - self.sign * 0.5])  # y/p 1.07 / 2.07

        self.scales_goldenium_PREpos = np.array([self.our_chaos_center[0] - self.sign * 0.08,
                                                 self.our_chaos_center[1] - 0.6,
                                                 1.57 - self.sign * 0.17])  # y/p 1.4 / 1.74

        self.scales_goldenium_pos = np.array([self.our_chaos_center[0] - self.sign * 0.29,  # 0.3
                                              self.our_chaos_center[1] + 0.37,
                                              1.57 + self.sign * 0.26])  # y/p 1.83 / 1.31

        self.scale_factor = np.array(rospy.get_param("scale_factor"))  # used in calculating outer bissectrisa for hull's angles
        self.critical_angle = rospy.get_param("critical_angle")
        self.approach_vec = np.array([-1 * self.HPAD, 0, 0])
        self.drive_back_dist = np.array(rospy.get_param("drive_back_dist"))  # FIXME
        self.drive_back_vec = np.array([-1*self.drive_back_dist, 0, 0])
        self.closest_landing = bt.BTVariable()
        self.nearest_PRElanding = bt.BTVariable()
        self.next_landing_var = bt.BTVariable()
        self.next_prelanding_var = bt.BTVariable()

        # self.guard_chaos_loc_var = bt.BTVariable(np.array([self.our_chaos_center[0] - self.sign * 0.3,
        #                                                    self.our_chaos_center[1] - 0.25,
        #                                                    1.57 - self.sign * 0.6]))  # FIXME change to another angle and loc * 0.785

        self.guard_chaos_loc_var = bt.BTVariable(np.array([1.35,
                                                           0.9,
                                                           1.57 - self.sign * 1.57]))  # FIXME change to another angle and loc * 0.785

        self.starting_pos_var = bt.BTVariable(np.array([1.5 + self.sign * 1.2,  # y/p 2.7 / 0.3
                                                        0.45,
                                                        1.57 + self.sign * 1.57]))  # y/p 3.14 / 0

        # TODO: add checking if all received coords lie inside chaos zone

        # TODO: pucks in front of starting cells are random, so while we aren't using camera
        #       will call them REDIUM  (it doesn't matter, because in this strategy we move them all to acc)
        #       It will matter in case big robot faces hard collision and need to unload pucks in starting cells

        self.pucks_subscriber = rospy.Subscriber("/pucks", MarkerArray, self.pucks_callback, queue_size=1)

    # works but only once
    # def pucks_callback(self, data):
    #     # [(0.95, 1.1, 3, 0, 0, 1), ...] - blue, id=3  IDs are not guaranteed to be the same from frame to frame
    #     # red (1, 0, 0)
    #     # green (0, 1, 0)
    #     # blue (0, 0, 1)
    #     # rospy.loginfo(data)
    #
    #     try:
    #         new_observation_pucks = [[marker.pose.position.x,
    #                                   marker.pose.position.y,
    #                                   marker.id,
    #                                   marker.color.r,
    #                                   marker.color.g,
    #                                   marker.color.b] for marker in data.markers]
    #
    #         purple_chaos_pucks, yellow_chaos_pucks, purple_pucks_rgb, yellow_pucks_rgb = self.parse_pucks(new_observation_pucks,
    #                                                                                                       self.purple_chaos_center,
    #                                                                                                       self.yellow_chaos_center,
    #                                                                                                       self.chaos_radius,
    #                                                                                                       self.purple_cells_area,
    #                                                                                                       self.yellow_cells_area)
    #
    #         if self.color_side == "purple_side":
    #             self.our_chaos_pucks.set(purple_chaos_pucks)
    #             self.opponent_chaos_pucks.set(yellow_chaos_pucks)
    #             self.our_pucks_rgb.set(purple_pucks_rgb)
    #         elif self.color_side == "yellow_side":
    #             self.our_chaos_pucks.set(yellow_chaos_pucks)
    #             self.opponent_chaos_pucks.set(purple_chaos_pucks)
    #             self.our_pucks_rgb.set(yellow_pucks_rgb)
    #
    #         if len(self.our_chaos_pucks.get()) == 4:
    #             self.is_observed_flag.set(True)
    #             rospy.loginfo("Got pucks observation:")
    #             rospy.loginfo(self.our_chaos_pucks.get())
    #
    #             self.pucks_subscriber.unregister()
    #
    #     except Exception:  # FIXME
    #         rospy.loginfo("list index out of range - no visible pucks on the field ")

    def pucks_callback(self, data):
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

            if self.is_main_robot_started.get() is False:

                purple_chaos_pucks, yellow_chaos_pucks, purple_pucks_rgb, yellow_pucks_rgb = initial_parse_pucks(new_observation_pucks,
                                                                                                                  self.purple_chaos_center,
                                                                                                                  self.yellow_chaos_center,
                                                                                                                  self.chaos_radius,
                                                                                                                  self.purple_cells_area,
                                                                                                                  self.yellow_cells_area)

                if self.color_side == "purple_side":
                    if len(purple_chaos_pucks) == 4:
                        self.my_chaos_pucks.set(purple_chaos_pucks)
                    if len(yellow_chaos_pucks) == 4:
                        self.opponent_chaos_pucks.set(yellow_chaos_pucks)

                    self.our_pucks_rgb.set(purple_pucks_rgb)

                elif self.color_side == "yellow_side":
                    if len(yellow_chaos_pucks) == 4:
                        self.my_chaos_pucks.set(yellow_chaos_pucks)
                    if len(purple_chaos_pucks) == 4:
                        self.opponent_chaos_pucks.set(purple_chaos_pucks)

                    self.our_pucks_rgb.set(yellow_pucks_rgb)

            if self.is_main_robot_started.get() is True:
                if len(self.my_chaos_pucks.get()) == 4:
                    self.is_our_chaos_observed_flag.set(True)
                    rospy.loginfo("Got pucks observation:")
                    rospy.loginfo(self.my_chaos_pucks.get())

                if len(self.opponent_chaos_pucks.get()) == 4:
                    self.is_opponent_chaos_observed_flag.set(True)
                    rospy.loginfo("Got pucks observation:")
                    rospy.loginfo(self.opponent_chaos_pucks.get())

                my_chaos_pucks, opp_chaos_pucks = self.compare_to_update_or_ignore(new_observation_pucks)

                self.my_chaos_pucks.set(my_chaos_pucks)
                self.opponent_chaos_pucks.set(opp_chaos_pucks)

        except Exception:  # FIXME
            rospy.loginfo("list index out of range - no visible pucks on the field ")

    def compare_to_update_or_ignore(self, new_observation):
        """
        Updates coordinates of the puck if camera sees it and is certain that it was moved

        In a new list first puck may be absent for at least three reasons:
        - it was collected by our robot
        - it is not visible (but it's still there) either because of robot in the line of view or light conditions
        - it was collected by opponent robot (and it's not there anymore)

        Updating procedure for my chaos:
        1) Parsing pucks to chaos collections
        if we see every puck left in our chaos than simply update all
        if we see less than calculated, than update only green and blue.
        If we see only one red, it's hard to say which particularly it is, so update red pucks only if both are observed

        :param new_observation: [(x, y, id, r, g, b), ...]
        :return: [(x, y, id, r, g, b), ...]
        """

        my_chaos_new = self.my_chaos_pucks.get().copy()
        opp_chaos_new = self.opponent_chaos_pucks.get().copy()

        observed_my_chaos_collection = []
        observed_opponent_chaos_collection = []
        other_pucks = []
        ref_colors = ['BLUNIUM', 'GREENIUM', 'REDIUM', 'REDIUM']
        colors_of_my_observed_chaos = []
        colors_of_opp_observed = []
        colors_of_my_collected_chaos = []

        my_chaos_area = Polygon([self.my_chaos_area[0],
                                 self.my_chaos_area[1],
                                 self.my_chaos_area[2],
                                 self.my_chaos_area[3]])

        opponent_chaos_area = Polygon([self.opponent_chaos_area[0],
                                       self.opponent_chaos_area[1],
                                       self.opponent_chaos_area[2],
                                       self.opponent_chaos_area[3]])

        for puck in self.my_collected_chaos.get():
            colors_of_my_collected_chaos.append(get_color(puck))
            print colors_of_my_collected_chaos

        for puck in new_observation:
            unknown_puck = Point(puck[0], puck[1])
            if unknown_puck.within(my_chaos_area):
                observed_my_chaos_collection.append(puck)
            elif unknown_puck.within(opponent_chaos_area):
                observed_opponent_chaos_collection.append(puck)
            else:
                other_pucks.append(puck)

        if len(observed_my_chaos_collection) == len(self.my_chaos_pucks.get()) - len(self.my_collected_chaos.get()):
            my_chaos_new = observed_my_chaos_collection
        else:
            my_chaos_new = self.parse_by_color(observed_my_chaos_collection, my_chaos_new)

        comparison = colors_of_my_observed_chaos[:]
        comparison.extend(colors_of_my_collected_chaos)
        print "colors_of_my_observed_chaos", sorted(colors_of_my_observed_chaos)

        if sorted(colors_of_my_observed_chaos) == ref_colors:
            print "equal"
            my_chaos_new = observed_my_chaos_collection
        else:
            my_chaos_new = self.parse_by_color(observed_my_chaos_collection, my_chaos_new)
            print "need to parse!"

        # if len(observed_opponent_chaos_collection) == 4:  # TODO or if we started collecting it?
        #     opp_chaos_new = observed_opponent_chaos_collection
        # else:
        #     opp_chaos_new = self.parse_by_color(observed_opponent_chaos_collection, opp_chaos_new)

        my_chaos_new = np.array(my_chaos_new)
        opp_chaos_new = np.array(opp_chaos_new)

        return my_chaos_new, opp_chaos_new

    @staticmethod
    def parse_by_color(new_obs, known):

        known = known.tolist()
        colors_in_new_obs = []
        colors_in_known = []
        list_of_pucks = []

        for puck in new_obs:
            puck_color = get_color(puck)
            colors_in_new_obs.append(puck_color)

        for puck in known:
            puck_color = get_color(puck)
            colors_in_known.append(puck_color)

        if colors_in_new_obs.count("BLUNIUM") == 1:
            ind = colors_in_new_obs.index("BLUNIUM")
            colors_in_new_obs.remove("BLUNIUM")
            list_of_pucks.append(new_obs.pop(ind))
            try:
                ind_old = colors_in_known.index("BLUNIUM")
                colors_in_known.remove("BLUNIUM")
                del known[ind_old]
            except ValueError as Error:
                print Error

        if colors_in_new_obs.count("GREENIUM") == 1:
            ind = colors_in_new_obs.index("GREENIUM")
            colors_in_new_obs.remove("GREENIUM")
            list_of_pucks.append(new_obs.pop(ind))
            try:
                ind_old = colors_in_known.index("GREENIUM")
                colors_in_known.remove("GREENIUM")
                del known[ind_old]
            except ValueError as Error:
                print Error

        if colors_in_new_obs.count('REDIUM') == 2:
            print "TWO REDIUMS, add them!"
            ind = colors_in_new_obs.index('REDIUM')
            colors_in_new_obs.remove('REDIUM')
            list_of_pucks.append(new_obs.pop(ind))
            try:
                ind_old = colors_in_known.index('REDIUM')
                colors_in_known.remove('REDIUM')
                print "old after deleting", colors_in_known
                del known[ind_old]
            except ValueError as Error:
                print Error

            ind = colors_in_new_obs.index('REDIUM')
            colors_in_new_obs.remove('REDIUM')
            list_of_pucks.append(new_obs.pop(ind))
            try:
                ind_old = colors_in_known.index('REDIUM')
                colors_in_known.remove('REDIUM')
                del known[ind_old]
            except ValueError as Error:
                print Error

            print "known"
            print known
            list_of_pucks.extend(known)

        elif colors_in_new_obs.count("REDIUM") < 2:
            # get coords of red in known and append them
            list_of_pucks.extend(known)
        return list_of_pucks

    # def is_chaos_available(self, side):
    #
    #     pass

    # def get_yolo_observation(self):
    #
    #     return

    def is_observed(self):
        # rospy.loginfo("is observed?")
        if self.is_our_chaos_observed_flag.get():
            # rospy.loginfo('YES! Got all pucks coords')
            return bt.Status.SUCCESS
        else:
            rospy.loginfo('Still waiting for the cam, known: ' + str(len(self.my_chaos_pucks.get())))
            return bt.Status.FAILED

    # FIXME move to math
    def calculate_next_landing(self, puck):
        """
        calculates closest landing to point wrt to current robot position
        :param point: [x,y]
        :return: [xl,yl,thetal]
        """
        while not self.update_main_coords():
            print "no coords available"
            rospy.sleep(0.5)

        dist, _ = calculate_distance(self.main_coords, puck)  # return deltaX and deltaY coords
        gamma = np.arctan2(dist[1], dist[0])
        puck = np.hstack((puck, gamma))
        landing = cvt_local2global(self.approach_vec, puck)
        self.next_landing_var.set(landing)
        prelanding = cvt_local2global(self.drive_back_vec, self.next_landing_var.get())
        self.next_prelanding_var.set(prelanding)
        rospy.loginfo("calculated next landing AND prelanding!")
        rospy.loginfo(self.next_landing_var.get())
        rospy.loginfo(self.next_prelanding_var.get())

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
        self.is_secondary_responding = self.update_secondary_coords()
        rospy.loginfo("Checking if scales are available to approach...")

        area = self.scales_area
        robot = self.secondary_coords  # can be 0, 0, 0  or  some value

        point = Point(robot[0], robot[1])
        polygon = Polygon([area[0], area[1], area[2], area[3]])

        if not self.is_secondary_responding:
            rospy.sleep(0)
            return bt.Status.SUCCESS
        else:
            if polygon.contains(point):
                rospy.loginfo('Landing busy')
                return bt.Status.RUNNING
            else:
                rospy.loginfo('Landing is free to go')
                return bt.Status.SUCCESS

    def update_chaos_pucks(self):
        """
        TODO: make it usable when collecting opponent's chaos

        delete taken puck from known on the field
        get color of last taken puck
        :return: None
        """
        incoming_puck_color = get_color(self.my_chaos_pucks.get()[0])
        self.incoming_puck_color.set(incoming_puck_color)
        rospy.loginfo("incoming_puck_color: " + str(self.incoming_puck_color.get()))
        self.my_collected_chaos.set(self.my_chaos_pucks.get()[0])
        self.my_chaos_pucks.set(np.delete(self.my_chaos_pucks.get(), 0, axis=0))
        rospy.loginfo("Known pucks after removing: " + str(self.my_chaos_pucks.get()))

    def calculate_pucks_configuration(self):
        """

        :return: # [(0.95, 1.1, 3, 0, 0, 1), ...]
        """
        while not self.update_main_coords():
            print "no coords available"
            rospy.sleep(0.5)

        known_chaos_pucks = sort_wrt_robot(self.main_coords, self.my_chaos_pucks.get())
        print "sorted"
        self.my_chaos_pucks.set(known_chaos_pucks)
        if len(self.my_chaos_pucks.get()) >= 3:
            is_hull_safe_to_approach, coords_sorted_by_angle = sort_by_inner_angle_and_check_if_safe(self.main_coords,
                                                                                                     self.my_chaos_pucks.get(),
                                                                                                     self.critical_angle)

            if not is_hull_safe_to_approach:
                self.my_chaos_pucks.set(coords_sorted_by_angle)  # calc vert-angle, sort by angle, return vertices (sorted)
                rospy.loginfo("hull is not safe to approach, sorted by angle")
            else:  # only sharp angles
                rospy.loginfo("hull is SAFE to approach, keep already sorted wrt robot")

        if len(self.my_chaos_pucks.get()) == 2:
            known_chaos_pucks = list(known_chaos_pucks)
            known_chaos_pucks.sort(key=lambda t: t[1])
            rospy.loginfo("rolled when two left __----------------______---------------------- _----___-___--__---_-_--_")
            known_chaos_pucks = np.array(known_chaos_pucks)
            
            self.my_chaos_pucks.set(known_chaos_pucks)
        rospy.loginfo("Known pucks sorted: " + str(self.my_chaos_pucks.get()))

    #     when we finally sorted them, chec if one of them is blue. If so, roll it so blue becomes last one to collect
    #     if self.known_chaos_pucks.get().size > 1 and all(self.known_chaos_pucks.get()[0][3:6] == [0, 0, 1]):
    #         # self.known_chaos_pucks.set(np.roll(self.known_chaos_pucks.get(), -1, axis=0))
    #         rospy.loginfo("blue rolled")

    def calculate_closest_landing(self):
        """
        Don't sort here, sort in calculate_config!

        :return: [(x, y, theta), ...]
        """
        if len(self.my_chaos_pucks.get()) == 1:
            landings = calculate_closest_landing_to_point(self.main_coords,
                                                          self.my_chaos_pucks.get()[:, :2],
                                                          self.approach_vec)

        else:
            landings = unleash_power_of_geometry(self.my_chaos_pucks.get()[:, :2],
                                                 self.scale_factor,
                                                 self.HPAD)

        self.closest_landing.set(landings[0])
        rospy.loginfo("Inside calculate_closest_landing, closest_landing is : ")
        print(self.closest_landing.get())
        print " "

    def calculate_prelanding(self):
        nearest_PRElanding = cvt_local2global(self.drive_back_vec, self.closest_landing.get())
        self.nearest_PRElanding.set(nearest_PRElanding)
        rospy.loginfo("Nearest PRElanding calculated: " + str(self.nearest_PRElanding.get()))
        print " "

    def update_robot_status(self):
        self.is_main_robot_started.set(True)
        rospy.loginfo('main_robot_started')
        return bt.Status.SUCCESS

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
            rospy.loginfo("main coords: " + str(self.main_coords))
            return True  # return True
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as msg:
            rospy.logwarn(str(msg))
            return False  # return False

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


class SberStrategy(StrategyConfig):
    def __init__(self, side):
        super(SberStrategy, self).__init__(side)

        move_immidiately_to_chaos = bt.SequenceWithMemoryNode([
                                        bt_ros.MoveToVariable(self.guard_chaos_loc_var, "move_client")

        ])

        red_cell_puck = bt.SequenceWithMemoryNode([
                            bt_ros.MoveLineToPoint(self.first_puck_landing, "move_client"),
                            bt.FallbackWithMemoryNode([
                                bt.SequenceWithMemoryNode([
                                    bt_ros.BlindStartCollectGround("manipulator_client"),
                                    bt.ActionNode(lambda: self.score_master.add("REDIUM")),  # FIXME: color is undetermined without camera!
                                    bt.ParallelWithMemoryNode([
                                        bt_ros.CompleteCollectGround("manipulator_client"),
                                        bt_ros.MoveToVariable(self.guard_chaos_loc_var, "move_client"),

                                        # bt.SequenceWithMemoryNode([
                                        #     bt.ActionNode(self.calculate_pucks_configuration),
                                        #     bt.ActionNode(self.calculate_closest_landing),
                                        #     bt.ActionNode(self.calculate_prelanding),
                                        #     bt_ros.MoveToVariable(self.nearest_PRElanding, "move_client"),
                                        #     bt_ros.ArcMoveToVariable(self.closest_landing, "move_client"),
                                        # ])
                                    ], threshold=2),
                                ]),
                                bt.ParallelWithMemoryNode([
                                    bt_ros.SetManipulatortoWall("manipulator_client"),
                                    bt_ros.MoveToVariable(self.guard_chaos_loc_var, "move_client")
                                ], threshold=2)
                            ]),
                        ])

        collect_chaos = bt.SequenceWithMemoryNode([
                    # 1st, but done in finishing red
                    bt.ActionNode(self.calculate_pucks_configuration),
                    bt.ActionNode(self.calculate_closest_landing),
                    bt.ActionNode(self.calculate_prelanding),

                    bt_ros.MoveToVariable(self.nearest_PRElanding, "move_client"),
                    bt_ros.ArcMoveToVariable(self.closest_landing, "move_client"),
                    bt_ros.BlindStartCollectGround("manipulator_client"),
                    bt.ActionNode(self.update_chaos_pucks),
                    bt.ActionNode(lambda: self.score_master.add(self.incoming_puck_color.get())),
                    bt_ros.MoveToVariable(self.nearest_PRElanding, "move_client"),

                    # 2nd
                    bt.ActionNode(self.calculate_pucks_configuration),
                    bt.ActionNode(self.calculate_closest_landing),
                    bt.ActionNode(self.calculate_prelanding),

                    bt.ParallelWithMemoryNode([
                        bt_ros.CompleteCollectGround("manipulator_client"),
                        bt.SequenceWithMemoryNode([
                            bt_ros.ArcMoveToVariable(self.nearest_PRElanding, "move_client"),
                            bt_ros.MoveToVariable(self.closest_landing, "move_client"),
                        ])
                    ], threshold=2),

                    bt_ros.BlindStartCollectGround("manipulator_client"),
                    bt.ActionNode(self.update_chaos_pucks),
                    bt.ActionNode(lambda: self.score_master.add(self.incoming_puck_color.get())),
                    bt_ros.MoveToVariable(self.nearest_PRElanding, "move_client"),

                    # 3rd
                    bt.ActionNode(self.calculate_pucks_configuration),
                    bt.ActionNode(self.calculate_closest_landing),
                    bt.ActionNode(self.calculate_prelanding),

                    bt.ParallelWithMemoryNode([
                        bt_ros.CompleteCollectGround("manipulator_client"),
                        bt.SequenceWithMemoryNode([
                            bt_ros.ArcMoveToVariable(self.nearest_PRElanding, "move_client"),
                            bt_ros.MoveToVariable(self.closest_landing, "move_client"),
                        ])
                    ], threshold=2),

                    bt_ros.BlindStartCollectGround("manipulator_client"),
                    bt.ActionNode(self.update_chaos_pucks),
                    bt.ActionNode(lambda: self.score_master.add(self.incoming_puck_color.get())),
                    bt_ros.MoveToVariable(self.nearest_PRElanding, "move_client"),

                    # 4th
                    bt.ActionNode(self.calculate_pucks_configuration),
                    bt.ActionNode(self.calculate_closest_landing),

                    bt.ParallelWithMemoryNode([
                        bt_ros.CompleteCollectGround("manipulator_client"),
                        bt_ros.MoveToVariable(self.closest_landing, "move_client"),
                    ], threshold=2),

                    bt_ros.BlindStartCollectGround("manipulator_client"),
                    bt.ActionNode(self.update_chaos_pucks),
                    bt.ActionNode(lambda: self.score_master.add(self.incoming_puck_color.get()))  # COMAAAAAA

                    # only for testing and unloading at home
                    # bt_ros.CompleteCollectGround("manipulator_client"),
                    # bt_ros.StepperUp("manipulator_client")
        ])

        green_cell_puck_after_chaos = bt.SequenceWithMemoryNode([
                                        bt.ParallelWithMemoryNode([
                                            bt_ros.CompleteCollectGround("manipulator_client"),
                                            bt.SequenceWithMemoryNode([
                                                bt.ActionNode(lambda: self.calculate_next_landing(self.green_cell_puck)),
                                                bt_ros.MoveToVariable(self.next_prelanding_var, "move_client"),
                                                bt_ros.MoveToVariable(self.next_landing_var, "move_client")
                                            ])
                                        ], threshold=2),

                                        bt.FallbackWithMemoryNode([
                                            bt.SequenceWithMemoryNode([
                                                bt.ActionNode(lambda: self.calculate_next_landing(self.blue_cell_puck)),
                                                bt_ros.BlindStartCollectGround("manipulator_client"),
                                                bt.ActionNode(lambda: self.score_master.add("REDIUM")),  # FIXME: color is undetermined without camera!
                                                bt.ParallelWithMemoryNode([
                                                    bt_ros.CompleteCollectGround("manipulator_client"),
                                                    bt.SequenceWithMemoryNode([
                                                        bt_ros.MoveToVariable(self.next_prelanding_var, "move_client"),
                                                        bt_ros.MoveToVariable(self.next_landing_var, "move_client")
                                                    ])
                                                ], threshold=2),
                                            ]),
                                            bt.ParallelWithMemoryNode([
                                                bt_ros.SetManipulatortoWall("manipulator_client"),
                                                bt_ros.MoveToVariable(self.next_landing_var, "move_client"),  # FIXME 
                                            ], threshold=2)
                                        ])
                                    ])

        # when not collectin blunium but pushing it in the end
        blue_cell_puck = bt.SequenceWithMemoryNode([
                            bt.FallbackWithMemoryNode([
                                bt.SequenceWithMemoryNode([
                                    bt_ros.BlindStartCollectGround("manipulator_client"),
                                    bt.ActionNode(lambda: self.score_master.add("GREENIUM")),  # FIXME: color is undetermined without camera!
                                    bt.ParallelWithMemoryNode([
                                        bt_ros.CompleteCollectGroundWhenFull("manipulator_client"),
                                        bt_ros.MoveLineToPoint(self.blunium_collect_PREpos, "move_client"),
                                        # bt_ros.MoveLineToPoint(self.blunium_get_back_pose, "move_client"),  # FIXME try Arc
                                    ], threshold=2),
                                ]),
                                bt.ParallelWithMemoryNode([
                                    bt_ros.SetManipulatortoUp("manipulator_client"),  # FIXME when adding chaos
                                    bt_ros.MoveLineToPoint(self.blunium_collect_PREpos, "move_client"),
                                    # bt_ros.MoveLineToPoint(self.blunium_get_back_pose, "move_client"),  # FIXME try Arc
                                ], threshold=2)  # if fail, FIXME never happens now
                            ])
                        ])

        # when robot is NOT FULL (inside 6)
        # move_and_collect_blunium = bt.SequenceWithMemoryNode([
        #                                 # bt_ros.MoveLineToPoint(self.tactics.blunium_collect_PREpos, "move_client"),
        #                                 bt_ros.StartCollectBlunium("manipulator_client"),
        #                                 bt.ParallelWithMemoryNode([
        #                                     bt_ros.MoveLineToPoint(self.blunium_collect_pos, "move_client"),
        #                                     bt_ros.CheckLimitSwitchInfLong("manipulator_client")
        #                                 ], threshold=1),  # CheckLimitSwitchInf
        #                                 # bt_ros.MoveLineToPoint(self.blunium_collect_pos_side, "move_client"),
        #                                 bt_ros.MoveLineToPoint(self.blunium_collect_pos + np.array([0, 0.04, 0]), "move_client"),  # FIXME
        #                                 bt_ros.FinishCollectBlunium("manipulator_client"),
        #                                 bt.ActionNode(lambda: self.score_master.add("BLUNIUM")),  # COMA!!!!!!
        #                                 bt_ros.MainSetManipulatortoGround("manipulator_client")
        #                             ])

        # when full (inside 7)
        move_and_collect_blunium = bt.SequenceWithMemoryNode([
                                        bt_ros.StartCollectBlunium("manipulator_client"),
                                        bt.ParallelWithMemoryNode([
                                            bt_ros.MoveLineToPoint(self.blunium_collect_pos, "move_client"),
                                            bt_ros.CheckLimitSwitchInfLong("manipulator_client")
                                        ], threshold=1),  # CheckLimitSwitchInf
                                        # bt_ros.MoveLineToPoint(self.blunium_collect_pos_side, "move_client"),
                                        bt_ros.MoveLineToPoint(self.blunium_collect_pos + np.array([0, 0.06, 0]), "move_client"),  # FIXME
                                        bt_ros.MainSetManipulatortoGround("manipulator_client")  # Here changed!!!!!!!!!!!!
                                        #bt_ros.FinishCollectBluniumWhenFull("manipulator_client"),
                                        # bt.ActionNode(lambda: self.score_master.add("BLUNIUM")),
                                        # bt_ros.MainSetManipulatortoGround("manipulator_client")
                                    ])

        approach_acc = bt.SequenceWithMemoryNode([
                            # bt_ros.MoveLineToPoint(self.third_puck_landing, "move_client"),
                            bt_ros.MoveLineToPoint(self.blunium_get_back_pose, "move_client"),
                            bt_ros.MoveLineToPoint(self.accelerator_PREunloading_pos, "move_client"),  # FIXME try Arc
                            bt_ros.SetSpeedSTM([-0.05, -0.1, 0], 0.9, "stm_client")  # [0, -0.1, 0]
                        ])

        # # when not full
        # collect_unload_first_in_acc = bt.SequenceWithMemoryNode([
        #                             bt_ros.StepperUp("manipulator_client"),  # FIXME do we need to do that? NO if all 7 pucks inside
        #                             bt_ros.UnloadAccelerator("manipulator_client"),
        #                             bt.ActionNode(lambda: self.score_master.unload("ACC")),
        #                             bt.ActionNode(lambda: self.score_master.reward("UNLOCK_GOLDENIUM_BONUS"))
        #                         ])

        # when full - don't move stepper up
        collect_unload_first_in_acc = bt.SequenceWithMemoryNode([
                                    # bt_ros.StepperUp("manipulator_client"),  # FIXME do we need to do that? NO if all 7 pucks inside
                                    bt_ros.UnloadAccelerator("manipulator_client"),
                                    bt.ActionNode(lambda: self.score_master.unload("ACC")),
                                    bt.ActionNode(lambda: self.score_master.reward("UNLOCK_GOLDENIUM_BONUS")),  # COMAAAAAA

                                    bt_ros.FinishCollectBluniumWhenFull("manipulator_client"),
                                    bt.ActionNode(lambda: self.score_master.add("BLUNIUM"))
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

        # move_and_push_blunium = bt.SequenceWithMemoryNode([
        #                             bt_ros.MoveLineToPoint(self.blunium_prepose, "move_client"),
        #                             bt.ParallelWithMemoryNode([
        #                                 bt_ros.MainSetManipulatortoGround("manipulator_client"),  # FIXME when adding chaos
        #                                 bt_ros.MoveLineToPoint(self.blunium_start_push_pose, "move_client"),
        #                             ], threshold=2),
        #                             bt_ros.MoveLineToPoint(self.blunium_end_push_pose, "move_client"),
        #                             bt.ActionNode(lambda: self.score_master.add("BLUNIUM")),
        #                             bt.ActionNode(lambda: self.score_master.unload("ACC"))
        #                         ])

        collect_goldenium = bt.SequenceWithMemoryNode([
                                bt_ros.Delay500("manipulator_client"),
                                # bt_ros.MoveLineToPoint(self.tactics.goldenium_1_PREgrab_pos, "move_client"),
                                bt_ros.MoveLineToPoint(self.goldenium_2_PREgrab_pos, "move_client"),
                                bt_ros.StartCollectGoldenium("manipulator_client"),

                                bt.ParallelWithMemoryNode([
                                    bt_ros.MoveLineToPoint(self.goldenium_grab_pos, "move_client"),
                                    bt_ros.CheckLimitSwitchInfLong("manipulator_client")
                                ], threshold=1)
                            ])

        move_to_goldenium_prepose = bt.SequenceWithMemoryNode([
                                        bt_ros.MoveLineToPoint(self.goldenium_back_pose, "move_client"),
                                        bt_ros.GrabGoldeniumAndHoldUp("manipulator_client"),
                                        bt.ActionNode(lambda: self.score_master.add("GOLDENIUM")),
                                        bt.ActionNode(lambda: self.score_master.reward("GRAB_GOLDENIUM_BONUS")),

                                        # bt_ros.MoveLineToPoint(self.tactics.goldenium_back_rot_pose, "move_client"),
                                        bt_ros.MoveLineToPoint(self.scales_goldenium_PREpos, "move_client")
                                    ])

        unload_goldenium = bt.SequenceWithMemoryNode([
                                bt.ConditionNode(self.is_scales_landing_free),
                                bt.SequenceWithMemoryNode([
                                    bt_ros.MoveLineToPoint(self.scales_goldenium_pos + np.array([0, -0.01, 0]), "move_client"),
                                    bt.ActionNode(lambda: self.score_master.unload("SCALES")),
                                    bt_ros.SetManipulatortoWall("manipulator_client"),
                                    bt_ros.GrabGoldeniumAndHoldUp("manipulator_client"),
                                    bt_ros.SetManipulatortoWall("manipulator_client"),
                                    bt_ros.GrabGoldeniumAndHoldUp("manipulator_client"),
                                    bt_ros.MoveLineToPoint(self.scales_goldenium_pos, "move_client"),
                                    bt_ros.UnloadGoldenium("manipulator_client"),
                                    bt_ros.SetManipulatortoUp("manipulator_client")
                                ])
                            ])

        self.tree = bt.SequenceWithMemoryNode([
                        bt.ActionNode(self.update_robot_status),
                        move_immidiately_to_chaos,
                        # red_cell_puck,

                        bt.FallbackWithMemoryNode([
                            bt.SequenceNode([
                                bt.ConditionNode(self.is_observed),
                                collect_chaos
                            ]),
                            bt.ConditionNode(lambda: bt.Status.RUNNING)  # infinitely waiting for camera
                        ]),

                        green_cell_puck_after_chaos,
                        blue_cell_puck,
                        move_and_collect_blunium,
                        approach_acc,
                        collect_unload_first_in_acc,  
                        unload_acc,
                        # move_and_push_blunium,
                        collect_goldenium,
                        move_to_goldenium_prepose,
                        unload_goldenium
                    ])



if __name__ == '__main__':
    try:
        rospy.init_node("main_robot_BT")
        main_robot_bt = MainRobotBT()
        bt_controller = BTController(main_robot_bt)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
