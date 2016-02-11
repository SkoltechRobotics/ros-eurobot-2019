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
import threading


# class PucksSlave(object):
#     def __init__(self):
#         self.pucks = None
#         self.mutex = threading.Lock()
#         rospy.Subscriber("/pucks", MarkerArray, self.pucks_callback, queue_size=1)

#     def pucks_callback(self, data):
#         # rospy.loginfo("pucks callback, get")
#         # rospy.loginfo(data)
#         with self.mutex:
#             self.pucks = data

#     def get_pucks(self):
#         with self.mutex:
#             return self.pucks


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
        self.strategy_number = 0

        self.bt = None
        self.bt_timer = None

        # self.pucks_slave = PucksSlave()

        rospy.Subscriber("navigation/response", String, self.move_client.response_callback)
        rospy.Subscriber("manipulator/response", String, self.manipulator_client.response_callback)

    # noinspection PyTypeChecker
    def start(self):

        self.bt = bt.Root(self.strategy.tree,
                          action_clients={"move_client": self.move_client,
                                          "manipulator_client": self.manipulator_client,
                                          "stm_client": self.stm_client})

        self.bt_timer = rospy.Timer(rospy.Duration(0.1), self.timer_callback)

    def change_side(self, side):

        self.side_status = side
        self.sberstr = SberStrategy(self.side_status)  # , self.pucks_slave # FIXME: add second tree from SuddenBlind
        # self.suddenblind = SuddenBlind(self.side_status)  # , self.pucks_slave
        # self.testfield = TestField(self.side_status)  # , self.pucks_slave

        self.strategy = self.sberstr

    def change_strategy(self, num):
        self.strategy_number = num
        if num == 0:
            print("CHANGE STRATEGY TO " + str(num))
            self.strategy = self.sberstr
        elif num == 1:
            print("CHANGE STRATEGY TO " + str(num))
            # self.strategy = self.suddenblind
        elif num == 2:
            print("CHANGE STRATEGY TO " + str(num))
            # self.strategy = self.testfield

    def timer_callback(self, event):
        status = self.bt.tick()
        if status != bt.Status.RUNNING:
            self.bt_timer.shutdown()
        # print("============== BT LOG ================")
        self.bt.log(0)


class StrategyConfig(object):
    def __init__(self, side):  # pucks_slave
        # self.pucks_slave = pucks_slave

        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer)

        self.robot_name = rospy.get_param("robot_name")

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
        self.our_pucks_rgb = bt.BTVariable(np.array([[0, 0, 0, 0, 0, 0],
                                                    [0, 0, 1, 0, 0, 0],
                                                    [0, 0, 2, 0, 0, 0]]))  # (x, y, id, r, g, b)

        self.my_collected_chaos = bt.BTVariable(np.array([]))  # (x, y, id, r, g, b)
        self.opp_chaos_collected_me = bt.BTVariable(np.array([]))  # (x, y, id, r, g, b)

        self.visible_pucks_on_field = bt.BTVariable(np.array([]))  # (x, y, id, r, g, b)

        self.incoming_puck_color = bt.BTVariable(None)
        self.collected_pucks = bt.BTVariable(np.array([]))

        self.is_main_robot_started = bt.BTVariable(False)
        self.is_observed_flag = bt.BTVariable(False)  # FIXME!!!!!!!! remove
        self.is_our_chaos_observed_flag = bt.BTVariable(False)
        self.is_opponent_chaos_observed_flag = bt.BTVariable(False)
        self.is_lost_puck_present_flag = bt.BTVariable(False)

        self.is_secondary_responding = False

        self.secondary_coords = np.array([0, 0, 0])
        self.main_coords = None

        self.score_master = ScoreController(self.collected_pucks, self.robot_name)

        self.red_cell_puck = rospy.get_param(self.robot_name + "/" + self.color_side + "/red_cell_puck")
        self.blunium = rospy.get_param(self.robot_name + "/" + self.color_side + "/blunium")
        self.goldenium = rospy.get_param(self.robot_name + "/" + self.color_side + "/goldenium")
        self.scales_area = np.array(rospy.get_param(self.robot_name + "/" + self.color_side + "/scales_area"))
        self.chaos_radius = rospy.get_param("chaos_radius")

        self.final_search_area = rospy.get_param("final_search_area")
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

        self.blind_chaos_pose = np.array([self.our_chaos_center[0] - self.sign * 0.228,
                                          self.our_chaos_center[1],
                                          1.57 + self.sign * 1.07])  # y/p  /0.5

        self.blunium_prepose = np.array([self.blunium[0] + self.sign * 0.07,
                                         self.blunium[1] + 0.35,
                                         -0.52])

        self.blunium_collect_PREpos = np.array([self.blunium[0],
                                                self.blunium[1] + 0.35,
                                                -1.57])

        self.blunium_collect_pos = np.array([self.blunium[0],
                                             self.blunium[1] + self.VPAD,
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

        self.blunium_nose_start_push_pose = np.array([self.blunium[0] + self.sign * 0.14,  # 0.11
                                                     self.blunium[1] + self.robot_outer_radius - 0.03,
                                                     0.56 - self.sign * 0.07])  # y/p  /0.63 or 0.56 both

        self.blunium_nose_end_push_pose = np.array([self.blunium_end_push_pose[0] - self.sign * 0.22,  # 0.22
                                                   self.blunium[1] + self.robot_outer_radius - 0.03,  # self.blunium[1] + 0.13
                                                   self.blunium_nose_start_push_pose[2]])

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

        self.unload_goldenium_on_blue = np.array([1.5 + self.sign * 1.3,  # y/p 2.7 / 0.3
                                            0.9,
                                            -1.57 + self.sign * 0.785])  # y/p -0.78 / -2.35

        self.scale_factor = np.array(rospy.get_param("scale_factor"))  # used in calculating outer bissectrisa for hull's angles
        self.critical_angle = rospy.get_param("critical_angle")
        self.approach_vec = np.array([-1 * self.HPAD, 0, 0])
        self.drive_back_dist = np.array(rospy.get_param("drive_back_dist"))  # FIXME
        self.drive_back_vec = np.array([-1*self.drive_back_dist, 0, 0])
        self.closest_landing = bt.BTVariable()
        self.nearest_PRElanding = bt.BTVariable()
        self.next_landing_var = bt.BTVariable()
        self.next_prelanding_var = bt.BTVariable()
        self.move_back_pose = bt.BTVariable()
        # self.guard_chaos_loc = bt.BTVariable(np.array([self.our_chaos_center[0] - self.sign * 0.3,
        #                                                    self.our_chaos_center[1] - 0.25,
        #                                                    1.57 - self.sign * 0.6]))  # FIXME change to another angle and loc * 0.785

        self.red_puck_guard_chaos_prepose = np.array([1.5 + self.sign * 0.27,
                                                        0.7,
                                                        1.57 + self.sign * 1.57])

        self.guard_chaos_loc = np.array([1.5 + self.sign * 0.27,
                                        0.93,
                                        1.57 + self.sign * 1.57])

        self.guard_chaos_rotate = np.array([self.guard_chaos_loc[0],
                                            self.guard_chaos_loc[1],
                                            1.57 - self.sign * 0.785])  # y/p  0.78/2.35

        self.blind_guard_chaos_finish = np.array([self.guard_chaos_loc[0] + self.sign * 0.23,
                                                    self.guard_chaos_loc[1] + 0.15,
                                                    self.guard_chaos_rotate[2]])

        self.starting_pos = np.array([1.5 + self.sign * 1.2,  # y/p 2.7 / 0.3
                                    0.45,
                                    1.57 + self.sign * 1.57])  # y/p 3.14 / 0

        rospy.Subscriber("/pucks", MarkerArray, self.pucks_callback, queue_size=1)

        # TODO: pucks in front of starting cells are random, so while we aren't using camera
        #       will call them REDIUM  (it doesn't matter, because in this strategy we move them all to acc)
        #       It will matter in case big robot faces hard collision and need to unload pucks in starting cells

    def pucks_callback(self):  # parse_pucks
        # data = self.pucks_slave.get_pucks()
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
                        self.is_our_chaos_observed_flag.set(True)
                        # rospy.loginfo("Got MY pucks observation:")
                        # print self.my_chaos_pucks.get()
                        # print " "
                    if len(yellow_chaos_pucks) == 4:
                        self.opponent_chaos_pucks.set(yellow_chaos_pucks)
                        self.is_opponent_chaos_observed_flag.set(True)
                        # rospy.loginfo("Got OPP pucks observation:")
                        # print self.opponent_chaos_pucks.get()
                        # print " "

                    self.our_pucks_rgb.set(purple_pucks_rgb)

                elif self.color_side == "yellow_side":
                    if len(yellow_chaos_pucks) == 4:
                        self.my_chaos_pucks.set(yellow_chaos_pucks)
                        self.is_our_chaos_observed_flag.set(True)
                        # rospy.loginfo("Got MY pucks observation:")
                        # print self.my_chaos_pucks.get()
                        # print " "
                    if len(purple_chaos_pucks) == 4:
                        self.opponent_chaos_pucks.set(purple_chaos_pucks)
                        self.is_opponent_chaos_observed_flag.set(True)
                        # rospy.loginfo("Got OPP pucks observation:")
                        # print self.opponent_chaos_pucks.get()
                        # print " "

                    self.our_pucks_rgb.set(yellow_pucks_rgb)

            if self.is_main_robot_started.get() is True:
                lost_pucks = yolo_parse_pucks(new_observation_pucks,
                                               self.final_search_area)
                self.visible_pucks_on_field.set(lost_pucks)

            """
            if self.is_main_robot_started.get() is True:
                my_chaos_pucks, opp_chaos_pucks = self.compare_to_update_or_ignore(new_observation_pucks)
                self.my_chaos_pucks.set(my_chaos_pucks)
                self.opponent_chaos_pucks.set(opp_chaos_pucks)
            """
            
            rospy.loginfo("Got MY pucks observation:")
            print self.my_chaos_pucks.get()
            print " "

            rospy.loginfo("Got OPP pucks observation:")
            print self.opponent_chaos_pucks.get()
            print " "
            

        except Exception:  # FIXME
            rospy.loginfo("list index out of range - no visible pucks on the field ")

    def is_my_chaos_observed(self):
        # rospy.loginfo("is observed?")
        if self.is_our_chaos_observed_flag.get():
            # rospy.loginfo('YES! Got all pucks coords')
            return bt.Status.SUCCESS
        elif not self.is_our_chaos_observed_flag.get() and not self.is_main_robot_started.get():
            rospy.loginfo('Still waiting for the cam, known: ' + str(len(self.my_chaos_pucks.get())))
            return bt.Status.RUNNING
        elif not self.is_our_chaos_observed_flag.get() and self.is_main_robot_started.get():
            return bt.Status.FAILED

    def is_opp_chaos_observed(self):
        # # rospy.loginfo("is observed?")
        # if self.is_opponent_chaos_observed_flag.get():
        #     # rospy.loginfo('YES! Got all pucks coords')
        #     return bt.Status.SUCCESS
        # else:
        #     rospy.loginfo('Still waiting for the cam, known: ' + str(len(self.my_chaos_pucks.get())))
        #     return bt.Status.FAILED

        if self.is_opponent_chaos_observed_flag.get():
            # rospy.loginfo('YES! Got all pucks coords')
            return bt.Status.SUCCESS
        elif not self.is_opponent_chaos_observed_flag.get() and not self.is_main_robot_started.get():
            rospy.loginfo('Still waiting for the cam, known: ' + str(len(self.my_chaos_pucks.get())))
            return bt.Status.RUNNING
        elif not self.is_opponent_chaos_observed_flag.get() and self.is_main_robot_started.get():
            return bt.Status.FAILED

    def is_lost_puck_present(self):
        rospy.loginfo("lost pucks")
        rospy.loginfo(self.visible_pucks_on_field.get())
        if len(self.visible_pucks_on_field.get()) > 0:
            rospy.loginfo('There are missing pucks on table')
            return bt.Status.SUCCESS
        else:
            rospy.loginfo('no pucks found')
            return bt.Status.FAILED

    # FIXME move to math
    def calculate_next_landing(self, puck):
        """
        calculates closest landing to point wrt to current robot position
        :param puck:
        :return: [xl,yl,thetal]
        """
        while not self.update_main_coords():
            print "no coords available"
            rospy.sleep(0.5)

        puck = puck[:2]
        dist, _ = calculate_distance(self.main_coords, puck)  # return deltaX and deltaY coords
        gamma = np.arctan2(dist[1], dist[0])
        puck = np.hstack((puck, gamma))
        landing = cvt_local2global(self.approach_vec, puck)
        self.next_landing_var.set(landing)
        rospy.loginfo("calculated next landing")
        rospy.loginfo(self.next_landing_var.get())

    def calculate_next_prelanding(self):
        prelanding = cvt_local2global(self.drive_back_vec, self.next_landing_var.get())
        self.next_prelanding_var.set(prelanding)
        rospy.loginfo("calculated next prelanding!")
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

    def update_chaos_pucks(self, side="my"):
        """
        TODO: make it usable when collecting opponent's chaos

        delete taken puck from known on the field
        get color of last taken puck
        :return: None
        """
        if side == "my":
            incoming_puck_color = get_color(self.my_chaos_pucks.get()[0])
            self.incoming_puck_color.set(incoming_puck_color)
            rospy.loginfo("incoming_puck_color: " + str(self.incoming_puck_color.get()))
            self.my_collected_chaos.set(self.my_chaos_pucks.get()[0])
            self.my_chaos_pucks.set(np.delete(self.my_chaos_pucks.get(), 0, axis=0))
            rospy.loginfo("Our known pucks after removing: " + str(self.my_chaos_pucks.get()))

        if side == "opponent":
            incoming_puck_color = get_color(self.opponent_chaos_pucks.get()[0])
            self.incoming_puck_color.set(incoming_puck_color)
            rospy.loginfo("incoming_puck_color: " + str(self.incoming_puck_color.get()))
            self.opp_chaos_collected_me.set(self.opponent_chaos_pucks.get()[0])
            self.opponent_chaos_pucks.set(np.delete(self.opponent_chaos_pucks.get(), 0, axis=0))
            rospy.loginfo("Opponent's known pucks after removing: " + str(self.opponent_chaos_pucks.get()))

    def remove_uncollected_chaos_puck(self):
        self.my_chaos_pucks.set(np.delete(self.my_chaos_pucks.get(), 0, axis=0))
        rospy.loginfo("Our known pucks after removing: " + str(self.my_chaos_pucks.get()))

    def calculate_pucks_configuration(self):
        """

        :return: # [(0.95, 1.1, 3, 0, 0, 1), ...]
        """
        self.parse_pucks()
        while not self.update_main_coords():
            print "no coords available"
            rospy.sleep(0.5)

        my_known_chaos_pucks = sort_wrt_robot(self.main_coords, self.my_chaos_pucks.get())
        opp_known_chaos_pucks = sort_wrt_robot(self.main_coords, self.opponent_chaos_pucks.get())
        self.my_chaos_pucks.set(my_known_chaos_pucks)
        self.opponent_chaos_pucks.set(opp_known_chaos_pucks)

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
            my_known_chaos_pucks = list(my_known_chaos_pucks)  # FIXME .tolist ?
            my_known_chaos_pucks.sort(key=lambda t: t[1])
            rospy.loginfo("rolled when two left")
            my_known_chaos_pucks = np.array(my_known_chaos_pucks)
            self.my_chaos_pucks.set(my_known_chaos_pucks)

        # # when we finally sorted them, chec if one of them is blue. If so, roll it so blue becomes last one to collect
        # if self.known_chaos_pucks.get().size > 1 and all(self.known_chaos_pucks.get()[0][3:6] == [0, 0, 1]):
        #     # self.known_chaos_pucks.set(np.roll(self.known_chaos_pucks.get(), -1, axis=0))
        #     rospy.loginfo("blue rolled")

        rospy.loginfo("Known pucks sorted: ")
        print self.my_chaos_pucks.get()
        print " "

    def set_closest_chaos_landing(self):
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

    def set_prelanding_to_chaos_landing(self):
        nearest_PRElanding = cvt_local2global(self.drive_back_vec, self.closest_landing.get())
        self.nearest_PRElanding.set(nearest_PRElanding)
        rospy.loginfo("Nearest PRElanding calculated: " + str(self.nearest_PRElanding.get()))
        print " "

    def set_move_back_pose(self):
        self.move_back_pose.set(self.nearest_PRElanding.get())

    def update_robot_status(self):
        self.is_main_robot_started.set(True)
        rospy.loginfo('main_robot_started')

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


class SuddenBlind(StrategyConfig):
    def __init__(self, side):  # pucks_slave
        super(SuddenBlind, self).__init__(side)  # pucks_slave

        collect_red_cell_puck = bt.SequenceWithMemoryNode([
                                    bt_ros.MoveLineToPoint(self.first_puck_landing, "move_client"),
                                    bt.FallbackWithMemoryNode([
                                        bt.SequenceWithMemoryNode([
                                            bt_ros.StartCollectGroundCheck("manipulator_client"),
                                            bt.ActionNode(lambda: self.score_master.add(get_color(self.our_pucks_rgb.get()[0]))),
                                            bt.ParallelWithMemoryNode([
                                                bt_ros.CompleteCollectGround("manipulator_client"),
                                                bt.SequenceWithMemoryNode([
                                                    bt_ros.MoveLineToPoint(self.guard_chaos_loc, "move_client"),
                                                    bt_ros.MoveLineToPoint(self.guard_chaos_rotate, "move_client"),
                                                    bt_ros.MoveLineToPoint(self.blind_guard_chaos_finish, "move_client"),
                                                ])
                                            ], threshold=2),
                                        ]),
                                        bt.ParallelWithMemoryNode([
                                            bt_ros.StopPump("manipulator_client"),
                                            bt_ros.SetManipulatortoUp("manipulator_client"),
                                            bt.SequenceWithMemoryNode([
                                                bt_ros.MoveLineToPoint(self.guard_chaos_loc, "move_client"),
                                                bt_ros.MoveLineToPoint(self.guard_chaos_rotate, "move_client"),
                                                bt_ros.MoveLineToPoint(self.blind_guard_chaos_finish, "move_client")
                                            ])
                                        ], threshold=3),
                                    ]),
                                ])

        blind_move_chaos_center_collect = bt.SequenceWithMemoryNode([

                                            bt.ActionNode(lambda: self.calculate_next_landing(self.green_cell_puck)),
                                            bt.ActionNode(self.calculate_next_prelanding),

                                            bt.FallbackWithMemoryNode([
                                                bt.SequenceWithMemoryNode([
                                                    bt_ros.StartCollectGroundCheck("manipulator_client"),
                                                    bt.ActionNode(lambda: self.score_master.add("UNDEFINED")),

                                                    bt.ParallelWithMemoryNode([
                                                        bt.SequenceWithMemoryNode([
                                                            bt_ros.CompleteCollectGround("manipulator_client"),
                                                            bt_ros.SetManipulatortoUp("manipulator_client")
                                                        ]),
                                                        bt.SequenceWithMemoryNode([
                                                            bt_ros.MoveToVariable(self.next_prelanding_var, "move_client"),
                                                            bt_ros.MoveToVariable(self.next_landing_var, "move_client"),
                                                        ])
                                                    ], threshold=2),
                                                ]),
                                                bt.ParallelWithMemoryNode([
                                                    bt.SequenceWithMemoryNode([
                                                        bt_ros.StopPump("manipulator_client"),
                                                        bt_ros.SetManipulatortoUp("manipulator_client")
                                                    ]),
                                                    bt.SequenceWithMemoryNode([
                                                        bt_ros.MoveToVariable(self.next_prelanding_var, "move_client"),
                                                        bt_ros.MoveToVariable(self.next_landing_var, "move_client"),
                                                    ])
                                                ], threshold=2)
                                            ])
                                        ])

        collect_green_cell_puck = bt.SequenceWithMemoryNode([
                                    bt.FallbackWithMemoryNode([
                                        bt.SequenceWithMemoryNode([
                                            bt_ros.StartCollectGroundCheck("manipulator_client"),
                                            bt.ActionNode(lambda: self.score_master.add(get_color(self.our_pucks_rgb.get()[1]))),
                                            bt.ParallelWithMemoryNode([
                                                bt_ros.CompleteCollectGround("manipulator_client"),
                                                bt_ros.MoveLineToPoint(self.blunium_nose_start_push_pose, "move_client")
                                            ], threshold=2),
                                        ]),

                                        bt.ParallelWithMemoryNode([
                                            bt_ros.StopPump("manipulator_client"),
                                            bt_ros.MoveLineToPoint(self.blunium_nose_start_push_pose, "move_client")
                                        ], threshold=2),
                                    ]),
                                ])

        collect_green_cell_puck2 = bt.SequenceWithMemoryNode([
                                        bt.FallbackWithMemoryNode([
                                            bt.SequenceWithMemoryNode([
                                                bt_ros.StartCollectGroundCheck("manipulator_client"),
                                                bt.ActionNode(lambda: self.score_master.add(get_color(self.our_pucks_rgb.get()[1]))),
                                                bt.ParallelWithMemoryNode([
                                                    bt_ros.CompleteCollectGround("manipulator_client"),
                                                    bt_ros.MoveLineToPoint(self.blunium_nose_start_push_pose, "move_client")
                                                ], threshold=2),
                                            ]),

                                            bt.ParallelWithMemoryNode([
                                                bt_ros.StopPump("manipulator_client"),
                                                bt_ros.MoveLineToPoint(self.blunium_nose_start_push_pose, "move_client")
                                            ], threshold=2),
                                        ]),
                                    ])

        move_guard_our_chaos_while_taking_red = bt.SequenceWithMemoryNode([

                                                    bt.ParallelWithMemoryNode([
                                                        bt.SequenceWithMemoryNode([
                                                            bt_ros.MoveLineToPoint(self.red_puck_guard_chaos_prepose, "move_client"),
                                                            bt_ros.MoveLineToPoint(self.guard_chaos_loc, "move_client"),
                                                            bt_ros.MoveLineToPoint(self.guard_chaos_rotate, "move_client")
                                                        ]),
                                                        bt.FallbackWithMemoryNode([
                                                            bt.SequenceWithMemoryNode([
                                                                bt_ros.DelayStartCollectGroundCheck("manipulator_client"),
                                                                bt.ActionNode(lambda: self.score_master.add(get_color(self.our_pucks_rgb.get()[0]))),
                                                                bt_ros.CompleteCollectGround("manipulator_client"),
                                                            ]),
                                                            bt.SequenceWithMemoryNode([
                                                                bt_ros.StopPump("manipulator_client"),
                                                                bt_ros.SetManipulatortoUp("manipulator_client")
                                                            ])
                                                        ])
                                                    ], threshold=2),
    ])

        ## maybe working
        move_to_opp_chaos_while_taking_red = bt.SequenceWithMemoryNode([
                                                bt.ActionNode(self.calculate_pucks_configuration),
                                                bt.ActionNode(lambda: self.calculate_next_landing(self.opponent_chaos_pucks.get()[0])),

                                                bt.ParallelWithMemoryNode([
                                                    bt_ros.MoveToVariable(self.next_landing_var, "move_client"),

                                                    bt.FallbackWithMemoryNode([
                                                        bt.SequenceWithMemoryNode([
                                                            bt_ros.DelayStartCollectGroundCheck("manipulator_client"),
                                                            bt.ActionNode(lambda: self.score_master.add(get_color(self.our_pucks_rgb.get()[0]))),
                                                            bt_ros.CompleteCollectGround("manipulator_client"),
                                                        ]),
                                                        bt.SequenceWithMemoryNode([
                                                            bt_ros.StopPump("manipulator_client"),
                                                            bt_ros.SetManipulatortoUp("manipulator_client")
                                                        ])
                                                    ])
                                                ], threshold=2),

                                                bt.SequenceWithMemoryNode([
                                                    bt.ActionNode(self.calculate_pucks_configuration),
                                                    bt.ActionNode(self.set_closest_chaos_landing),
                                                    bt.ActionNode(self.set_prelanding_to_chaos_landing),

                                                    bt.FallbackWithMemoryNode([
                                                        bt.SequenceWithMemoryNode([
                                                            bt_ros.StartCollectGroundCheck("manipulator_client"),
                                                            bt.ActionNode(lambda: self.update_chaos_pucks(side="opponent")),
                                                            bt.ActionNode(lambda: self.score_master.add(self.incoming_puck_color.get())),
                                                            bt.ParallelWithMemoryNode([
                                                                bt_ros.CompleteCollectGround("manipulator_client"),
                                                                bt.SequenceWithMemoryNode([
                                                                    bt_ros.MoveToVariable(self.nearest_PRElanding, "move_client"),
                                                                    bt_ros.MoveToVariable(self.closest_landing, "move_client")
                                                                ])
                                                            ], threshold=2),
                                                        ]),
                                                        bt.ParallelWithMemoryNode([
                                                            bt_ros.StopPump("manipulator_client"),
                                                            bt_ros.SetManipulatortoUp("manipulator_client"),
                                                            bt.SequenceWithMemoryNode([
                                                                bt_ros.MoveToVariable(self.nearest_PRElanding, "move_client"),
                                                                bt_ros.MoveToVariable(self.closest_landing, "move_client")
                                                            ])
                                                        ], threshold=2)
                                                    ])
                                                ])
                                            ])

        move_to_opp_chaos_while_taking_red_collision = bt.ParallelWithMemoryNode([
                                                            bt_ros.DelayCollision("manipulator_client"),

                                                            bt.SequenceWithMemoryNode([
                                                                bt.ActionNode(self.calculate_pucks_configuration),
                                                                bt.ActionNode(lambda: self.calculate_next_landing(self.opponent_chaos_pucks.get()[0])),

                                                                bt.ParallelWithMemoryNode([
                                                                    bt_ros.MoveToVariable(self.next_landing_var, "move_client"),

                                                                    bt.FallbackWithMemoryNode([
                                                                        bt.SequenceWithMemoryNode([
                                                                            bt_ros.DelayStartCollectGroundCheck("manipulator_client"),
                                                                            bt.ActionNode(lambda: self.score_master.add(get_color(self.our_pucks_rgb.get()[0]))),
                                                                            bt_ros.CompleteCollectGround("manipulator_client"),
                                                                        ]),
                                                                        bt.SequenceWithMemoryNode([
                                                                            bt_ros.StopPump("manipulator_client"),
                                                                            bt_ros.SetManipulatortoUp("manipulator_client")
                                                                        ])
                                                                    ])
                                                                ], threshold=2),

                                                                bt.SequenceWithMemoryNode([
                                                                    bt.ActionNode(self.calculate_pucks_configuration),
                                                                    bt.ActionNode(self.set_closest_chaos_landing),
                                                                    bt.ActionNode(self.set_prelanding_to_chaos_landing),

                                                                    bt.FallbackWithMemoryNode([
                                                                        bt.SequenceWithMemoryNode([
                                                                            bt_ros.StartCollectGroundCheck("manipulator_client"),
                                                                            bt.ActionNode(lambda: self.update_chaos_pucks(side="opponent")),
                                                                            bt.ActionNode(lambda: self.score_master.add(self.incoming_puck_color.get())),
                                                                            bt_ros.CompleteCollectGround("manipulator_client"),
                                                                        ]),
                                                                        bt.ParallelWithMemoryNode([
                                                                            bt_ros.StopPump("manipulator_client"),
                                                                            bt_ros.SetManipulatortoUp("manipulator_client"),
                                                                        ], threshold=2)
                                                                    ])
                                                                ])
                                                            ])
                                                        ], threshold=1)

        collect_chaos = bt.SequenceWithMemoryNode([

                            # bt.ActionNode(self.calculate_pucks_configuration),
                            # bt.ActionNode(self.set_closest_chaos_landing),
                            # bt.ActionNode(self.set_prelanding_to_chaos_landing),
                            # bt_ros.MoveToVariable(self.nearest_PRElanding, "move_client"),
                            # bt_ros.MoveToVariable(self.closest_landing, "move_client"),

                            # # We here arrived to 1st chaos puck and ready to start collecting
                            bt.FallbackWithMemoryNode([
                                bt.SequenceWithMemoryNode([
                                    bt_ros.StartCollectGroundCheck("manipulator_client"),
                                    bt.ActionNode(self.update_chaos_pucks),
                                    bt.ActionNode(lambda: self.score_master.add(self.incoming_puck_color.get())),
                                    bt.ActionNode(self.set_move_back_pose),
                                    bt_ros.MoveToVariable(self.move_back_pose, "move_client"),  # to calculated move_back_pose

                                    bt.ActionNode(self.calculate_pucks_configuration),
                                    bt.ActionNode(self.set_closest_chaos_landing),
                                    bt.ActionNode(self.set_prelanding_to_chaos_landing),

                                    bt.ParallelWithMemoryNode([
                                        bt_ros.CompleteCollectGround("manipulator_client"),
                                        bt.SequenceWithMemoryNode([
                                            bt_ros.MoveToVariable(self.nearest_PRElanding, "move_client"),
                                            bt_ros.MoveToVariable(self.closest_landing, "move_client")
                                        ])
                                    ], threshold=2),
                                ]),
                                # if we failed to pump puck
                                bt.SequenceWithMemoryNode([
                                    bt_ros.StopPump("manipulator_client"),
                                    bt.ActionNode(self.set_move_back_pose),
                                    bt_ros.MoveToVariable(self.move_back_pose, "move_client"),  # to calculated move_back_pose
                                    bt.ActionNode(self.remove_uncollected_chaos_puck),
                                    bt.ActionNode(self.calculate_pucks_configuration),
                                    bt.ActionNode(self.set_closest_chaos_landing),
                                    bt.ActionNode(self.set_prelanding_to_chaos_landing),

                                    bt.ParallelWithMemoryNode([
                                        bt_ros.SetManipulatortoWall("manipulator_client"),
                                        bt.SequenceWithMemoryNode([
                                            bt_ros.MoveToVariable(self.nearest_PRElanding, "move_client"),
                                            bt_ros.MoveToVariable(self.closest_landing, "move_client")
                                        ])
                                    ], threshold=2)
                                ])
                            ]),


                            # # We here arrived to 2nd chaos puck and ready to start collecting
                            bt.FallbackWithMemoryNode([
                                bt.SequenceWithMemoryNode([
                                    bt_ros.StartCollectGroundCheck("manipulator_client"),
                                    bt.ActionNode(self.update_chaos_pucks),
                                    bt.ActionNode(lambda: self.score_master.add(self.incoming_puck_color.get())),
                                    bt.ActionNode(self.set_move_back_pose),
                                    bt_ros.MoveToVariable(self.move_back_pose, "move_client"),  # to calculated move_back_pose

                                    bt.ActionNode(self.calculate_pucks_configuration),
                                    bt.ActionNode(self.set_closest_chaos_landing),
                                    bt.ActionNode(self.set_prelanding_to_chaos_landing),

                                    bt.ParallelWithMemoryNode([
                                        bt_ros.CompleteCollectGround("manipulator_client"),
                                        bt.SequenceWithMemoryNode([
                                            bt_ros.MoveToVariable(self.nearest_PRElanding, "move_client"),
                                            bt_ros.MoveToVariable(self.closest_landing, "move_client")
                                        ])
                                    ], threshold=2),
                                ]),
                                # if we failed to pump puck
                                bt.SequenceWithMemoryNode([
                                    bt_ros.StopPump("manipulator_client"),
                                    bt.ActionNode(self.set_move_back_pose),
                                    bt_ros.MoveToVariable(self.move_back_pose, "move_client"),  # to calculated move_back_pose
                                    bt.ActionNode(self.remove_uncollected_chaos_puck),
                                    bt.ActionNode(self.calculate_pucks_configuration),
                                    bt.ActionNode(self.set_closest_chaos_landing),
                                    bt.ActionNode(self.set_prelanding_to_chaos_landing),

                                    bt.ParallelWithMemoryNode([
                                        bt_ros.SetManipulatortoWall("manipulator_client"),
                                        bt.SequenceWithMemoryNode([
                                            bt_ros.MoveToVariable(self.nearest_PRElanding, "move_client"),
                                            bt_ros.MoveToVariable(self.closest_landing, "move_client")
                                        ])
                                    ], threshold=2)
                                ])
                            ]),

                            # # We here arrived to 3rd chaos puck and ready to start collecting
                            bt.FallbackWithMemoryNode([
                                bt.SequenceWithMemoryNode([
                                    bt_ros.StartCollectGroundCheck("manipulator_client"),
                                    bt.ActionNode(self.update_chaos_pucks),
                                    bt.ActionNode(lambda: self.score_master.add(self.incoming_puck_color.get())),
                                    bt.ActionNode(self.set_move_back_pose),
                                    bt_ros.MoveToVariable(self.move_back_pose, "move_client"),  # to calculated move_back_pose

                                    bt.ActionNode(self.calculate_pucks_configuration),
                                    bt.ActionNode(self.set_closest_chaos_landing),
                                    bt.ActionNode(self.set_prelanding_to_chaos_landing),

                                    bt.ParallelWithMemoryNode([
                                        bt_ros.CompleteCollectGround("manipulator_client"),
                                        bt.SequenceWithMemoryNode([
                                            bt_ros.MoveToVariable(self.nearest_PRElanding, "move_client"),
                                            bt_ros.MoveToVariable(self.closest_landing, "move_client")
                                        ])
                                    ], threshold=2),
                                ]),
                                # if we failed to pump puck
                                bt.SequenceWithMemoryNode([
                                    bt_ros.StopPump("manipulator_client"),
                                    bt.ActionNode(self.set_move_back_pose),
                                    bt_ros.MoveToVariable(self.move_back_pose, "move_client"),  # to calculated move_back_pose
                                    bt.ActionNode(self.remove_uncollected_chaos_puck),
                                    bt.ActionNode(self.calculate_pucks_configuration),
                                    bt.ActionNode(self.set_closest_chaos_landing),
                                    bt.ActionNode(self.set_prelanding_to_chaos_landing),

                                    bt.ParallelWithMemoryNode([
                                        bt_ros.SetManipulatortoWall("manipulator_client"),
                                        bt.SequenceWithMemoryNode([
                                            bt_ros.MoveToVariable(self.nearest_PRElanding, "move_client"),
                                            bt_ros.MoveToVariable(self.closest_landing, "move_client")
                                        ])
                                    ], threshold=2)
                                ])
                            ]),

                            # # We here arrived to 4th chaos puck and ready to start collecting
                            bt.FallbackWithMemoryNode([
                                bt.SequenceWithMemoryNode([
                                    bt_ros.StartCollectGroundCheck("manipulator_client"),
                                    bt.ActionNode(self.update_chaos_pucks),
                                    bt.ActionNode(lambda: self.score_master.add(self.incoming_puck_color.get())),

                                    bt.ParallelWithMemoryNode([
                                        bt.SequenceWithMemoryNode([
                                            bt_ros.CompleteCollectGround("manipulator_client"),
                                            bt_ros.MainSetManipulatortoGround("manipulator_client"),
                                        ]),
                                        bt_ros.MoveLineToPoint(self.blunium_nose_start_push_pose, "move_client")
                                    ], threshold=2)
                                ]),
                                # if we failed to pump puck - try again while moving to push blunium
                                bt.ParallelWithMemoryNode([
                                    bt.FallbackWithMemoryNode([
                                        bt.SequenceWithMemoryNode([
                                            bt_ros.StartCollectGroundCheck("manipulator_client"),
                                            bt.ActionNode(self.update_chaos_pucks),
                                            bt.ActionNode(lambda: self.score_master.add(self.incoming_puck_color.get())),
                                            bt_ros.CompleteCollectGround("manipulator_client"),
                                            bt_ros.MainSetManipulatortoGround("manipulator_client"),
                                        ]),
                                        bt.SequenceWithMemoryNode([
                                            bt_ros.StopPump("manipulator_client"),
                                            bt_ros.MainSetManipulatortoGround("manipulator_client")
                                        ])
                                    ]),
                                    bt_ros.MoveLineToPoint(self.blunium_nose_start_push_pose, "move_client")
                                ], threshold=2)
                            ])
                        ])

        push_nose_blunium = bt.SequenceWithMemoryNode([
                                bt.ParallelWithMemoryNode([
                                    bt_ros.StepperUp("manipulator_client"),
                                    bt_ros.MoveLineToPoint(self.blunium_nose_end_push_pose, "move_client"),
                                ], threshold=2),
                                bt.ActionNode(lambda: self.score_master.add("BLUNIUM")),
                                bt.ActionNode(lambda: self.score_master.unload("ACC")),
                                bt.ActionNode(lambda: self.score_master.reward("UNLOCK_GOLDENIUM_BONUS")),
                                bt_ros.SetSpeedSTM([-0.05, -0.1, 0], 0.9, "stm_client")  # [0, -0.1, 0]
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

        collect_unload_goldenium = bt.SequenceWithMemoryNode([
                                        bt_ros.Delay500("manipulator_client"),
                                        bt_ros.MoveLineToPoint(self.goldenium_2_PREgrab_pos, "move_client"),
                                        bt_ros.StartCollectGoldenium("manipulator_client"),
                                        bt.ParallelWithMemoryNode([
                                            bt_ros.MoveLineToPoint(self.goldenium_grab_pos, "move_client"),
                                            bt_ros.CheckLimitSwitchInfLong("manipulator_client")
                                        ], threshold=1),
                                        bt_ros.MoveLineToPoint(self.goldenium_back_pose, "move_client"),

                                        bt.FallbackWithMemoryNode([
                                            bt.SequenceWithMemoryNode([
                                                bt_ros.GrabGoldeniumAndHoldUp("manipulator_client"),
                                                bt.ActionNode(lambda: self.score_master.add("GOLDENIUM")),
                                                bt.ActionNode(lambda: self.score_master.reward("GRAB_GOLDENIUM_BONUS")),
                                                bt_ros.MoveLineToPoint(self.scales_goldenium_PREpos, "move_client"),

                                                bt.SequenceWithMemoryNode([
                                                    bt.ConditionNode(self.is_scales_landing_free),
                                                    bt.SequenceWithMemoryNode([
                                                        bt.ParallelWithMemoryNode([
                                                            bt_ros.MoveLineToPoint(self.scales_goldenium_pos + np.array([0, -0.01, 0]), "move_client"),
                                                            bt_ros.SetToScales_ifReachedGoal(self.scales_goldenium_pos + np.array([0, -0.05, 0]), "manipulator_client", threshold=0.03),
                                                            bt_ros.PublishScore_ifReachedGoal(self.scales_goldenium_pos + np.array([0, -0.05, 0]), self.score_master, "SCALES", threshold=0.03),
                                                        ], threshold=3),
                                                        #     bt.ActionNode(lambda: self.get_yolo_observation(area="all_center")), # TODO
                                                        bt_ros.GrabGoldeniumAndHoldUp("manipulator_client"),
                                                        bt_ros.SetManipulatortoWall("manipulator_client"),
                                                        bt_ros.GrabGoldeniumAndHoldUp("manipulator_client"),
                                                        bt_ros.MoveLineToPoint(self.scales_goldenium_pos, "move_client"),
                                                        bt_ros.UnloadGoldenium("manipulator_client"),
                                                        bt_ros.SetManipulatortoUp("manipulator_client"),
                                                    ])
                                                ])
                                            ]),

                                            bt.SequenceWithMemoryNode([
                                                bt_ros.StartCollectGoldenium("manipulator_client"),
                                                bt.ParallelWithMemoryNode([
                                                    bt_ros.MoveLineToPoint(self.goldenium_grab_pos, "move_client"),
                                                    bt_ros.CheckLimitSwitchInfLong("manipulator_client")
                                                ], threshold=1),
                                                bt_ros.MoveLineToPoint(self.goldenium_back_pose, "move_client"),

                                                bt.FallbackWithMemoryNode([
                                                    bt.SequenceWithMemoryNode([
                                                        bt_ros.GrabGoldeniumAndHoldUp("manipulator_client"),
                                                        bt.ActionNode(lambda: self.score_master.add("GOLDENIUM")),
                                                        bt.ActionNode(lambda: self.score_master.reward("GRAB_GOLDENIUM_BONUS")),
                                                        bt_ros.MoveLineToPoint(self.scales_goldenium_PREpos, "move_client"),

                                                        bt.SequenceWithMemoryNode([
                                                            bt.ConditionNode(self.is_scales_landing_free),
                                                            bt.SequenceWithMemoryNode([
                                                                bt.ParallelWithMemoryNode([
                                                                    bt_ros.MoveLineToPoint(self.scales_goldenium_pos + np.array([0, -0.01, 0]), "move_client"),
                                                                    bt_ros.SetToScales_ifReachedGoal(self.scales_goldenium_pos + np.array([0, -0.05, 0]), "manipulator_client", threshold=0.03),
                                                                    bt_ros.PublishScore_ifReachedGoal(self.scales_goldenium_pos + np.array([0, -0.05, 0]), self.score_master, "SCALES", threshold=0.03),
                                                                ], threshold=3),
                                                                #     bt.ActionNode(lambda: self.get_yolo_observation(area="all_center")), # TODO
                                                                bt_ros.GrabGoldeniumAndHoldUp("manipulator_client"),
                                                                bt_ros.SetManipulatortoWall("manipulator_client"),
                                                                bt_ros.GrabGoldeniumAndHoldUp("manipulator_client"),
                                                                bt_ros.MoveLineToPoint(self.scales_goldenium_pos, "move_client"),
                                                                bt_ros.UnloadGoldenium("manipulator_client"),
                                                                bt_ros.SetManipulatortoUp("manipulator_client"),
                                                            ])
                                                        ])
                                                    ]),

                                                    bt.SequenceWithMemoryNode([
                                                        bt_ros.StopPump("manipulator_client"),
                                                        bt_ros.SetManipulatortoGround("manipulator_client"),
                                                        bt_ros.MoveLineToPoint(self.scales_goldenium_PREpos, "move_client")
                                                    ])
                                                ])
                                            ])
                                        ])
                                    ])

        # unload_goldenium = bt.SequenceWithMemoryNode([
        #                         bt.ConditionNode(self.is_scales_landing_free),
        #                         bt.SequenceWithMemoryNode([
        #                             bt.ParallelWithMemoryNode([
        #                                 bt_ros.MoveLineToPoint(self.scales_goldenium_pos + np.array([0, -0.01, 0]), "move_client"),
        #                                 bt_ros.SetToScales_ifReachedGoal(self.scales_goldenium_pos + np.array([0, -0.05, 0]), "manipulator_client", threshold=0.03),
        #                                 bt_ros.PublishScore_ifReachedGoal(self.scales_goldenium_pos + np.array([0, -0.05, 0]), self.score_master, "SCALES", threshold=0.03),
        #                             ], threshold=3),
        #                             #     bt.ActionNode(lambda: self.get_yolo_observation(area="all_center")), # TODO
        #                             bt_ros.GrabGoldeniumAndHoldUp("manipulator_client"),
        #                             bt_ros.SetManipulatortoWall("manipulator_client"),
        #                             bt_ros.GrabGoldeniumAndHoldUp("manipulator_client"),
        #                             bt_ros.MoveLineToPoint(self.scales_goldenium_pos, "move_client"),
        #                             bt_ros.UnloadGoldenium("manipulator_client"),
        #                             bt_ros.SetManipulatortoUp("manipulator_client"),
        #                         ])
        #                     ])

        search_lost_puck_unload_cell = bt.SequenceWithMemoryNode([
                                            bt.ConditionNode(self.is_lost_puck_present),
                                            bt.ActionNode(lambda: self.calculate_next_landing(self.visible_pucks_on_field.get()[0])),
                                            bt_ros.MoveToVariable(self.next_prelanding_var, "move_client"),
                                            bt_ros.MoveToVariable(self.next_landing_var, "move_client"),

                                            bt.FallbackWithMemoryNode([
                                                bt.SequenceWithMemoryNode([
                                                    bt_ros.StartCollectGroundCheck("manipulator_client"),
                                                    bt.ActionNode(lambda: self.score_master.add(get_color(self.visible_pucks_on_field.get()[0]))),
                                                    bt.ParallelWithMemoryNode([
                                                        bt_ros.CompleteCollectGround("manipulator_client"),
                                                        bt_ros.MoveLineToPoint(self.blunium_nose_end_push_pose, "move_client")
                                                    ], threshold=2),
                                                    bt_ros.SetSpeedSTM([-0.05, -0.1, 0], 0.9, "stm_client"),
                                                    bt_ros.UnloadAccelerator("manipulator_client"),
                                                    bt.ActionNode(lambda: self.score_master.unload("ACC"))
                                                ]),
                                                bt_ros.StopPump("manipulator_client"),
                                            ])
                                        ])

        ## INTERVIENE STRATEGY
        self.tree = bt.SequenceWithMemoryNode([
                        bt.ActionNode(self.update_robot_status),

                        bt.FallbackWithMemoryNode([
                            bt.SequenceNode([
                                bt.ConditionNode(self.is_opp_chaos_observed),
                                move_to_opp_chaos_while_taking_red,
                                # move_to_opp_chaos_while_taking_red_collision,
                                collect_chaos  # FIXME
                            ]),
                            bt.ConditionNode(lambda: bt.Status.RUNNING)  # infinitely waiting for camera
                            # bt.SequenceWithMemoryNode([
                            #     collect_red_cell_puck,
                            #     blind_move_chaos_center_collect,
                            #     collect_green_cell_puck
                            # ])
                        ]),

                        push_nose_blunium,
                        unload_acc,
                        collect_unload_goldenium,
                        search_lost_puck_unload_cell,
                        bt_ros.SetManipulatortoUp("manipulator_client"),
                        bt_ros.MoveLineToPoint(self.starting_pos, "move_client")
                    ])

        ## TODO move to sber str
        ## SAFE STRATEGY
        # self.tree = bt.SequenceWithMemoryNode([
        #                 bt.ActionNode(self.update_robot_status),
        #
        #                 bt.FallbackWithMemoryNode([
        #                     bt.SequenceNode([
        #                         bt.ConditionNode(self.is_my_chaos_observed),
        #                         move_guard_our_chaos_while_taking_red,
        #                         collect_chaos,
        #                         collect_green_cell_puck
        #                     ]),
        #                     bt.SequenceWithMemoryNode([
        #                         collect_red_cell_puck,
        #                         blind_move_chaos_center_collect,
        #                         collect_green_cell_puck2
        #                     ])
        #                 ]),
        #
        #                 push_nose_blunium,
        #                 unload_acc,
        #                 collect_unload_goldenium,
        #                 bt_ros.SetManipulatortoUp("manipulator_client"),
        #                 bt_ros.MoveLineToPoint(self.starting_pos, "move_client")
        #             ])


class SberStrategy(StrategyConfig):  # pucks_slave
    def __init__(self, side):
        super(SberStrategy, self).__init__(side)  # pucks_slave

        move_to_guard_zone = bt.SequenceWithMemoryNode([
                                bt.ParallelWithMemoryNode([
                                    bt.SequenceWithMemoryNode([
                                        bt_ros.MoveLineToPoint(self.guard_chaos_loc, "move_client")
                                    ]),
                                    bt.SequenceWithMemoryNode([
                                        bt_ros.Delay500("manipulator_client"),
                                        bt_ros.DelayBlindStartCollectGround("manipulator_client"),
                                        bt_ros.CompleteCollectGround("manipulator_client"),
                                    ])
                                ], threshold=2),
                                bt.ActionNode(lambda: self.score_master.add("REDIUM"))   # FIXME!!!!!!!!!!!!!!!!!!!!!
                            ])

        move_home = bt_ros.MoveLineToPoint(self.starting_pos, "move_client")

        move_to_opp_chaos = bt.SequenceWithMemoryNode([
                                bt.ActionNode(self.calculate_pucks_configuration),
                                bt.ActionNode(lambda: self.calculate_next_landing(self.opponent_chaos_pucks.get()[0])),

                                bt.ParallelWithMemoryNode([
                                    bt_ros.MoveToVariable(self.next_landing_var, "move_client"),
                                    bt.SequenceWithMemoryNode([
                                        bt_ros.DelayBlindStartCollectGround("manipulator_client"),
                                        bt_ros.CompleteCollectGround("manipulator_client")
                                    ])
                                ], threshold=2),
                                bt.ActionNode(lambda: self.score_master.add("REDIUM")),

                                bt_ros.BlindStartCollectGround("manipulator_client"),
                                bt.ActionNode(self.calculate_pucks_configuration),
                                bt.ActionNode(self.set_closest_chaos_landing),
                                bt.ActionNode(self.set_prelanding_to_chaos_landing),
                                bt.ParallelWithMemoryNode([
                                    bt_ros.CompleteCollectGround("manipulator_client"),
                                    bt.SequenceWithMemoryNode([
                                        bt_ros.MoveToVariable(self.nearest_PRElanding, "move_client"),
                                        bt.ActionNode(self.calculate_pucks_configuration),
                                        bt.ActionNode(self.set_closest_chaos_landing),
                                        bt_ros.MoveToVariable(self.closest_landing, "move_client"),
                                    ])
                                ], threshold=2),
                                bt.ActionNode(lambda: self.update_chaos_pucks(side="opponent")),
                                bt.ActionNode(lambda: self.score_master.add(self.incoming_puck_color.get())),
        ])

        collect_chaos = bt.SequenceWithMemoryNode([
                    # # 1st, but done in finishing red
                    # bt.ActionNode(self.calculate_pucks_configuration),
                    # bt.ActionNode(self.calculate_closest_landing),
                    # bt.ActionNode(self.calculate_prelanding),
                    
                    # bt_ros.MoveToVariable(self.nearest_PRElanding, "move_client"),
                    # bt_ros.ArcMoveToVariable(self.closest_landing, "move_client"),



                    bt_ros.BlindStartCollectGround("manipulator_client"),
                    bt.ActionNode(self.update_chaos_pucks),
                    bt.ActionNode(lambda: self.score_master.add(self.incoming_puck_color.get())),
                    bt_ros.MoveToVariable(self.nearest_PRElanding, "move_client"),

                    # 2nd
                    bt.ActionNode(self.calculate_pucks_configuration),
                    bt.ActionNode(self.set_closest_chaos_landing),
                    bt.ActionNode(self.set_prelanding_to_chaos_landing),

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
                    bt.ActionNode(self.set_closest_chaos_landing),
                    bt.ActionNode(self.set_prelanding_to_chaos_landing),

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
                    bt.ActionNode(self.set_closest_chaos_landing),

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

        finish_chaos_push_nose_blunium = bt.SequenceWithMemoryNode([
                                bt.ParallelWithMemoryNode([
                                    bt.SequenceWithMemoryNode([
                                        bt_ros.CompleteCollectGround("manipulator_client"),
                                        bt_ros.MainSetManipulatortoGround("manipulator_client"),  # FIXME when adding chaos
                                        bt_ros.StepperUp("manipulator_client")
                                    ]),
                                    bt_ros.MoveLineToPoint(self.blunium_nose_start_push_pose, "move_client"),
                                ], threshold=2),
                                bt_ros.MoveLineToPoint(self.blunium_nose_end_push_pose, "move_client"),
                                bt.ActionNode(lambda: self.score_master.add("BLUNIUM")),
                                bt.ActionNode(lambda: self.score_master.unload("ACC")),
                                bt.ActionNode(lambda: self.score_master.reward("UNLOCK_GOLDENIUM_BONUS")),
                            ])

        approach_acc = bt.SequenceWithMemoryNode([
                            bt_ros.SetSpeedSTM([-0.05, -0.1, 0], 0.9, "stm_client")  # [0, -0.1, 0]
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
                                bt_ros.Delay500("manipulator_client"),
                                bt_ros.MoveLineToPoint(self.goldenium_2_PREgrab_pos, "move_client"),
                                bt_ros.StartCollectGoldenium("manipulator_client"),

                                bt.ParallelWithMemoryNode([
                                    bt_ros.MoveLineToPoint(self.goldenium_grab_pos, "move_client"),
                                    bt_ros.CheckLimitSwitchInfLong("manipulator_client")
                                ], threshold=1)
                            ])

        move_to_goldenium_prepose = bt.SequenceWithMemoryNode([
                                        bt_ros.MoveLineToPoint(self.goldenium_back_pose, "move_client"),
                                        bt.ActionNode(lambda: self.score_master.add("GOLDENIUM")),
                                        bt.ActionNode(lambda: self.score_master.reward("GRAB_GOLDENIUM_BONUS")),
                                        bt.ParallelWithMemoryNode([
                                            bt_ros.GrabGoldeniumAndHoldUp("manipulator_client"),
                                            bt_ros.MoveLineToPoint(self.scales_goldenium_PREpos, "move_client")
                                        ], threshold=2)
                                        # bt_ros.MoveLineToPoint(self.goldenium_back_rot_pose, "move_client"),
                                    ])

        unload_goldenium = bt.SequenceWithMemoryNode([
                                bt.ConditionNode(self.is_scales_landing_free),
                                bt.SequenceWithMemoryNode([
                                    bt.ParallelWithMemoryNode([
                                        bt_ros.MoveLineToPoint(self.scales_goldenium_pos + np.array([0, -0.01, 0]), "move_client"),
                                        bt_ros.SetToScales_ifReachedGoal(self.scales_goldenium_pos + np.array([0, -0.07, 0]), "manipulator_client", threshold=0.1),
                                        bt_ros.PublishScore_ifReachedGoal(self.scales_goldenium_pos + np.array([0, -0.1, 0]), self.score_master, "SCALES", threshold=0.1),
                                    ], threshold=3),
                                    # bt.ActionNode(lambda: self.score_master.unload("SCALES")),
                                    # bt_ros.SetManipulatortoWall("manipulator_client"),
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
                        # #red_cell_puck
                        # # move_to_opp_chaos,
                        # # collect_chaos,
                        # move_to_guard_zone,

                        bt.FallbackWithMemoryNode([
                            bt.SequenceNode([
                                bt.ConditionNode(self.is_my_chaos_observed),
                                move_to_opp_chaos
                            ]),
                            bt.ConditionNode(lambda: bt.Status.RUNNING)  # infinitely waiting for camera
                            # move_to_guard_zone
                        ]),

                        bt.FallbackWithMemoryNode([
                            bt.SequenceNode([
                                bt.ConditionNode(self.is_my_chaos_observed),
                                collect_chaos
                            ]),
                            bt.ConditionNode(lambda: bt.Status.RUNNING)  # infinitely waiting for camera
                            # move_home
                        ]),

                        finish_chaos_push_nose_blunium,
                        approach_acc,
                        unload_acc,
                        collect_goldenium,
                        move_to_goldenium_prepose,
                        unload_goldenium
                    ])


class TestField(StrategyConfig):
    def __init__(self, side):  # pucks_slave
        super(TestField, self).__init__(side)  # pucks_slave

        collect_red_cell_puck = bt.SequenceWithMemoryNode([
                                    bt_ros.MoveLineToPoint(self.first_puck_landing, "move_client"),
                                    bt.FallbackWithMemoryNode([
                                        bt.SequenceWithMemoryNode([
                                            bt_ros.StartCollectGroundCheck("manipulator_client"),
                                            bt.ActionNode(lambda: self.score_master.add(get_color(self.our_pucks_rgb.get()[0]))),
                                            bt_ros.CompleteCollectGround("manipulator_client"),
                                        ]),
                                        bt.ParallelWithMemoryNode([
                                            bt_ros.StopPump("manipulator_client"),
                                            bt_ros.SetManipulatortoWall("manipulator_client"),
                                        ], threshold=2),
                                    ])
                                ])

        collect_green_cell_puck2 = bt.SequenceWithMemoryNode([

                                        bt.ActionNode(lambda: self.calculate_next_landing(self.green_cell_puck)),
                                        bt.ActionNode(self.calculate_next_prelanding),
                                        bt_ros.MoveToVariable(self.next_prelanding_var, "move_client"),
                                        bt_ros.MoveToVariable(self.next_landing_var, "move_client"),

                                        bt.FallbackWithMemoryNode([
                                            bt.SequenceWithMemoryNode([
                                                bt_ros.StartCollectGroundCheck("manipulator_client"),
                                                bt.ActionNode(lambda: self.score_master.add(get_color(self.our_pucks_rgb.get()[1]))),
                                                bt_ros.CompleteCollectGround("manipulator_client")
                                            ]),
                                            bt_ros.StopPump("manipulator_client")
                                        ])
                                    ])

        collect_chaos = bt.SequenceWithMemoryNode([

                            bt.ActionNode(self.calculate_pucks_configuration),
                            bt.ActionNode(self.set_closest_chaos_landing),
                            bt.ActionNode(self.set_prelanding_to_chaos_landing),

                            bt_ros.MoveToVariable(self.nearest_PRElanding, "move_client"),
                            bt_ros.ArcMoveToVariable(self.closest_landing, "move_client"),

                            # # We here arrived to 1st chaos puck and ready to start collecting
                            bt.FallbackWithMemoryNode([
                                bt.SequenceWithMemoryNode([
                                    bt_ros.StartCollectGroundCheck("manipulator_client"),
                                    bt.ActionNode(self.update_chaos_pucks),
                                    bt.ActionNode(lambda: self.score_master.add(self.incoming_puck_color.get())),
                                    bt.ActionNode(self.set_move_back_pose),
                                    bt_ros.MoveToVariable(self.move_back_pose, "move_client"),  # to calculated move_back_pose

                                    bt.ActionNode(self.calculate_pucks_configuration),
                                    bt.ActionNode(self.set_closest_chaos_landing),
                                    bt.ActionNode(self.set_prelanding_to_chaos_landing),

                                    bt.ParallelWithMemoryNode([
                                        bt_ros.CompleteCollectGround("manipulator_client"),
                                        bt.SequenceWithMemoryNode([
                                            bt_ros.MoveToVariable(self.nearest_PRElanding, "move_client"),
                                            bt_ros.MoveToVariable(self.closest_landing, "move_client")
                                        ])
                                    ], threshold=2),
                                ]),
                                # if we failed to pump puck
                                bt.SequenceWithMemoryNode([
                                    bt_ros.StopPump("manipulator_client"),
                                    bt.ActionNode(self.set_move_back_pose),
                                    bt_ros.MoveToVariable(self.move_back_pose, "move_client"),  # to calculated move_back_pose
                                    bt.ActionNode(self.remove_uncollected_chaos_puck),
                                    bt.ActionNode(self.calculate_pucks_configuration),
                                    bt.ActionNode(self.set_closest_chaos_landing),
                                    bt.ActionNode(self.set_prelanding_to_chaos_landing),

                                    bt.ParallelWithMemoryNode([
                                        bt_ros.SetManipulatortoWall("manipulator_client"),
                                        bt.SequenceWithMemoryNode([
                                            bt_ros.MoveToVariable(self.nearest_PRElanding, "move_client"),
                                            bt_ros.MoveToVariable(self.closest_landing, "move_client")
                                        ])
                                    ], threshold=2)
                                ])
                            ]),


                            # # We here arrived to 2nd chaos puck and ready to start collecting
                            bt.FallbackWithMemoryNode([
                                bt.SequenceWithMemoryNode([
                                    bt_ros.StartCollectGroundCheck("manipulator_client"),
                                    bt.ActionNode(self.update_chaos_pucks),
                                    bt.ActionNode(lambda: self.score_master.add(self.incoming_puck_color.get())),
                                    bt.ActionNode(self.set_move_back_pose),
                                    bt_ros.MoveToVariable(self.move_back_pose, "move_client"),  # to calculated move_back_pose

                                    bt.ActionNode(self.calculate_pucks_configuration),
                                    bt.ActionNode(self.set_closest_chaos_landing),
                                    bt.ActionNode(self.set_prelanding_to_chaos_landing),

                                    bt.ParallelWithMemoryNode([
                                        bt_ros.CompleteCollectGround("manipulator_client"),
                                        bt.SequenceWithMemoryNode([
                                            bt_ros.MoveToVariable(self.nearest_PRElanding, "move_client"),
                                            bt_ros.MoveToVariable(self.closest_landing, "move_client")
                                        ])
                                    ], threshold=2),
                                ]),
                                # if we failed to pump puck
                                bt.SequenceWithMemoryNode([
                                    bt_ros.StopPump("manipulator_client"),
                                    bt.ActionNode(self.set_move_back_pose),
                                    bt_ros.MoveToVariable(self.move_back_pose, "move_client"),  # to calculated move_back_pose
                                    bt.ActionNode(self.remove_uncollected_chaos_puck),
                                    bt.ActionNode(self.calculate_pucks_configuration),
                                    bt.ActionNode(self.set_closest_chaos_landing),
                                    bt.ActionNode(self.set_prelanding_to_chaos_landing),

                                    bt.ParallelWithMemoryNode([
                                        bt_ros.SetManipulatortoWall("manipulator_client"),
                                        bt.SequenceWithMemoryNode([
                                            bt_ros.MoveToVariable(self.nearest_PRElanding, "move_client"),
                                            bt_ros.MoveToVariable(self.closest_landing, "move_client")
                                        ])
                                    ], threshold=2)
                                ])
                            ]),

                            # # We here arrived to 3rd chaos puck and ready to start collecting
                            bt.FallbackWithMemoryNode([
                                bt.SequenceWithMemoryNode([
                                    bt_ros.StartCollectGroundCheck("manipulator_client"),
                                    bt.ActionNode(self.update_chaos_pucks),
                                    bt.ActionNode(lambda: self.score_master.add(self.incoming_puck_color.get())),
                                    bt.ActionNode(self.set_move_back_pose),
                                    bt_ros.MoveToVariable(self.move_back_pose, "move_client"),  # to calculated move_back_pose

                                    bt.ActionNode(self.calculate_pucks_configuration),
                                    bt.ActionNode(self.set_closest_chaos_landing),
                                    bt.ActionNode(self.set_prelanding_to_chaos_landing),

                                    bt.ParallelWithMemoryNode([
                                        bt_ros.CompleteCollectGround("manipulator_client"),
                                        bt.SequenceWithMemoryNode([
                                            bt_ros.MoveToVariable(self.nearest_PRElanding, "move_client"),
                                            bt_ros.MoveToVariable(self.closest_landing, "move_client")
                                        ])
                                    ], threshold=2),
                                ]),
                                # if we failed to pump puck
                                bt.SequenceWithMemoryNode([
                                    bt_ros.StopPump("manipulator_client"),
                                    bt.ActionNode(self.set_move_back_pose),
                                    bt_ros.MoveToVariable(self.move_back_pose, "move_client"),  # to calculated move_back_pose
                                    bt.ActionNode(self.remove_uncollected_chaos_puck),
                                    bt.ActionNode(self.calculate_pucks_configuration),
                                    bt.ActionNode(self.set_closest_chaos_landing),
                                    bt.ActionNode(self.set_prelanding_to_chaos_landing),

                                    bt.ParallelWithMemoryNode([
                                        bt_ros.SetManipulatortoWall("manipulator_client"),
                                        bt.SequenceWithMemoryNode([
                                            bt_ros.MoveToVariable(self.nearest_PRElanding, "move_client"),
                                            bt_ros.MoveToVariable(self.closest_landing, "move_client")
                                        ])
                                    ], threshold=2)
                                ])
                            ]),

                            # # We here arrived to 4th chaos puck and ready to start collecting
                            bt.FallbackWithMemoryNode([
                                bt.SequenceWithMemoryNode([
                                    bt_ros.StartCollectGroundCheck("manipulator_client"),
                                    bt.ActionNode(self.update_chaos_pucks),
                                    bt.ActionNode(lambda: self.score_master.add(self.incoming_puck_color.get())),
                                    bt_ros.CompleteCollectGround("manipulator_client"),
                                    bt_ros.MainSetManipulatortoGround("manipulator_client"),
                                ]),
                                # if we failed to pump puck - try again while moving to push blunium
                                bt.SequenceWithMemoryNode([
                                    bt_ros.StopPump("manipulator_client"),
                                    bt_ros.MainSetManipulatortoGround("manipulator_client")
                                ])
                            ])
                        ])

        push_nose_blunium = bt.SequenceWithMemoryNode([
                                bt.ParallelWithMemoryNode([
                                    bt_ros.StepperUp("manipulator_client"),
                                    bt_ros.MoveLineToPoint(self.blunium_nose_end_push_pose, "move_client"),
                                ], threshold=2),
                                bt.ActionNode(lambda: self.score_master.add("BLUNIUM")),
                                bt.ActionNode(lambda: self.score_master.unload("ACC")),
                                bt.ActionNode(lambda: self.score_master.reward("UNLOCK_GOLDENIUM_BONUS")),
                                bt_ros.SetSpeedSTM([-0.05, -0.1, 0], 0.9, "stm_client")  # [0, -0.1, 0]
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

        collect_unload_goldenium = bt.SequenceWithMemoryNode([
                                        bt_ros.Delay500("manipulator_client"),
                                        bt_ros.MoveLineToPoint(self.goldenium_2_PREgrab_pos, "move_client"),
                                        bt_ros.StartCollectGoldenium("manipulator_client"),
                                        bt.ParallelWithMemoryNode([
                                            bt_ros.MoveLineToPoint(self.goldenium_grab_pos, "move_client"),
                                            bt_ros.CheckLimitSwitchInfLong("manipulator_client")
                                        ], threshold=1),
                                        bt_ros.MoveLineToPoint(self.goldenium_back_pose, "move_client"),

                                        bt.FallbackWithMemoryNode([
                                            bt.SequenceWithMemoryNode([
                                                bt_ros.GrabGoldeniumAndHoldUp("manipulator_client"),
                                                bt.ActionNode(lambda: self.score_master.add("GOLDENIUM")),
                                                bt.ActionNode(lambda: self.score_master.reward("GRAB_GOLDENIUM_BONUS")),
                                                bt_ros.MoveLineToPoint(self.scales_goldenium_PREpos, "move_client"),

                                                bt.SequenceWithMemoryNode([
                                                    bt.ConditionNode(self.is_scales_landing_free),
                                                    bt.SequenceWithMemoryNode([
                                                        bt.ParallelWithMemoryNode([
                                                            bt_ros.MoveLineToPoint(self.scales_goldenium_pos + np.array([0, -0.01, 0]), "move_client"),
                                                            bt_ros.SetToScales_ifReachedGoal(self.scales_goldenium_pos + np.array([0, -0.05, 0]), "manipulator_client", threshold=0.03),
                                                            bt_ros.PublishScore_ifReachedGoal(self.scales_goldenium_pos + np.array([0, -0.05, 0]), self.score_master, "SCALES", threshold=0.03),
                                                        ], threshold=3),
                                                        #     bt.ActionNode(lambda: self.get_yolo_observation(area="all_center")), # TODO
                                                        bt_ros.GrabGoldeniumAndHoldUp("manipulator_client"),
                                                        bt_ros.SetManipulatortoWall("manipulator_client"),
                                                        bt_ros.GrabGoldeniumAndHoldUp("manipulator_client"),
                                                        bt_ros.MoveLineToPoint(self.scales_goldenium_pos, "move_client"),
                                                        bt_ros.UnloadGoldenium("manipulator_client"),
                                                        bt_ros.SetManipulatortoUp("manipulator_client"),
                                                    ])
                                                ])
                                            ]),

                                            bt.SequenceWithMemoryNode([
                                                bt_ros.StartCollectGoldenium("manipulator_client"),
                                                bt.ParallelWithMemoryNode([
                                                    bt_ros.MoveLineToPoint(self.goldenium_grab_pos, "move_client"),
                                                    bt_ros.CheckLimitSwitchInfLong("manipulator_client")
                                                ], threshold=1),
                                                bt_ros.MoveLineToPoint(self.goldenium_back_pose, "move_client"),

                                                bt.FallbackWithMemoryNode([
                                                    bt.SequenceWithMemoryNode([
                                                        bt_ros.GrabGoldeniumAndHoldUp("manipulator_client"),
                                                        bt.ActionNode(lambda: self.score_master.add("GOLDENIUM")),
                                                        bt.ActionNode(lambda: self.score_master.reward("GRAB_GOLDENIUM_BONUS")),
                                                        bt_ros.MoveLineToPoint(self.scales_goldenium_PREpos, "move_client"),

                                                        bt.SequenceWithMemoryNode([
                                                            bt.ConditionNode(self.is_scales_landing_free),
                                                            bt.SequenceWithMemoryNode([
                                                                bt.ParallelWithMemoryNode([
                                                                    bt_ros.MoveLineToPoint(self.scales_goldenium_pos + np.array([0, -0.01, 0]), "move_client"),
                                                                    bt_ros.SetToScales_ifReachedGoal(self.scales_goldenium_pos + np.array([0, -0.05, 0]), "manipulator_client", threshold=0.03),
                                                                    bt_ros.PublishScore_ifReachedGoal(self.scales_goldenium_pos + np.array([0, -0.05, 0]), self.score_master, "SCALES", threshold=0.03),
                                                                ], threshold=3),
                                                                #     bt.ActionNode(lambda: self.get_yolo_observation(area="all_center")), # TODO
                                                                bt_ros.GrabGoldeniumAndHoldUp("manipulator_client"),
                                                                bt_ros.SetManipulatortoWall("manipulator_client"),
                                                                bt_ros.GrabGoldeniumAndHoldUp("manipulator_client"),
                                                                bt_ros.MoveLineToPoint(self.scales_goldenium_pos, "move_client"),
                                                                bt_ros.UnloadGoldenium("manipulator_client"),
                                                                bt_ros.SetManipulatortoUp("manipulator_client"),
                                                            ])
                                                        ])
                                                    ]),

                                                    bt.SequenceWithMemoryNode([
                                                        bt_ros.StopPump("manipulator_client"),
                                                        bt_ros.SetManipulatortoGround("manipulator_client"),
                                                        bt_ros.MoveLineToPoint(self.scales_goldenium_PREpos, "move_client")
                                                    ])
                                                ])
                                            ])
                                        ])
                                    ])

        self.tree = bt.SequenceWithMemoryNode([
                        bt.ActionNode(self.update_robot_status),

                        collect_red_cell_puck,

                        bt.FallbackWithMemoryNode([
                            bt.SequenceNode([
                                bt.ConditionNode(self.is_my_chaos_observed),
                                collect_chaos,
                            ]),
                            collect_green_cell_puck2
                        ]),

                        bt_ros.MoveLineToPoint(self.guard_chaos_loc, "move_client"),
                        bt_ros.MoveLineToPoint(self.blunium_nose_start_push_pose, "move_client"),

                        push_nose_blunium,
                        unload_acc,
                        collect_unload_goldenium,
                        bt_ros.SetManipulatortoUp("manipulator_client"),
                        bt_ros.MoveLineToPoint(self.starting_pos, "move_client")
                    ])


if __name__ == '__main__':
    try:
        rospy.init_node("main_robot_BT")
        main_robot_bt = MainRobotBT()
        bt_controller = BTController(main_robot_bt)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
