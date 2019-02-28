#!/usr/bin/env python
# coding: utf-8
import rospy
import numpy as np
import tf2_ros
from tf.transformations import euler_from_quaternion
from visualization_msgs.msg import MarkerArray
from std_msgs.msg import String
from threading import Lock
from manipulator import Manipulator
# from geometry_msgs.msg import Twist
# from std_msgs.msg import Int32MultiArray
from sympy import Point, Polygon

from core_functions import calculate_distance
# from core_functions import wrap_angle
from core_functions import wrap_back
from core_functions import cvt_local2global
from math import radians, degrees

"""

THIS VERSION CORRECTLY PASSES ALL TESTS USING  POLYGON FUNC   BUT   DOESN't COLLECT BLUE PUCKS






This algorithm knows nothing about obstacles and etc, it just generates sorted list of options.
And path-planner will choose from these options according to obstacles around

TODO:
    - clockwise approaching in order to keep camera view open
    - FIXME path_planner should decide which puck to collect and to remove from known
    - in case we parse only first frame and while collecting pucks robot moves some of them, applying sort func is useless 
    - Alexey need to parse particular local zones
    - add check if there are more than 4 pucks in zone
    - add approach_immidiately flag

4 in chaos zone
3 periodic table
6 wall
3 wall
1 ramp

Input: list of n coordinates of pucks, that belong to one local group

Receiving coords from camera each 2-3 seconds
Each time coords are received we call function callback which compares currently known coords and newly received ones
If difference between newly received and known are within accuracy level (threshold), do nothing, else update known coords

Inside:
    - calculates the convex hull of these points
    - calculates inner angles and sorts them
    - calculates bissectrisa of the angle and adds offset
    -
"""


class TacticsNode:
    def __init__(self):
        rospy.init_node('TacticsNode', anonymous=True)

        # TF
        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer)
        self.mutex = Lock()
        self.robot_name = "secondary_robot"
        self.red_zone_coords = np.array([0.4, 0.4, 3.14])
        self.critical_angle = np.pi * 2/3
        
        self.approach_dist = 0.11  # meters, distance from robot to puck where robot will try to grab it
        # self.approach_dist = 0.153  # meters, distance from robot to puck where robot will try to grab it

        self.approach_vec = np.array([-0.11, 0, 0])
        # self.approach_vec = np.array([-0.153, 0, 0])  # big robot

        self.drive_back_dist = np.array([-0.04, 0, 0])
        self.coords_threshold = 0.01  # meters, this is variance of detecting pucks coords using camera, used in update
        self.scale_factor = 10  # used in calculating outer bissectrisa for hull's angles
        self.RATE = 10
        self.pucks_unloaded = 0

        self.robot_coords = np.zeros(3)
        self.active_goal = None
        self.goal_landing = None
        self.atoms_inside = 0  # to preliminary calculate our score

        self.sorted_chaos_landings = np.array([])
        self.known_chaos_pucks = np.array([])  # (id, x, y)
        self.known_wall6_pucks = np.array([])
        self.wall_six_landing = np.array([])

        self.operating_state = 'waiting for command'
        self.is_finished = False
        self.is_puck_collected_and_arm_ready = False
        self.is_robot_moving = False
        self.is_robot_collecting_puck = False
        self.is_puck_collected = False
        self.is_puck_sucked = False

        self.cmd_id = None
        self.cmd_type = None

        # publishers
        self.move_command_publisher = rospy.Publisher('move_command', String, queue_size=10)
        self.stm_command_publisher = rospy.Publisher('stm_command', String, queue_size=1)
        self.response_publisher = rospy.Publisher("response", String, queue_size=10)

        rospy.sleep(2)

        self.timer = None

        print("here is ok should be")
        self.manipulator = Manipulator()
        print("here is ok? init = Manip")
        
        # ON SECONDARY
        if not self.manipulator.calibrate_small():
            print("here shouldn't be")
            return
        rospy.sleep(2)
        print("here must be")

        # ON MAIN
        # if not self.manipulator.calibrate_big():
        #     print("here shouldn't be")
        #     return

        # coords are published as markers in one list according to 91-92 undistort.py
        # FIXME
        #  add /secondary_robot when in simulator, remove when on robot!!!!!!!!!!!

        rospy.Subscriber("/pucks", MarkerArray, self.pucks_coords_callback, queue_size=1)
        rospy.Subscriber("cmd_tactics", String, self.tactics_callback, queue_size=1)
        rospy.Subscriber("response", String, self.response_callback, queue_size=10)

    def pucks_coords_callback(self, data):  # FIXME this is for chaos zone and landings for CHAOS zone are sorted !!!!!
        """
        implement comparing with threshold,
        if newly received coord differs from old known one less than threshold level,
        than ignore it and continue collecting pucks.
        Else change coord of that puck and recalculate

        In first step we just write received coords to list of known pucks,
        in further steps we compare two lists and decide whether to update it or ignore

        :param self:
        :param data:
        :return:
        """
        self.mutex.acquire()

        if len(self.known_chaos_pucks) == 0:

            # FIXME IDs are not guaranteed to be the same from frame to frame, add id of zone
            new_observation_pucks = [[marker.pose.position.x, marker.pose.position.y, marker.id, marker.color.r, marker.color.g, marker.color.b] for marker in data.markers]
            # [(0.95, 1.1, 3, 0, 0, 1), ...] - blue, id=3
            new_observation_pucks = np.array(new_observation_pucks)

            print('TN -- new_observation_pucks')
            print(new_observation_pucks)
            try:
                self.known_chaos_pucks = new_observation_pucks[:, :2]
                print("known")
                print(self.known_chaos_pucks)
            except Exception as Error:
                print("list index out of range - no visible pucks on the field ")


        # # FIXME IDs are not guaranteed to be the same from frame to frame
        # # add id of zone!!!!!!!!!!!!
        # new_observation_pucks = [(x_.id, x_.pose.position.x, x_.pose.position.y) for x_ in data.markers]
        # new_observation_pucks = np.array(new_observation_pucks)
        # print('TN -- new_observation_pucks')
        # print(new_observation_pucks)
        #
        # # self.known_chaos_pucks = np.array([])  # FIXME this is just for tests!
        #
        # if len(self.known_chaos_pucks) == 0:
        #     self.known_chaos_pucks = new_observation_pucks[:, 1:]  # [array(x0, y0), array(x1, y1), ...]
            # coords = self.known_chaos_pucks[:, 1:]  # [(x0, y0), (x1, y1), ...]
            # self.known_chaos_pucks = self.calculate_pucks_configuration(coords)

        # else:
        #     self.compare_to_update_or_ignore(new_observation_pucks)  # in case robot accidentally moved some of pucks

        self.mutex.release()

    # noinspection PyTypeChecker
    def tactics_callback(self, data):

        self.mutex.acquire()

        if self.timer is not None:
            self.timer.shutdown()

        # self.parse_and_update_active_cmd(data)
        rospy.loginfo("")
        rospy.loginfo("=====================================")
        rospy.loginfo("TN: NEW CMD:\t" + str(data.data))
        rospy.loginfo("=====================================")
        rospy.loginfo("")

        data_split = data.data.split()
        self.cmd_id = data_split[0]
        self.cmd_type = data_split[1]

        self.timer = rospy.Timer(rospy.Duration(1.0 / self.RATE), self.timer_callback)

        # FIXME There is a time delay for coords from camera to come
        self.mutex.release()

    # noinspection PyUnusedLocal
    def timer_callback(self, event):

        """
        When BT publishes command something like "collect atoms in Chaos Zone", we start calculating and updating configuration of pucks
        :param event: cmd_type can be one of these:
        - collect_chaos
        - if in chaos taken blue - put it separately
        - take_from_wall_and_hold_above (
        - take_goldenium_and_hold_middle
        - collect_accel_blue (diff height, maybe we need to hold it up for a while)
        - skip
        :return:
        """
        print("is finished?___1", self.is_finished)
        print ("OPERATING_STATE=",self.operating_state)

        if self.cmd_type == "collect_chaos" and len(self.known_chaos_pucks) > 0:
            self.execute_puck_cmd()

        if self.atoms_inside == 4 and self.operating_state == "waiting for command":
            self.operating_state = "moving to red zone"
            rospy.loginfo(self.operating_state)
            self.move_to_start_zone()
            
        # print("is finished?___2", self.is_finished)

        if self.operating_state == "moving to red zone" and self.is_finished:
            print("is finished?", self.is_finished)
            self.operating_state = "start unloading"
            rospy.loginfo(self.operating_state)
            # self.unload_pucks_on_table()
            self.manipulator.release_small()
            self.pucks_unloaded = 4
            self.atoms_inside = 0

        if self.operating_state == "start unloading" and self.pucks_unloaded == 4:
            self.completely_stop()




        if self.cmd_type == "unload_in_red" and self.atoms_inside == 4:
            # self.unload_pucks_on_table()
            self.manipulator.release_small()
            self.pucks_unloaded = 4
            self.atoms_inside = 0

    def move_to_start_zone(self):
        self.is_finished = False
        self.active_goal = self.red_zone_coords
        cmd = self.compose_command(self.active_goal, cmd_id='move_to_red_zone', move_type='move_line')
        self.move_command_publisher.publish(cmd)
        print("is finished?", self.is_finished)

    # def unload_pucks_on_table(self):
    #     self.stm_command_publisher.publish("1 50")
    #     rospy.sleep(1.5)
    #     self.stm_command_publisher.publish("1 50")
    #     rospy.sleep(1.5)
    #     self.manipulator.release_small()
    #     self.pucks_unloaded += 1
    #     self.atoms_inside -= 1
    #     self.manipulator.release_small()
    #     self.pucks_unloaded += 1
    #     self.atoms_inside -= 1
    #     self.manipulator.release_small()
    #     self.pucks_unloaded += 1
    #     self.atoms_inside -= 1
    #     self.manipulator.release_small()
    #     self.pucks_unloaded += 1
    #     self.atoms_inside -= 1
        

    def execute_puck_cmd(self):
        """
        0. Get the closest to robot landing from the list
        1. Approach it
        2. Grab and load it
        3. Remove from list of known
        4. Add to list of collected and count points TODO

        we append id of that puck to list of collected pucks and remove it from list of pucks to be collected.
        :return:
        """

        # if self.operating_state == 'waiting somewhere' and not self.is_robot_moving:
        #     rospy.loginfo(self.operating_state)
        #     self.known_chaos_pucks = self.calculate_pucks_configuration()  # now [[x1, y1], ...]. Should be [(0.95, 1.1, 3, 0, 0, 1), ...]
        #     self.sorted_chaos_landings = self.calculate_landings(self.known_chaos_pucks)
        #     self.goal_landing = self.sorted_chaos_landings[0]
        #     prelanding = cvt_local2global(self.drive_back_dist*2, self.goal_landing)
        #     self.active_goal = prelanding
        #     self.is_finished = False
        #     cmd = self.compose_command(self.active_goal, cmd_id='approach_first_PRElanding', move_type='move_line')
        #     self.move_command_publisher.publish(cmd)
        #     self.is_robot_moving = True
        #     self.operating_state = 'approaching first PRElanding'
        #     rospy.loginfo(self.operating_state)
        #
        # if self.operating_state == 'approaching first PRElanding' and self.is_finished:
        #     self.operating_state = 'waiting for command'
        #     rospy.loginfo(self.operating_state)
        #     self.is_robot_moving = False
        #     self.is_finished = False
        #
        # # and here starts the cycle
        # if self.operating_state == 'waiting for command' and not self.is_robot_moving:
        #     rospy.loginfo(self.operating_state)
        #     self.known_chaos_pucks = self.calculate_pucks_configuration()  # now [[x1, y1], ...]. Should be [(0.95, 1.1, 3, 0, 0, 1), ...]
        #
        #     if len(self.known_chaos_pucks) > 1 and all(self.known_chaos_pucks[0][3:6] == [0, 0, 1]):  # blue not the last left in chaos zone
        #         print("blue not the last left in chaos zone, roll it!")
        #         self.known_chaos_pucks = np.roll(self.known_chaos_pucks, -1, axis=0)  # roll sorted list of knowns so blue becomes last one to collect
        #         print(self.known_chaos_pucks)
        #
        #     self.sorted_chaos_landings = self.calculate_landings(self.known_chaos_pucks)
        #     self.goal_landing = self.sorted_chaos_landings[0]
        #     prelanding = cvt_local2global(self.drive_back_dist, self.goal_landing)
        #     self.active_goal = prelanding
        #     self.is_finished = False
        #     cmd = self.compose_command(self.active_goal, cmd_id='approach_nearest_PRElanding', move_type='move_arc')
        #     self.move_command_publisher.publish(cmd)
        #     self.is_robot_moving = True
        #     self.operating_state = 'approaching nearest PRElanding'
        #     rospy.loginfo(self.operating_state)

        if self.operating_state == 'waiting for command' and not self.is_robot_moving:
            rospy.loginfo(self.operating_state)
            self.sorted_chaos_landings = self.calculate_pucks_configuration()

            self.goal_landing = self.sorted_chaos_landings[0]
            prelanding = cvt_local2global(self.drive_back_dist, self.goal_landing)

            self.active_goal = prelanding
            cmd = self.compose_command(self.active_goal, cmd_id='approach_nearest_PRElanding', move_type='move_arc')
            self.move_command_publisher.publish(cmd)
            self.is_robot_moving = True
            self.is_finished = False
            self.operating_state = 'approaching nearest PRElanding'
            rospy.loginfo(self.operating_state)

        if self.operating_state == 'approaching nearest PRElanding' and self.is_finished:
            self.operating_state = 'nearest PRElanding approached'
            rospy.loginfo(self.operating_state)
            self.is_robot_moving = False
            self.active_goal = None

        if self.operating_state == 'nearest PRElanding approached' and not self.is_robot_moving:
            self.active_goal = self.goal_landing
            self.is_finished = False
            cmd = self.compose_command(self.active_goal, cmd_id='approach_nearest_LANDING', move_type='move_line')
            self.move_command_publisher.publish(cmd)
            self.operating_state = 'approaching nearest LANDING'
            rospy.loginfo(self.operating_state)
            self.is_robot_moving = True

        if self.operating_state == 'approaching nearest LANDING' and self.is_finished:
            self.operating_state = 'nearest LANDING approached'
            rospy.loginfo(self.operating_state)
            self.is_robot_moving = False


        # if self.operating_state == 'nearest LANDING approached' and not self.is_robot_collecting_puck:
        #     # self.goal_landing = None
        #     self.is_robot_collecting_puck = True
        #     self.is_puck_sucked = self.manipulator.grab_and_suck_small()

        #     prelanding = cvt_local2global(self.drive_back_dist, self.goal_landing)
        #     self.active_goal = prelanding
        #     cmd = self.compose_command(self.active_goal, cmd_id='driving_back_for_safety', move_type='move_line')
        #     self.move_command_publisher.publish(cmd)

        #     self.is_puck_collected = self.manipulator.finish_collect_small()
        #     # self.is_puck_collected = self.manipulator.collect_big()

        #     self.operating_state = 'collecting puck'
        #     rospy.loginfo(self.operating_state)
        #     # self.imitate_manipulator()

        # if self.operating_state == 'collecting puck' and self.is_puck_collected:
        #     self.operating_state = 'puck successfully collected'
        #     rospy.loginfo("Wohoo! " + str(self.known_chaos_pucks[0]) + " " + str(self.operating_state))
        #     self.is_robot_collecting_puck = False
        #     self.atoms_inside += 1
        #     print(" ")
        #     print("now delete puck", self.known_chaos_pucks[0])
        #     print(" ")
        #     self.known_chaos_pucks = np.delete(self.known_chaos_pucks, 0, axis=0)
        #     rospy.loginfo('TN: pucks left: ' + str(len(self.known_chaos_pucks)))


        if self.operating_state == 'nearest LANDING approached' and not self.is_robot_collecting_puck:
            self.is_puck_sucked = self.manipulator.grab_and_suck_small()
            self.operating_state = 'sucking puck'
            rospy.loginfo(self.operating_state)
            self.is_robot_collecting_puck = True

        if self.operating_state == 'sucking puck' and self.is_puck_sucked:
            rospy.sleep(0.5)
            self.operating_state = 'driving back for safety'
            rospy.loginfo(self.operating_state)
            self.is_finished = False

            prelanding = cvt_local2global(self.drive_back_dist, self.goal_landing)
            self.active_goal = prelanding
            cmd = self.compose_command(self.active_goal, cmd_id='driving_back_for_safety', move_type='move_line')
            self.move_command_publisher.publish(cmd)

            self.is_robot_collecting_puck = False
            self.is_robot_moving = True

        if self.operating_state == 'driving back for safety' and self.is_finished:
            self.operating_state = 'finishing collecting puck'
            rospy.loginfo(self.operating_state)
            self.goal_landing = None
            self.is_finished = False
            self.is_puck_collected = self.manipulator.finish_collect_small()
            # self.is_puck_collected = self.manipulator.collect_big()
            self.is_robot_collecting_puck = True
            # self.imitate_manipulator()

        if self.operating_state == 'finishing collecting puck' and self.is_puck_collected:
            self.operating_state = 'puck successfully collected'
            rospy.loginfo("Wohoo! " + str(self.known_chaos_pucks[0]) + " " + str(self.operating_state))
            self.is_robot_collecting_puck = False
            self.atoms_inside += 1
            print(" ")
            print("now delete puck", self.known_chaos_pucks[0])
            print(" ")
            self.known_chaos_pucks = np.delete(self.known_chaos_pucks, 0, axis=0)
            rospy.loginfo('TN: pucks left: ' + str(len(self.known_chaos_pucks)))
            rospy.loginfo('TN: pucks Inside: ' + str(self.atoms_inside))

        if self.operating_state == 'puck successfully collected':
            self.is_puck_collected = False
            self.is_robot_moving = False
            self.active_goal = None
            self.sorted_chaos_landings = None
            self.is_robot_collecting_puck = False
            self.is_puck_sucked = False
            self.operating_state = 'waiting for command'
            rospy.loginfo(self.operating_state)


    def response_callback(self, data):
        """
        here when robot reaches the goal MotionPlannerNode will publish in response topic "finished" and in this code
        callback_response will fire and change self.robot_reached_goal_flag to True
        :param data:
        :return:
        """
        if data.data == 'finished':  # TODO change so that response is marked by cmd_id ''
            self.is_finished = True

    @staticmethod
    def compose_command(landing, cmd_id, move_type):
        x, y, theta = landing[0], landing[1], landing[2]
        command = str(cmd_id)
        command += ' '
        command += str(move_type)
        command += ' '
        command += str(x)
        command += ' '
        command += str(y)
        command += ' '
        command += str(theta)
        rospy.loginfo("=============================")
        rospy.loginfo('TN: NEW COMMAND COMPOSED')
        rospy.loginfo(command)
        rospy.loginfo("=============================")
        return command

    def imitate_manipulator(self):
        # TODO read responce from step motor and when puck is inside - only than update number of collected pucks
        rospy.loginfo('TN: puck collected by imitator!!')
        self.is_puck_collected = True

    # def stop_collecting(self):
    #     # self.timer.shutdown()
    #     # rospy.sleep(1.0 / 40)
    #     # print(" ")
    #     # print(" ")

    #     rospy.loginfo("TN -- Robot has stopped collecting pucks.")
    #     self.is_robot_moving = False
    #     # self.active_goal = None
    #     # self.is_finished = True
    #     self.is_robot_collecting_puck = False
    #     self.is_puck_collected = False
    #     self.sorted_chaos_landings = None  # FIXME if BT interrupts procedure of collecting pucks, will it calculate landings again?

        # self.response_publisher.publish(self.cmd_id + " finished")  FIXME

    def completely_stop(self):
        self.timer.shutdown()
        rospy.loginfo("TN -- Robot has completely stopped")
        rospy.sleep(1.0 / 40)
        self.operating_state = 'waiting for command'
        rospy.loginfo(self.operating_state)
        print(" ")
        print(" ")
        self.response_publisher.publish(self.cmd_id + " finished")

    # def compare_to_update_or_ignore(self, new_obs_of_pucks):
    #
    #     """
    #     Input: [(id, x, y), ...]
    #     Output: format: [(id, x, y), ...]
    #
    #     Old
    #     [(1, x1, y1),
    #      (2, x2, y2)
    #      (3, x3, y3)]
    #
    #      New obs
    #      [(2, x2', y2'),
    #       (3, x3', y3')]
    #
    #     In a new list first puck may be absent for at least three reasons:
    #     - it was collected by our robot
    #     - it is not visible (but it's still there) either because of robot in the line of view or light conditions
    #     - it was collected by enemy robot (and it's not there anymore)
    #
    #     """
    #
    #     assert new_obs_of_pucks.shape[1] == 3
    #
    #     known_ids = self.known_chaos_pucks[:, 0]
    #     known_pucks = self.known_chaos_pucks
    #     for puck in new_obs_of_pucks:
    #         puck_id = puck[0]
    #         if puck_id in known_ids:
    #             ind = known_ids.index(puck_id)  # correct? FIXME
    #             x_new, y_new = puck[1], puck[2]
    #             x_old, y_old = known_pucks[1], known_pucks[2]
    #             if np.sqrt((x_new - x_old) ** 2 + (y_new - y_old) ** 2) > self.coords_threshold:
    #                 self.known_chaos_pucks[ind][1:] = puck[1:]
    #         else:
    #             # wtf happened? blink from sun? wasn't recognised at first time?
    #             self.known_chaos_pucks = np.stack((self.known_chaos_pucks, puck), axis=0)
    #             # self.known_coords_of_pucks.append(puck)

    def calculate_pucks_configuration(self):

        landings_sorted = np.array([])

        if len(self.known_chaos_pucks) == 1:
            landings_sorted = self.calculate_closest_landing_to_point(self.known_chaos_pucks)

        elif len(self.known_chaos_pucks) == 2:
            self.known_chaos_pucks = self.sort_wrt_robot(self.known_chaos_pucks)
            # print(" ")
            # print("only 2 pucks, so sort wrt robot")
            # print(self.known_chaos_pucks)
            # print(" ")
            landings_sorted = self.calculate_landings(self.known_chaos_pucks)  # they are already sorted

        elif len(self.known_chaos_pucks) >= 3:
            self.known_chaos_pucks = self.sort_wrt_robot(self.known_chaos_pucks)
            is_hull_safe_to_approach, coords_sorted_by_angle = self.sort_by_inner_angle_and_check_if_safe(self.known_chaos_pucks)

            if is_hull_safe_to_approach:  # only sharp angles
                # print(" ")
                # print("hull is SAFE to approach, sorted wrt robot")
                # print(self.known_chaos_pucks)
                # print(" ")
                landings_sorted = self.calculate_landings(self.known_chaos_pucks)  # without sorting, because they are already sorted

            elif not is_hull_safe_to_approach:
                self.known_chaos_pucks = coords_sorted_by_angle  # calc vert-angle, sort by angle, return vertices (sorted)
                # print(" ")
                # print("hull is not safe to approach, sorted by angle")
                # print(self.known_chaos_pucks)
                # print(" ")
                landings_sorted = self.calculate_landings(self.known_chaos_pucks)

        return landings_sorted

    def sort_by_inner_angle_and_check_if_safe(self, coords):

        """
        here we first find inner angles
        than sort them with sharpest first
        than compares them with trheshold and returns True False
        :param coords:
        :return:
        """
        poly = None
        is_hull_safe_to_approach = False
        coords = np.array(coords)
        try:
            if len(coords) == 4:
                p1, p2, p3, p4 = map(Point, coords[:, :2])
                poly = Polygon(p1, p2, p3, p4)
            elif len(coords) == 3:
                p1, p2, p3 = map(Point, coords[:, :2])
                poly = Polygon(p1, p2, p3)

            vertices_angles_raw = poly.angles.items()
            vertices_angles = [[(float(k[0].x), float(k[0].y)), radians(degrees(k[1]))] for k in vertices_angles_raw]
            vertices_angles.sort(key=lambda t: t[1])  # sorts by calculated angle
            vertices_angles = np.array(vertices_angles)

            # print("vertices_angles")
            # print(vertices_angles)  # [[(1.12101702, 1.07152553), 0.5125220134823292], ...]
            # print(" ")

            sorted_vertices = np.array([v[0] for v in vertices_angles])
            sorted_vertices[:2] = self.sort_wrt_robot(sorted_vertices[:2])
            # print(" ")
            # print("trick with sorting two sharpes below")
            # print(sorted_vertices)  # [[1.12101702, 1.07152553], ...]
            # print(" ")

            if any(vertices_angles[:3, 1] > self.critical_angle):
                is_hull_safe_to_approach = False
            elif all(vertices_angles[:3, 1] < self.critical_angle):
                is_hull_safe_to_approach = True

        except AttributeError:
            print("pucks lie in straigt line, so no angles")
            indexes_sorted = self.find_indexes_of_outer_points_on_line(coords)
            sorted_vertices = np.array([coords[i] for i in indexes_sorted])
            if len(sorted_vertices) == 4:
                sorted_vertices[:2] = self.sort_wrt_robot(sorted_vertices[:2])  # sort side pucks so robot first collect closest one
            if len(sorted_vertices) == 3:
                sorted_vertices = self.sort_wrt_robot(sorted_vertices)
            # print("sorted_vertices by angles", sorted_vertices)

        return is_hull_safe_to_approach, sorted_vertices

    def calculate_landings(self, coords):
        """
        Calculates offset a hull,
        get line between orig hull and offset
        search intersection point between orbit around centre of each puck and line
        There are many safe landing coords, not just on outer bissectrisa, but for now only the intersection

        Solve system of two equations:
        x**2 + y**2 = r**2, where r is an approch distance to puck for robot
        y = tg(gamma) * x, using calculated slope to get equation of the line
        Orbit inersects line in two points, so we get two landings

        for two pucks there are 4 candidates, for three - 6 candidates
        we calculate to keep only those that lie between puck and it's outer offset

        :return: list of landing coordinates for all angles [[x_l, y_l, gamma], ...]
        """
        landings = []

        mc = np.mean(coords, axis=0)
        offset = list(coords)  # a miserable attempt to copy a list
        offset -= mc  # Normalize the polygon by subtracting the center values from every point.
        offset *= self.scale_factor
        offset += mc

        for orig, ofs in zip(coords, offset):
            dist, _ = calculate_distance(ofs, orig)
            gamma = np.arctan2(dist[1], dist[0])  # and here we calculate slope of outer bis
            gamma = wrap_back(gamma)
            x_candidate1 = np.sqrt(self.approach_dist**2 / (1 + np.tan(gamma)**2))
            x_candidate2 = - np.sqrt(self.approach_dist**2 / (1 + np.tan(gamma)**2))

            y_candidate1 = np.tan(gamma) * x_candidate1
            y_candidate2 = np.tan(gamma) * x_candidate2

            candidate1 = np.array([x_candidate1 + orig[0], y_candidate1 + orig[1], gamma])
            candidate2 = np.array([x_candidate2 + orig[0], y_candidate2 + orig[1], gamma])

            xmin = min(orig[0], ofs[0])
            xmax = max(orig[0], ofs[0])
            ymin = min(orig[1], ofs[1])
            ymax = max(orig[1], ofs[1])

            if all([candidate1[0] >= xmin, candidate1[0] <= xmax, candidate1[1] >= ymin, candidate1[1] <= ymax]):
                landings.append(candidate1)
            elif all([candidate2[0] >= xmin, candidate2[0] <= xmax, candidate2[1] >= ymin, candidate2[1] <= ymax]):
                landings.append(candidate2)
            else:
                print ("ORIG=", orig)
                print ("OFS=", ofs)
                print ("candidate1=", candidate1)
                print ("candidate2=", candidate2)
                print ("SOMETHING WRONG AT LANDING CALCULATIONS")

        landings = np.array(landings)
        # print(" ")
        # print("candidates are: ")
        # print (landings)
        # print(" ")
        return landings

    def calculate_closest_landing_to_point(self, point):
        """
        calculates closest landing to point wrt to current robot position
        :param point: [x,y]
        :return: [xl,yl,thetal]
        """
        point = point[0]

        while not self.update_coords():
            rospy.sleep(0.05)
        print("mistake here _____________________________-----------------")
        print("point is")
        print(point)
        dist, _ = calculate_distance(self.robot_coords, point)  # return deltaX and deltaY coords
        gamma = np.arctan2(dist[1], dist[0])
        point = np.hstack((point, gamma))
        landing = cvt_local2global(self.approach_vec, point)
        landing = landing[np.newaxis, :]
        return landing

    def sort_wrt_robot(self, coords):
        """
        To make operation more quick there is no need to choose the sharpest angle,
        instead better choose the closest one from angles that are marked as safe.
        TODO add id to every landing so it corresponds to color of puck
        :param coords: [[x_l, y_l, gamma], ...]
        :return: format: [[3, 3, 1.57], [1, 1, 3.14], [2, 2, 4, 71], [0, 0, 6.28]]
        """
        coords = np.array(coords)

        if len(coords) == 1:
            return coords
        else:
            while not self.update_coords():
                rospy.sleep(0.05)

            distances_from_robot_to_landings = []
            for coordinate in coords:
                # print("landing is", landing)
                dist, _ = calculate_distance(self.robot_coords, coordinate)  # return deltaX and deltaY coords
                dist_norm = np.linalg.norm(dist)
                distances_from_robot_to_landings.append(dist_norm)
            a = zip(coords, distances_from_robot_to_landings)
            a.sort(key=lambda t: t[1])
            sorted_coords = np.array([v[0] for v in a])
            return sorted_coords

    @staticmethod
    def find_indexes_of_outer_points_on_line(coords):
        """
        finds indexes of minimum and maximum coords, for example there are 4 points in a raw and we'll get outers [1, 2]
        checkk if line is horizontal (look at gamma of any point provided)
        if line of 4 points is horizontal than just take two points with max and min Y-coord,
        else choose points with max and min X-coord
        :param coords: [[x1, y1], [x2, y2], ...]
        :return: indexes [1, 2, 0, 3]
        """
        # print("inside find outer indexes")
        # print("coords")
        # print(coords)
        indexes = []
        is_line_horizontal = [True if coords[0][0] == coords[-1][0] else False]
        # print("is_line_horizontal", is_line_horizontal)
        if is_line_horizontal:
            srt = np.argsort(coords[:, 1], axis=0)
        else:
            srt = np.argsort(coords[:, 0], axis=0)  # argsort returns indexes

        srt = list(srt)

        indexes.append(srt.pop(0))
        indexes.append(srt.pop(-1))
        while len(srt) > 0:
            indexes.append(srt.pop(0))
        # print("indexes_sorted", indexes)

        return indexes

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


if __name__ == "__main__":
    tactics = TacticsNode()
    rospy.spin()
