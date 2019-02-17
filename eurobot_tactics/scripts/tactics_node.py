#!/usr/bin/env python
# coding: utf-8
# import sys
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
# from sympy.geometry import Triangle, Point
# sys.path.append('/home/safoex/eurobot2019_ws/src/ros-eurobot-2019/libs')  # FIXME

from core_functions import calculate_distance
# from core_functions import wrap_angle
from core_functions import wrap_back
from core_functions import cvt_local2global
from math import radians, degrees

"""
This algorithm knows nothing about obstacles and etc, it just generates sorted list of options.
And path-planner will choose from these options according to obstacles around

TODO:
    - clockwise approaching in order to keep camera view open
    - if all or most of pucks lie separately and safely, just start with the closest one and move counter / clockwise depending on the side
    - need to maintain unique id or order of pucks when deleting 
    - FIXME path_planner should decide which puck to collect and to remove from known
    - in case we parse only first frame and while collecting pucks robot moves some of them, applying sort func is useless 
    - Alexey need to parse particular local zones
    - FIXME !!!!!!!! need to remove coords from known list by ID, and NOT by sorting them wrt to robot -- otherwise it can lead to troubles

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

        # self.critical_angle = np.pi * 2/3
        self.area_threshold = 0.03  # FIXME make it ydelnaya area na puck
        self.approach_dist = 0.11  # meters, distance from robot to puck where robot will try to grab it # FIXME
        self.approach_vec = np.array([-0.11, 0, 0])
        self.drive_back_dist = np.array([-0.05, 0, 0])
        self.coords_threshold = 0.01  # meters, this is variance of detecting pucks coords using camera, used in update
        self.scale_factor = 10  # used in calculating outer bissectrisa for hull's angles
        self.RATE = 10

        self.robot_coords = np.zeros(3)
        self.active_goal = None
        self.goal_landing = None
        self.atoms_collected = 0  # to preliminary calculate our score

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

        self.cmd_id = None
        self.cmd_type = None

        # publishers
        self.move_command_publisher = rospy.Publisher('move_command', String, queue_size=10)
        self.stm_command_publisher = rospy.Publisher('stm_command', String, queue_size=1)
        self.response_publisher = rospy.Publisher("response", String, queue_size=10)

        rospy.sleep(2)

        self.timer = None

        self.stm_command_publisher.publish("null 32")
        self.manipulator = Manipulator()

        # coords are published as markers in one list according to 91-92 undistort.py
        # FIXME
        #  add /secondary_robot when in simulator, remove when on robot!!!!!!!!!!!

        rospy.Subscriber("/secondary_robot/pucks", MarkerArray, self.pucks_coords_callback, queue_size=1)
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

        # FIXME IDs are not guaranteed to be the same from frame to frame
        # TODO add id of zone!!!!!!!!!!!!
        new_observation_pucks = [(x_.id, x_.pose.position.x, x_.pose.position.y) for x_ in data.markers]
        new_observation_pucks = np.array(new_observation_pucks)
        print('TN -- new_observation_pucks')
        print(new_observation_pucks)

        self.known_chaos_pucks = np.array([])  # FIXME this is just for tests!

        if len(self.known_chaos_pucks) == 0:
            self.known_chaos_pucks = new_observation_pucks[:, 1:]  # [array(x0, y0), array(x1, y1), ...]
            # coords = self.known_chaos_pucks[:, 1:]  # [(x0, y0), (x1, y1), ...]
            # self.known_chaos_pucks = self.calculate_pucks_configuration(coords)

        # else:
        #     self.compare_to_update_or_ignore(new_observation_pucks)  # in case robot accidentally moved some of pucks

        self.mutex.release()

    def calculate_pucks_configuration(self):
        # self.mutex.acquire()  STOPS WORKING

        coords = self.known_chaos_pucks  # [array(x0, y0), array(x1, y1), ...]
        hull_indexes = self.calculate_convex_hull(coords)  # Calculate pucks configuration once
        hull = [coords[i] for i in hull_indexes]  # using returned indexes to extract hull's coords

        print("hull is")
        print(hull)  # [array([ 0.98,  0.82]), array([ 1.06,  0.98]), array([ 0.98,  0.9 ])]

        self.sorted_chaos_landings = self.calculate_sort_chaos_landing_coords(hull)
        self.known_chaos_pucks = self.sort_coords_wrt_robot()  # FIXME !!!!!!!! need to remove coords from known list by ID, and NOT by sorting them wrt to robot

        print(" ")
        print("sorted known_chaos_pucks")
        print(self.known_chaos_pucks)
        print(" ")
        print("TN: NEW SORTED LANDINGS CALCULATED")
        print(self.sorted_chaos_landings)
        print(" ")
        # self.mutex.release()
        return coords

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
        self.cmd_id = data_split[0]  # FIXME WHAT TO DO WITH CMD_ID
        self.cmd_type = data_split[1]

        self.timer = rospy.Timer(rospy.Duration(1.0 / self.RATE), self.timer_callback)  # FIXME ask Misha

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
        # rospy.loginfo(' ')
        # rospy.loginfo('Tactics TIMER_CALLBACK')
        # rospy.loginfo('There are ' + str(len(self.known_chaos_pucks)) + ' pucks on the field')

        if self.cmd_type == "collect_chaos" and len(self.known_chaos_pucks) > 0:
            self.execute_puck_cmd()

        # elif self.cmd_type == "collect_wall_six" and len(self.known_wall6_pucks) > 0:
            # self.known_wall6_pucks, self.wall_six_landing = self.execute_puck_cmd(self.known_wall6_pucks, self.wall_six_landing)

        # elif self.cmd_type == "collect_wall_3" and len(self.known_coords_of_wall_three_pucks) > 0:
        #     grab_from_wall
        # elif self.cmd_type == "collect_accel_blue_and_hold_up" and len(self.known_coords_of_pucks) > 0:
        #     func()
        # TODO inside collect_puck we need to provide type of collect: to collect it, to pick and hold or else
        # elif self.cmd_type == "take_goldenium_and_hold_middle" and len(self.known_coords_of_pucks) > 0:
        #     func()
        # place_atom_on_weights

        else:
            self.stop_collecting()

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
        # print("-------------------")
        # print("TN: operating status ", self.operating_state)
        # print("TN: active goal ", self.active_goal)
        # print("-------------------")

        if self.operating_state == 'waiting for command' and not self.is_robot_moving:
            rospy.loginfo(self.operating_state)
            self.calculate_pucks_configuration()

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

        if self.operating_state == 'nearest LANDING approached' and not self.is_robot_collecting_puck:
            self.goal_landing = None
            # self.is_puck_collected = self.manipulator.collect_puck()
            self.stm_command_publisher.publish("null 34")  # TODO what's this cmd about?
            self.is_robot_collecting_puck = True
            self.operating_state = 'collecting puck'
            rospy.loginfo(self.operating_state)
            self.imitate_manipulator()

        if self.operating_state == 'collecting puck' and self.is_puck_collected:
            self.operating_state = 'puck successfully collected'
            rospy.loginfo("Wohoo! " + str(self.known_chaos_pucks[0]) + " " + str(self.operating_state))
            self.is_robot_collecting_puck = False
            self.atoms_collected += 1
            print(" ")
            print("now delete puck", self.known_chaos_pucks[0])
            print(" ")
            self.known_chaos_pucks = np.delete(self.known_chaos_pucks, 0, axis=0)
            rospy.loginfo('TN: pucks left: ' + str(len(self.known_chaos_pucks)))

        if self.operating_state == 'puck successfully collected':
            self.operating_state = 'waiting for command'
            rospy.loginfo(self.operating_state)
            self.is_puck_collected = False
            self.is_robot_moving = False
            self.active_goal = None
            self.sorted_chaos_landings = None

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

    def stop_collecting(self):
        self.timer.shutdown()
        rospy.loginfo("TN -- Robot has stopped collecting pucks.")
        rospy.sleep(1.0 / 40)
        print(" ")
        print(" ")

        self.is_robot_moving = False
        self.active_goal = None
        self.is_finished = True
        self.is_robot_collecting_puck = False
        self.is_puck_collected = False
        self.sorted_chaos_landings = None  # FIXME if BT interrupts procedure of collecting pucks, will it calculate landings again?
        self.operating_state = 'waiting for command'

        print(self.operating_state)
        print(" ")
        print(" ")
        self.response_publisher.publish(self.cmd_id + " finished")

    def compare_to_update_or_ignore(self, new_obs_of_pucks):

        """
        Input: [(id, x, y), ...]
        Output: format: [(id, x, y), ...]

        Old
        [(1, x1, y1),
         (2, x2, y2)
         (3, x3, y3)]

         New obs
         [(2, x2', y2'),
          (3, x3', y3')]

        In a new list first puck may be absent for at least three reasons:
        - it was collected by our robot
        - it is not visible (but it's still there) either because of robot in the line of view or light conditions
        - it was collected by enemy robot (and it's not there anymore)

        """

        assert new_obs_of_pucks.shape[1] == 3

        known_ids = self.known_chaos_pucks[:, 0]
        known_pucks = self.known_chaos_pucks
        for puck in new_obs_of_pucks:
            puck_id = puck[0]
            if puck_id in known_ids:
                ind = known_ids.index(puck_id)  # correct? FIXME
                x_new, y_new = puck[1], puck[2]
                x_old, y_old = known_pucks[1], known_pucks[2]
                if np.sqrt((x_new - x_old) ** 2 + (y_new - y_old) ** 2) > self.coords_threshold:
                    self.known_chaos_pucks[ind][1:] = puck[1:]
            else:
                # wtf happened? blink from sun? wasn't recognised at first time?
                self.known_chaos_pucks = np.stack((self.known_chaos_pucks, puck), axis=0)
                # self.known_coords_of_pucks.append(puck)

    @staticmethod
    def rotate(a, b, c):
        return (b[0] - a[0]) * (c[1] - b[1]) - (b[1] - a[1]) * (c[0] - b[0])

    @staticmethod
    def calculate_convex_hull(coords):
        """
        jarvis march
        input: [(x0, y0), (x1, y1), ...]
        :return: convex hull, INDEXES of coordinates in input list, not coords themselves, TODO counter or clockwise, need to figure out
        """
        n = len(coords)
        p = range(n)
        # start point
        for i in range(1, n):
            if coords[p[i]][0] < coords[p[0]][0]:
                p[i], p[0] = p[0], p[i]
        h = [p[0]]
        del p[0]
        p.append(h[0])
        while True:
            right = 0
            for i in range(1, len(p)):
                if TacticsNode.rotate(coords[h[-1]], coords[p[right]], coords[p[i]]) < 0:  # FIXME
                    right = i
            if p[right] == h[0]:
                break
            else:
                h.append(p[right])
                del p[right]

        # hull = [coords[i] for i in h]  # this was used to extract coords from input list using calculated indexes
        return h

    def calculate_sort_chaos_landing_coords(self, hull):
        """
        Calculates offset a hull,
        get line between orig hull and offset
        search intersection point between orbit around centre of each puck and line
        There are many safe landing coords, not just on outer bissectrisa, but for now only the intersection

        Solve system of two equations:
        x**2 + y**2 = r**2, where r is an approch distance to puck for robot
        y = tg(gamma) * x, using calculated slope to get equation of the line
        Orbit inersects line in two points, so we get two landings

        :return: list of landing coordinates for all angles [[x_l, y_l, gamma], ...]
        """
        landings_coordinates = []
        if len(hull) >= 2:
            candidates = self.calculate_candidates(hull)

            if len(hull) >= 3:
                landing_indexes = self.calculate_convex_hull(candidates[:, :2])  # convex hull of landings this time
                landings_coordinates = [candidates[i] for i in landing_indexes]

            elif len(hull) == 2:
                landing_indexes = self.find_indexes_of_outer_points_on_line(candidates)
                landings_coordinates = [candidates[i] for i in landing_indexes]

            # for both options we sort landings that we found and return to main logic function
            landings_coordinates = self.sort_landing_coords_wrt_current_pose(landings_coordinates)

        elif len(hull) == 1:
            landings_coordinates = self.calculate_closest_landing_to_point(hull)

        return landings_coordinates

    @staticmethod
    def polygon_area(hull):
        x = np.array([h[0] for h in hull])
        y = np.array([h[1] for h in hull])  # because slicing doesn't work for some reason
        # coordinate shift
        x_ = x - np.mean(x)
        y_ = y - np.mean(y)
        correction = x_[-1] * y_[0] - y_[-1] * x_[0]
        main_area = np.dot(x_[:-1], y_[1:]) - np.dot(y_[:-1], x_[1:])
        return 0.5*np.abs(main_area + correction)

    def calculate_candidates(self, hull):
        candidates = []
        mc = np.mean(hull, axis=0)
        offset = list(hull)  # a miserable attempt to copy a list
        offset -= mc  # Normalize the polygon by subtracting the center values from every point.
        offset *= self.scale_factor
        offset += mc

        for orig, ofs in zip(hull, offset):
            dist, _ = calculate_distance(ofs, orig)
            gamma = np.arctan2(dist[1], dist[0])  # and here we calculate slope of outer bis
            gamma = wrap_back(gamma)
            x_landing1 = np.sqrt(self.approach_dist**2 / (1 + np.tan(gamma)**2))
            x_landing2 = - np.sqrt(self.approach_dist**2 / (1 + np.tan(gamma)**2))

            y_landing1 = np.tan(gamma) * x_landing1
            y_landing2 = np.tan(gamma) * x_landing2

            candidate1 = np.array([x_landing1 + orig[0], y_landing1 + orig[1], gamma])
            candidate2 = np.array([x_landing2 + orig[0], y_landing2 + orig[1], gamma])

            xmin = min(orig[0], ofs[0])
            xmax = max(orig[0], ofs[0])
            ymin = min(orig[1], ofs[1])
            ymax = max(orig[1], ofs[1])

            if candidate1[0] >= xmin and candidate1[0] <= xmax and candidate1[1] >= ymin and candidate1[1] <= ymax:
                candidates.append(candidate1)
            elif candidate2[0] >= xmin and candidate2[0] <= xmax and candidate2[1] >= ymin and candidate2[1] <= ymax:
                candidates.append(candidate2)
            else:
                print ("ORIG=", orig)
                print ("OFS=", ofs)
                print ("candidate1=", candidate1)
                print ("candidate2=", candidate2)
                print ("SOMETHING WRONG AT LANDING CALCULATIONS")

        candidates = np.array(candidates)  # for two pucks there are 4 candidates, for three - 6 candidates
        print(" ")
        print("candidates are: ")
        print (candidates)
        print(" ")
        return candidates

    def calculate_closest_landing_to_point(self, point):
        """
        calculates closest landing to point wrt to current robot position
        :param point: [x,y]
        :return: [xl,yl,thetal]
        """
        point = point[0]

        while not self.update_coords():
            rospy.sleep(0.05)

        dist, _ = calculate_distance(self.robot_coords, point)  # return deltaX and deltaY coords
        gamma = np.arctan2(dist[1], dist[0])
        point = np.hstack((point, gamma))
        landing = cvt_local2global(self.approach_vec, point)
        landing = landing[np.newaxis, :]
        return landing

    @staticmethod
    def find_indexes_of_outer_points_on_line(coords):
        """
        finds indexes of minimum and maximum coords, for example there are 4 points in a raw and we'll get outers [0, 3]
        checkk if line is horizontal (look at gamma of any point provided)
        if line of 4 points is horizontal than just take two points with max and min X-coord,
        else choose points with max and min Y-coord
        :param coords: [[x1, y1], [x2, y2], ...]
        :return: indexes [0, 3]
        """
        indexes = []
        is_line_horizontal = [True if abs(coords[0][2]) < 1e-4 else False]  # FIXME
        if is_line_horizontal:
            srt = np.argsort(coords[:, 0], axis=0)
        else:
            srt = np.argsort(coords[:, 1], axis=0)

        indexes.append(srt[0])
        indexes.append(srt[-1])

        return indexes

    def sort_landing_coords_wrt_current_pose(self, landings_coordinates):
        """
        To make operation more quick there is no need to choose the sharpest angle,
        instead better choose the closest one from angles that are marked as safe.
        TODO add id to every landing so it corresponds to color of puck
        :param landings_coordinates: [[x_l, y_l, gamma], ...]
        :return: format: [[3, 3, 1.57], [1, 1, 3.14], [2, 2, 4, 71], [0, 0, 6.28]]
        """
        landings_coordinates = np.array(landings_coordinates)

        if len(landings_coordinates) == 1:
            return landings_coordinates
        else:
            hull_area = self.polygon_area(landings_coordinates)
            print(" ")
            print("hull area is", hull_area)
            print(" ")
            # if pucks are lying on a line or nearly on a line, than we start collecting from the sharpest angle
            if hull_area <= self.area_threshold and len(landings_coordinates) > 2:
                sorted_coords = self.calculate_and_sort_by_inner_angles(landings_coordinates)

            # if pucks are lying far from each other, start collecting with the closest puck
            else:
                while not self.update_coords():
                    rospy.sleep(0.05)

                distances_from_robot_to_landings = []
                for landing in landings_coordinates:
                    # print("landing is", landing)
                    dist, _ = calculate_distance(self.robot_coords, landing)  # return deltaX and deltaY coords
                    dist_norm = np.linalg.norm(dist)
                    distances_from_robot_to_landings.append(dist_norm)
                a = zip(landings_coordinates, distances_from_robot_to_landings)
                a.sort(key=lambda t: t[1])
                sorted_coords = [v[0] for v in a]
            return sorted_coords

    @staticmethod
    def calculate_and_sort_by_inner_angles(landings_coordinates):
        """
        Input:
        hull [array([ 0.9,  0.9]), array([ 1.06,  0.98]), array([ 0.98,  0.82]), array([ 0.98,  0.9 ])]

        Calculates inner angles of a convex hull
        Sorts them
        TODO Remove DANGEROUS angles that are below threshold safe level

        :return: list of vertices sorted by their angles starting with sharpest one
        [array([ 1.06,  0.98]),
         array([ 0.98,  0.82]),
         array([ 0.9,  0.9]),
         array([ 0.98,  0.9 ])]
        """
        print("INSIDE INNER ANGLES FUNC")
        poly = None
        landings_coordinates = np.array(landings_coordinates)
        if len(landings_coordinates) == 4:
            p1, p2, p3, p4 = map(Point, landings_coordinates[:, :2])
            poly = Polygon(p1, p2, p3, p4)
        elif len(landings_coordinates) == 3:
            p1, p2, p3 = map(Point, landings_coordinates[:, :2])
            poly = Polygon(p1, p2, p3)
        # print("poly")
        # print(poly)
        # print(" ")
        inner_angles = np.array([poly.angles[k] for k in poly.angles])  # [acos(4/5), 3*pi/2, ...]
        inner_angles_rad = [radians(degrees(k)) for k in inner_angles]
        print("landings_coordinates")
        print(landings_coordinates)
        print(" ")

        print("inner_angles_rad")
        print(inner_angles_rad)
        print(" ")

        vertices_angles = zip(landings_coordinates, inner_angles_rad)  # returns [(array([ 0.9,  0.9]), acos(4/5)), ...]
        vertices_angles.sort(key=lambda t: t[1])  # sorts by calculated angle
        print(" ")
        print("vertices_angles sorted")
        print(vertices_angles)
        print(" ")
        sorted_vertices_by_angle = [v[0] for v in vertices_angles]
        # TODO need to maintain unique id or order of pucks when deleting
        return sorted_vertices_by_angle

    def sort_coords_wrt_robot(self):
        """
        To make operation more quick there is no need to choose the sharpest angle,
        instead better choose the closest one from angles that are marked as safe.
        :return: format [[(id0, x0, y0), d0], ...]  NOT ANYMORE
        """
        # TODO remove this function and generalise function above
        if len(self.known_chaos_pucks) == 1:
            return self.known_chaos_pucks
        else:
            while not self.update_coords():
                rospy.sleep(0.05)

        distances_from_robot_to_pucks = []
        for puck in self.known_chaos_pucks:
            dist, _ = calculate_distance(self.robot_coords, puck)  # return deltaX and deltaY coords
            dist_norm = np.linalg.norm(dist)
            distances_from_robot_to_pucks.append(dist_norm)
        distances_from_robot_to_pucks = np.array(distances_from_robot_to_pucks)
        a = zip(self.known_chaos_pucks, distances_from_robot_to_pucks)
        a.sort(key=lambda t: t[1])  # reverse=True
        pucks_wrt_robot_sorted = [v[0] for v in a]
        return pucks_wrt_robot_sorted

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
