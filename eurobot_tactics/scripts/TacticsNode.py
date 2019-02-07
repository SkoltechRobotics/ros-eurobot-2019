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
# from sympy import Point, Polygon
# from sympy.geometry import Triangle, Point

# TODO
# from core_functions import calculate_distance
# from core_functions import wrap_angle


"""
This algorithm knows nothing about obstacles and etc, it just generates sorted list of options.
And path-planner will choose from these options according to obstacles around

TODO:
    - when we receive DONE, re-calculate convex hull and generate new sorted list of closest pucks wrt current pose
    - clockwise approaching in order to keep camera view open
    - if all or most of pucks lie separately and safely, just start with the closest one and move counter / clockwise depending on the side
    - need to maintain unique id or order of pucks when deleting 
    - change self.coords to self.robot_coords
    - we should have more vars like known_coords_of_pucks (like _chaos_pucks). It's ok leave by now as is, because we test only separate local zones
    
4 in chaos zone
3 periodic table
6 wall
3 wall
1 ramp

Input: list of n coordinates of pucks, that belong to one local group


Receiving coords from camera each 2-3 seconds
Each time coords are received we call function callback which compares currently known coords and newly received ones
If difference between newly received and known are within accuracy level (threshold), do nothing, else update known coords

With timer we can call 10 times per sec and recalculate environment
and call function move_arc / move line or whatever Egorka implemented

When robot reaches landing position it publishes in response that tusk is ready and than it's time to call Alexey's code to take pucks 

Inside:
    - 
    - calculates the convex hull of these points
    - calculates inner angles and sorts them
    - calculates bissectrisa of the angle and adds offset
    -
Output: ready-steady coordinate and go coordinate

Publishes
"""


def wrap_angle(angle):
    """
    Wraps the given angle to the range [-pi, +pi].
    :param angle: The angle (in rad) to wrap (can be unbounded).
    :return: The wrapped angle (guaranteed to in [-pi, +pi]).
    """
    return (angle + np.pi) % (np.pi * 2) - np.pi


def calculate_distance(coords1, coords2):
    theta_diff = None
    distance_map_frame = coords2[:2] - coords1[:2]
    if len(coords1) == 3 and len(coords2) == 3:
        theta_diff = wrap_angle(coords2[2] - coords1[2])
    return distance_map_frame, theta_diff


class TacticsNode:
    def __init__(self):
        rospy.init_node('TacticsNode', anonymous=True)

        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer)
        self.mutex = Lock()
        self.robot_name = "secondary_robot"

        # self.critical_angle = np.pi * 2/3
        self.approach_dist = 0.12  # meters, distance from robot to puck where robot will try to grab it # FIXME
        self.coords_threshold = 0.01  # meters, this is variance of detecting pucks coords using camera, used in update
        self.ScaleFactor_ = 2  # used in calculating outer bissectrisa for hull's angles
        self.RATE = 10  # FIXME !!!!! self.RATE = rospy.Rate(20)

        # self.robot_coords = np.array([1.2, 1.2, 0])  # FIXME !!!!!!!!!!!!
        self.robot_coords = np.zeros(3)
        self.atoms_collected = 0  # to preliminary calculate our score

        self.sorted_chaos_landing_coordinates = np.array([])
        self.known_coords_of_chaos_pucks = np.array([])  # (id, x, y)
        self.known_coords_of_wall_six_pucks = np.array([])
        self.wall_six_landing_coordinates = np.array([])

        self.operating_state = 'waiting in the start zone'
        self.response_finished = False
        self.puck_collected_and_arm_ready_flag = False
        self.robot_performs_moving = False
        self.robot_is_collecting_puck = False
        self.puck_collected = False

        self.cmd_id = None
        self.cmd_type = None

        self.move_command_publisher = rospy.Publisher('move_command', String, queue_size=10)  # FIXME
        self.pub_cmd = rospy.Publisher('/secondary_robot/stm_command', String, queue_size=1)
        self.pub_response = rospy.Publisher("response", String, queue_size=10)

        rospy.sleep(2)

        self.timer = None

        self.manipulator = Manipulator()

        # coords are published as markers in one list according to 91-92 undistort.py
        rospy.Subscriber("/secondary_robot/pucks", MarkerArray, self.pucks_coords_callback, queue_size=1)
        rospy.Subscriber("/secondary_robot/cmd_tactics", String, self.tactics_callback, queue_size=1)
        rospy.Subscriber('/secondary_robot/response', String, self.callback_response, queue_size=10)

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

        # rospy.loginfo("")
        # rospy.loginfo("=====================================")
        # rospy.loginfo("NEW CMD:\t" + str(data.data))

        # print("data")
        # print(data.markers[0])
        # print("----")

        # FIXME IDs are not guaranteed to be the same from frame to frame
        # FIXME::AL: is_new_pucks()
        # TODO add id of zone!!!!!!!!!!!!
        pucks_new_observation = [(x_.id, x_.pose.position.x, x_.pose.position.y) for x_ in data.markers]
        pucks_new_observation = np.array(pucks_new_observation)
        print('TN -- pucks_new_observation - should be printed ONE TIME only')
        if len(self.known_coords_of_chaos_pucks) == 0:
            self.known_coords_of_chaos_pucks = pucks_new_observation
            coords = self.known_coords_of_chaos_pucks[:, 1:]  # [(x0, y0), (x1, y1), ...]
            hull_indexes = TacticsNode.calculate_convex_hull(coords)  # Calculate pucks configuration once
            hull = [coords[i] for i in hull_indexes]  # using returned indexes to extract hull's coords
            # inner_angles = self.calculate_inner_angles(hull)
            landings_coordinates = self.calculate_possible_chaos_landing_coords(hull)
            self.sorted_chaos_landing_coordinates = self.sort_landing_coords_wrt_current_pose(landings_coordinates)

        # TODO try this in further tests
        # else:
        #     self.compare_to_update_or_ignore(pucks_new_observation)  # in case robot accidentally moved some of pucks

        self.mutex.release()

    def tactics_callback(self, data):

        self.mutex.acquire()

        if self.timer is not None:
            self.timer.shutdown()

        # self.parse_and_update_active_cmd(data)
        rospy.loginfo("")
        rospy.loginfo("=====================================")
        rospy.loginfo("NEW CMD:\t" + str(data.data))

        data_split = data.data.split()
        self.cmd_id = data_split[0]  # FIXME WHAT TO DO WITH CMD_ID
        self.cmd_type = data_split[1]

        rospy.loginfo("Current cmd:\t" + str(self.cmd_type))
        rospy.loginfo("=====================================")
        rospy.loginfo("")

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

        # TODO add Try-Except in case coords are not yet received
        # TODO add obvious state cases like IF STATE ==
        # inside collect_puck we need to provide type of collect: to collect it, to pick and hold or else
        rospy.loginfo('----------- Tactics TIMER_CALLBACK ------------------')

        if self.cmd_type == "collect_chaos" and len(self.known_coords_of_chaos_pucks) > 0:
            # a = self.known_coords_of_chaos_pucks
            # b = self.sorted_chaos_landing_coordinates
            # a, b = self.execute_current_puck_cmd(a, b)
            # this doesn't work
            self.known_coords_of_chaos_pucks, self.sorted_chaos_landing_coordinates = self.execute_current_puck_cmd(self.known_coords_of_chaos_pucks, self.sorted_chaos_landing_coordinates)

        elif self.cmd_type == "collect_wall_six" and len(self.known_coords_of_wall_six_pucks) > 0:
            self.known_coords_of_wall_six_pucks, self.wall_six_landing_coordinates = self.execute_current_puck_cmd(self.known_coords_of_wall_six_pucks, self.wall_six_landing_coordinates)

        # elif self.cmd_type == "collect_wall_3" and len(self.known_coords_of_wall_three_pucks) > 0:
        #     grab_from_wall
        # elif self.cmd_type == "collect_accel_blue_and_hold_up" and len(self.known_coords_of_pucks) > 0:
        #     func()
        # elif self.cmd_type == "take_goldenium_and_hold_middle" and len(self.known_coords_of_pucks) > 0:
        #     func()
        # place_atom_on_weights

        else:
            self.stop_collecting()

    def execute_current_puck_cmd(self, coords, landings):
        """
        0. Get the closest to robot landing from the list
        1. Approach it
        2. Grab and load it
        3. Remove from list of known
        4. Add to list of collected and count points TODO

        we append id of that puck to list of collected pucks and remove it from list of pucks to be collected.
        :return:
        """
        print("-------------------")
        print(self.operating_state)
        print("-------------------")

        # TODO somehow to add checking if it is near starting zone and autom change robot status (for testing)
        if self.operating_state == 'waiting in the start zone' or self.operating_state == 'robot finished job':  # FIXME
            landing = np.array([0.6, 0.6, 0])
            landing = TacticsNode.compose_command(landing, cmd_id='reach_chaos', move_type='move_line')
            self.move_command_publisher.publish(landing)  # TODO add command id
            self.operating_state = 'robot moving to the goal zone'

            print("checking if robot changes flag after moving to the goal zone")
            print(self.response_finished)

        # here we listen for responce 'finished'
        # if self.operating_state == 'robot is moving to the goal zone' and not self.robot_reached_goal_and_stays_still_flag:
        #     print("wait")
        #     pass

        # TRUE fron response

        if self.operating_state == 'robot moving to the goal zone' and self.response_finished:
            self.operating_state = 'robot close to goal zone'
            print("changing state to ---robot close to goal zone---")
            print("self.response_finished is True ???", self.response_finished)
            print("self.robot_performs_moving is False ??", self.robot_performs_moving)
            # self.response_finished = False

        if self.operating_state == 'robot close to goal zone' and self.response_finished is True and self.robot_performs_moving is False:
            self.robot_performs_moving = True
            self.response_finished = False
            # self.operating_state = 'robot is collecting chaos zone'
            landing = landings[0]
            landing = TacticsNode.compose_command(landing, cmd_id='empty_chaos', move_type='move_arc')
            self.move_command_publisher.publish(landing)
            print("checking if robot changes flag when already reached zone")
            print(self.response_finished)
            # TODO here we already reached first landing coord and are asking robot to reach it one more time.
            # will it respone 'finished' this time?

        if self.operating_state == 'robot close to goal zone' and self.response_finished:
            self.operating_state = 'robot reached goal zone'

        # here when robot reaches the goal MotionPlannerNode will publish in response topic "finished" and in this code
        # callback_response will fire and change self.robot_reached_goal_flag to True
        if self.operating_state == 'robot reached goal zone' and self.response_finished is True and self.robot_is_collecting_puck is False:
            self.robot_is_collecting_puck = True
            print("----------------------")
            print("robot collecting pucks")
            print('robot reached goal zone')
            # self.puck_collected = self.manipulator.collect_puck()  # TODO load_inside=False
            self.puck_collected = TacticsNode.imitate_manipulator()

            # TODO inside collect_puck we need to provide type of collect: to collect it, to pick and hold or else

        if self.puck_collected is True:
            self.atoms_collected += 1
            print("TN -- atoms collected, count: ", self.atoms_collected)
            coords = np.delete(coords, 0, axis=0)
            landings = np.delete(landings, 0, axis=0)
            landings = self.sort_landing_coords_wrt_current_pose(landings)

            # FIXME path_planner should decide which puck to collect and to remove from known
            print("TN -- ---known coords AFTER deleting ----")
            print(landings)
            print(" ")
            print("TN -- pucks left: ", len(coords))

            self.robot_performs_moving = False
            self.response_finished = True
            self.robot_is_collecting_puck = False
            self.puck_collected = False
            self.operating_state = 'robot close to goal zone'

        # if len(coords) == 0:
        #     self.operating_state = 'robot collected all known pucks, finish'
        #     self.stop_collecting()

        return coords, landings

    def callback_response(self, data):
        # here when robot reaches the goal MotionPlannerNode will publish in response topic "finished" and in this code
        # callback_response will fire and change self.robot_reached_goal_flag to True
        if data.data == 'finished':  # TODO change so that response is marked by cmd_id ''
            self.response_finished = True

    @staticmethod
    def compose_command(landing, cmd_id, move_type):  # TODO here input cmd_type move_arc or move_line
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
        rospy.loginfo('-----TN----NEW COMMAND COMPOSED --------')
        rospy.loginfo(command)
        return command

    @staticmethod
    def imitate_manipulator():
        return True

    def stop_collecting(self):
        self.timer.shutdown()
        rospy.loginfo("TN -- Robot has stopped collecting pucks.")
        rospy.sleep(1.0 / 40)
        self.operating_state = 'robot finished job'
        print(self.operating_state)
        self.pub_response.publish("stopped_collecting")

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

        known_ids = self.known_coords_of_chaos_pucks[:, 0]
        known_pucks = self.known_coords_of_chaos_pucks
        for puck in new_obs_of_pucks:
            puck_id = puck[0]
            if puck_id in known_ids:
                ind = known_ids.index(puck_id)  # correct? FIXME
                x_new, y_new = puck[1], puck[2]
                x_old, y_old = known_pucks[1], known_pucks[2]
                if np.sqrt((x_new - x_old) ** 2 + (y_new - y_old) ** 2) > self.coords_threshold:
                    self.known_coords_of_chaos_pucks[ind][1:] = puck[1:]
            else:
                # wtf happened? blink from sun? wasn't recognised at first time?
                self.known_coords_of_chaos_pucks = np.stack((self.known_coords_of_chaos_pucks, puck), axis=0)
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

    # def calculate_inner_angles(self, hull):
    #     """
    #     Input:
    #     hull
    #
    #     Calculates inner angles of a convex hull
    #     Sorts them
    #     optional (Removes angles that are below threshold safe level)
    #
    #     :return: sorted list of angles starting with sharpest one
    #     """
    #     # p1, p2, p3, p4 = map(Point, [(0, 0), (1, 0), (5, 1), (0, 1)])
    #     # p1, p2, p3, p4 = map(Point, hull)  # need to unpack hull?
    #     poly = Polygon(hull)
    #     inner_angles = np.array(poly.angles)
    #
    #     # angles bigger that critical value are marked with big value and are not considered
    #     # inner_safe_angles = np.putmask(inner_angles, (inner_angles > self.critical_angle), 1000)
    #     inner_safe_angles = np.where(inner_angles > self.critical_angle, None, inner_angles)
    #     inner_safe_angles = zip(hull, inner_safe_angles)  # returns for ex [((x0, y0), pi/3), ((x1, y1), pi/2), ((x2, y2), pi/6), ((x3, y3), None)]
    #     inner_safe_angles.sort(key=lambda t: t[1])  # sorts by calculated angle. But how will 'None' perform? TODO reverse?
    #
    #     # TODO need to maintain unique id or order of pucks when deleting
    #
    #     return inner_safe_angles

    def calculate_possible_chaos_landing_coords(self, hull):
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

        # assert np.all(inner_safe_angles[:, 2]) > 0

        # this method can be applied when pucks are close to each other
        # calculate offset
        # calc angle of bissectrisa which it outer wrt original hull
        # calc point that is const vector away from centre of puck

        landings_coordinates = []
        mc = np.mean(hull, axis=0)
        offset = list(hull)  # a miserable attempt to copy a list
        offset -= mc  # Normalize the polygon by subtracting the center values from every point.
        offset *= self.ScaleFactor_
        offset += mc

        for orig, ofs in zip(hull, offset):

            dist, _ = calculate_distance(ofs, orig)
            gamma = np.arctan2(dist[1], dist[0])  # and here we calculate slope of outer bis

            x_landing1 = np.sqrt(self.approach_dist**2 / (1 + np.tan(gamma)**2))
            x_landing2 = - np.sqrt(self.approach_dist**2 / (1 + np.tan(gamma)**2))

            y_landing1 = np.tan(gamma) * x_landing1
            y_landing2 = np.tan(gamma) * x_landing2

            landing1 = np.array([x_landing1 + orig[0], y_landing1 + orig[1], gamma])
            landing2 = np.array([x_landing2 + orig[0], y_landing2 + orig[1], gamma])

            landings_coordinates.append(landing1)
            landings_coordinates.append(landing2)

        landings = np.array(landings_coordinates)
        landing_indexes = TacticsNode.calculate_convex_hull(landings[:, :2])  # to get rid off
        landings_coordinates = [landings[i] for i in landing_indexes]

        # another method is for situation where pucks are safely away from each other
        # and we can approach them from any angle (should choose closest one)
        # we can start by calculating area of a hull
        # safe_orbit =

        return landings_coordinates

    def sort_landing_coords_wrt_current_pose(self, landings_coordinates):
        """
        To make operation more quick there is no need to choose the sharpest angle,
        instead better choose the closest one from angles that are marked as safe.
        TODO add id to every landing so it corresponds to color of puck
        :param landings_coordinates: [[x_l, y_l, gamma], ...]
        :return: format: [[3, 3, 1.57], [1, 1, 3.14], [2, 2, 4, 71], [0, 0, 6.28]]
        """
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

    # def sort_pucks_coords_wrt_current_pose(self):
    #     """
    #     To make operation more quick there is no need to choose the sharpest angle,
    #     instead better choose the closest one from angles that are marked as safe.
    #     :return: format [[(id0, x0, y0), d0], ...]
    #     """
    #
    #     distances_from_robot_to_pucks = []
    #     for puck in self.known_coords_of_pucks:
    #         dist, _ = calculate_distance(self.robot_coords, puck)  # return deltaX and deltaY coords
    #         dist_norm = np.linalg.norm(dist)
    #         distances_from_robot_to_pucks.append(dist_norm)
    #     distances_from_robot_to_pucks = np.array(distances_from_robot_to_pucks)
    #     pucks_wrt_robot_sorted = zip(self.known_coords_of_pucks, distances_from_robot_to_pucks)
    #     pucks_wrt_robot_sorted.sort(key=lambda t: t[1])
    #     [v[0] for v in a]
    #     return pucks_wrt_robot_sorted

    def update_coords(self):
        try:
            trans = self.tfBuffer.lookup_transform('map', self.robot_name, rospy.Time())
            q = [trans.transform.rotation.x,
                 trans.transform.rotation.y,
                 trans.transform.rotation.z,
                 trans.transform.rotation.w]
            angle = euler_from_quaternion(q)[2] % (2 * np.pi)

            self.robot_coords = np.array([trans.transform.translation.x, trans.transform.translation.y, angle])
            rospy.loginfo("TN -- Robot coords:\t" + str(self.robot_coords))
            return True
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as msg:
            rospy.logwarn(str(msg))
            return False


if __name__ == "__main__":
    tactics = TacticsNode()
    rospy.spin()
