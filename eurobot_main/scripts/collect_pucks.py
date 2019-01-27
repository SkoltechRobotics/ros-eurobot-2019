import rospy
import numpy as np
import tf2_ros
import time
from threading import Lock

from sympy import Point, Polygon
from sympy.geometry import Triangle, Point

from std_msgs.msg import String
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from threading import Lock
from std_msgs.msg import Int32MultiArray


# TODO
# from core_functions import calculate_distance
# from core_functions import wrap_angle


"""
This algorithm knows nothing about obstacles and etc, it just generates sorted list of options.
And path-planner will choose from these options according to obstacles around

TODO:
    - publish in response that current puck was collected and robot is ready for next one
    - when atom is grabbed and collected, publish DONE in response -- YIELD ?
    - when we receive DONE, re-calculate convex hull and generate new sorted list of closest pucks wrt current pose
    - when we get a response that puck is collected, we remove it's coord from list of known coords
    - clockwise approaching in order to keep camera view open
    - if all or most of pucks lie separately and safely, just start with the closest one and move counter / clockwise depending on the side
    - need to maintain unique id or order of pucks when deleting 
    - critical angle is not the only condition two decide. If pucks far away from each other, just make sure that robot doesn't touch others while 
    approaching the closest puck
    - change self.coords to self.robot_coords
    - how to call update_coords from MotionPlannerNode ?  I need robot_coords
    - flag --> arm_ready_flag 
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

With timer we can call 10 times per sec and recalc environment
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

# Проверить, что робот остановился.
# Информация о том, что робот отсановился, публикуется в топик response

# Проверить, что цель достигнута.
# Подписаться на топик, в который будут публиковаться координаты атомов с камеры
#
# Проверить, что манипулятор в исходной позиции
#
# Проверить, отпустить, проверить - это одна команда
#
# Опустить манипулятор
# Проверить, что манипулятор опустился
# Включить насос (на сколько?)
# Поднять манипулятор до конца
# Подпереть атом граблей
# Выключить насос


def wrap_angle(angle):
    """
    Wraps the given angle to the range [-pi, +pi].
    :param angle: The angle (in rad) to wrap (can be unbounded).
    :return: The wrapped angle (guaranteed to in [-pi, +pi]).
    """
    return (angle + np.pi) % (np.pi * 2) - np.pi


def calculate_distance(coords1, coords2):

    # assert
    # TODO

    distance_map_frame = coords2[:2] - coords1[:2]
    theta_diff = wrap_angle(coords2[2] - coords1[2])
    return distance_map_frame, theta_diff


class TacticsNode:

    def __init__(self):
        rospy.init_node('TacticsNode')
        self.critical_angle = np.pi * 2/3
        self.approach_dist = 0.1  # meters, distance from robot to puck where robot will try to grab it # FIXME
        self.robot_coords = None
        self.mutex = Lock()
        self.coords_threshold = 0.01 # meters, this is variance of detecting pucks coords using camera, used in update
        self.sorted_landing_coordinates = None  # FIXME
        self.known_coords_of_pucks = np.array([])  # (id, x, y)
        self.puck_status = None
        self.ScaleFactor_ = 2  # used in calculating outer bissectrisa for hull's angles
        # status of a puck we're currently working on, read from response.
        # When status becomes "collected" -- remove from self.known_coords_of_pucks
        self.atoms_placed = 0  # to preliminary calculate our score
        self.arm_ready_flag = False
        self.RATE = rospy.Rate(20)
        self.cmd_id = None
        self.cmd_type = None

        # FIXME
        self.move_command_publisher = rospy.Publisher('move_command', String, queue_size=10)
        self.pub_cmd = rospy.Publisher('/secondary_robot/stm_command', String, queue_size=1)
        self.pub_response = rospy.Publisher("response", String, queue_size=10)

        rospy.sleep(2)

        self.timer = None

        # coords are published as markers in one list according to 91-92 undistort.py
        rospy.Subscriber("pucks", MarkerArray, self.pucks_coords_callback, queue_size=1)
        rospy.Subscriber("cmd_tactics", String, self.tactics_callback, queue_size=1)

    def pucks_coords_callback(self, data):
        """
        # TODO
        # implement comparing with threshold,
        # if newly received coord differs from old known one less than threshold level,
        # than ignore it and continue collecting pucks.
        # Else change coord of that puck and recalculate

        In first step we just write received coords to list of known pucks,
        in further steps we compare two lists and decide whether to update it or ignore

        :param self:
        :param data:
        :return:
        """
        self.mutex.acquire()

        rospy.loginfo("")
        rospy.loginfo("=====================================")
        rospy.loginfo("NEW CMD:\t" + str(data.data))

        print("data")
        print(data)

        # TODO all pucks must have unique id so we can compare their previously known coords and newly received

        # FIXME from observation to observation ID are not guaranteed to be the same

        new_obs_of_pucks = [(x_.id, x_.pose.position.x, x_.pose.position.y) for x_ in data]
        new_obs_of_pucks = np.array(new_obs_of_pucks)

        if len(self.known_coords_of_pucks) == 0:
            self.known_coords_of_pucks = new_obs_of_pucks

        # TODO try this in further tests
        # else:
        #     self.compare_to_update_or_ignore(new_obs_of_pucks)  # in case robot accidentally moved some of pucks

        self.mutex.release()

    def tactics_callback(self, data):
        """
        When BT publishes command something like "collect atoms in Chaos Zone", we start calculating and updating configuration of pucks
        :param data: cmd_type can be one of these:
        - collect_chaos
        - if in chaos taken blue - put it separately
        - take_from_wall_and_hold_above (
        - take_goldenium_and_hold_middle
        - collect_accel_blue (diff height, maybe we need to hold it up for a while)
        :return:
        """
        self.mutex.acquire()

        rospy.loginfo("")
        rospy.loginfo("=====================================")
        rospy.loginfo("NEW CMD:\t" + str(data.data))

        if self.timer is not None:
            self.timer.shutdown()

        data_split = data.data.split()
        cmd_id = data_split[0]
        cmd_type = data_split[1]
        # TODO cmd_args = data_split[2:]
        rospy.loginfo(cmd_type)

        self.update_task_status(cmd_id, cmd_type)
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.RATE), self.timer_callback, oneshot=False)  # FIXME ask Misha

        self.mutex.release()

    def update_task_status(self, cmd_id, cmd_type):
        rospy.loginfo("Current cmd:\t" + str(cmd_type))
        rospy.loginfo("=====================================")
        rospy.loginfo("")

        self.cmd_id = cmd_id
        self.cmd_type = cmd_type

    def timer_callback(self, event):

        # TODO add Try-Except in case coords are not yet received

        # TODO
        if self.cmd_type == "collect_chaos" and len(self.known_coords_of_pucks) > 0:

            hull = self.calculate_convex_hull()
            # inner_angles = self.calculate_inner_angles(hull)
            landings_coordinates = self.calculate_possible_landing_coords(hull)
            self.sort_landing_coords_wrt_current_pose(landings_coordinates)
            self.collect_closest_safe_puck()

        # elif self.cmd_type == "collect_wall_6" and len(self.known_coords_of_pucks) > 0:
        #     grab_from_wall
        # elif self.cmd_type == "collect_wall_3" and len(self.known_coords_of_pucks) > 0:
        #     grab_from_wall
        # elif self.cmd_type == "collect_accel_blue_and_hold_up" and len(self.known_coords_of_pucks) > 0:
        #     func()
        # elif self.cmd_type == "take_goldenium_and_hold_middle" and len(self.known_coords_of_pucks) > 0:
        #     func()
        # place_atom_on_weights
        #

        else:
            self.stop_collecting()

    # def callback_response(self, data):
    #
    #     # TODO
    #
    #     if data.data == 'finished':
    #         self.arm_ready_flag = True
    #         print self.arm_ready_flag
    #
    #     # if self.current_cmd == "move_line":
    #     #     self.move_line()

    def collect_closest_safe_puck(self):
        """
        1. Approach it
        2. Grab and load it
        3. Remove from list of known
        4. Add to list of collected and count points TODO

        we append id of that puck to list of collected pucks and remove it from list of pucks to be collected.
        :return:
        """
        # MotionPlannerNode.move_arc()
        # call Alexey's function

        # m_cmd = str(cmd)
        #
        # rospy.loginfo("m_cmd:\t" + str(m_cmd))
        # rospy.loginfo("Sending cmd: " + m_cmd)
        # self.pub_cmd.publish(m_cmd)

        # TODO self.arm_ready_flag
        # TODO this part with deleting probably should be a separate function
        if self.puck_status == "collected":
            np.delete(self.known_coords_of_pucks, 1, 0)
            np.delete(self.sorted_landing_coordinates, 1, 0)
            print("pucks left: ", len(self.known_coords_of_pucks))

    def stop_collecting(self):
        self.timer.shutdown()
        rospy.loginfo("Robot has stopped collecting pucks.")
        rospy.sleep(1.0 / 40)
        self.pub_response.publish("finished")

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

        known_ids = self.known_coords_of_pucks[:, 0]
        known_pucks = self.known_coords_of_pucks
        for puck in new_obs_of_pucks:
            puck_id = puck[0]
            if puck_id in known_ids:
                ind = known_ids.index(puck_id)  # correct? FIXME
                x_new, y_new = puck[1], puck[2]
                x_old, y_old = known_pucks[1], known_pucks[2]
                if np.sqrt((x_new - x_old)**2 + (y_new - y_old)**2) > self.coords_threshold:
                    self.known_coords_of_pucks[ind][1:] = puck[1:]
            else:
                # wtf happened? blik from sun? wasn't recognised at first time?
                self.known_coords_of_pucks = np.stack((self.known_coords_of_pucks, puck), axis=0)
                # self.known_coords_of_pucks.append(puck)

    @staticmethod
    def rotate(A, B, C):
        return (B[0]-A[0])*(C[1]-B[1])-(B[1]-A[1])*(C[0]-B[0])

    def calculate_convex_hull(self):
        """
        jarvis march
        :return: convex hull, list of coordinates, TODO counter or clockwise, need to figure out
        """
        A = self.known_coords_of_pucks[:, 1:]  # [(x0, y0), (x1, y1), ...]
        n = len(A)
        P = range(n)
        # start point
        for i in range(1, n):
            if A[P[i]][0] < A[P[0]][0]:
                P[i], P[0] = P[0], P[i]
        H = [P[0]]
        del P[0]
        P.append(H[0])
        while True:
            right = 0
            for i in range(1, len(P)):
                if self.rotate(A[H[-1]], A[P[right]], A[P[i]]) < 0:
                    right = i
            if P[right] == H[0]:
                break
            else:
                H.append(P[right])
                del P[right]
        print("H is")
        print(H)
        return H

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

    def calculate_possible_landing_coords(self, hull):
        """
        Calculates offset a hull,
        cacl outer bissectrisa to all angles
        search intersection point between orbit around centre of puck and this bissectrisa between orig and offset

        There are many safe landing coords, not just on outer bissectrisa

        TODO
        we can start by calculating area of a hull

        :return: list of landing coordinates for all angles [[x_l, y_l], ...]
        """

        # assert np.all(inner_safe_angles[:, 2]) > 0

        # this method can be applied when pucks are close to each other
        # calculate offset
        # calc angle of bissectrisa which it outer wrt original hull
        # calc point that is const vector away from centre of puck

        landings_coordinates = []
        mc = np.mean(hull)
        offset = list(hull)  # a miserable attempt to copy a list
        offset -= mc  # Normalize the polygon by subtracting the center values from every point.
        offset *= self.ScaleFactor_
        offset += mc

        #  need list?
        for orig, ofs in zip(hull, offset):
            gamma = None
            dist = calculate_distance(orig, ofs)
            if abs(dist[0]) < 1e-4:
                if ofs[1] < orig[1]:
                    gamma = np.pi * 1/2
                elif ofs[1] > orig[1]:
                    gamma = np.pi * 3/2
            else:
                gamma = np.arctan2(dist[1], dist[0])  # and here we calculate slope of outer bis
            # need wrap_angle?
            # intersection point between orbit around centre of puck and this otrezok between orig and offset
            # x**2 + y**2 = r_orbit**2
            # FIXME: gamma != -1
            try:
                const = np.sqrt(self.approach_dist**2 / (gamma + 1))
                x_landing = const + orig[0]
                y_landing = gamma * const + orig[1]
                landings_coordinates.append([x_landing, y_landing])
            except ZeroDivisionError:
                print("gamma = -1 which is UNACCEPTABLE !!!1")

        # another method is for situation where pucks are safely away from each other
        # and we can approach them from any angle (should choose closest one)
        # safe_orbit =

        return landings_coordinates

    def sort_landing_coords_wrt_current_pose(self, landings_coordinates):
        """
        To make operation more quick there is no need to choose the sharpest angle,
        instead better choose the closest one from angles that are marked as safe.
        TODO add id to every landing so it corresponds to color of puck
        :param landings_coordinates: [[x_l, y_l], ...]
        :return: format: [([x_l, y_l], d), ...]  ex [([3, 3], 1), ([1, 1], 2), ([2, 2], 3), ([0, 0], 4)]
        """

        distances_from_robot_to_landings = []
        for landing in landings_coordinates:
            dist, _ = calculate_distance(self.robot_coords, landing)  # return deltaX and deltaY coords
            dist_norm = np.linalg.norm(dist)
            distances_from_robot_to_landings.append(dist_norm)
        a = zip(landings_coordinates, distances_from_robot_to_landings)
        a.sort(key=lambda t: t[1])
        self.sorted_landing_coordinates = a[:, :2]

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
    #     return pucks_wrt_robot_sorted


if __name__ == "__main__":
    tactics = TacticsNode()
    rospy.spin()
