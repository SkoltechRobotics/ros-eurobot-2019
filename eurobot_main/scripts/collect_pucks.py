import rospy
import numpy as np
import tf2_ros
import time
from threading import Lock

from sympy import Point, Polygon
from sympy.geometry import Triangle, Point

from std_msgs.msg import String
from visualization_msgs.msg import MarkerArray

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
    


Input: list of n coordinates of pucks, that belong to one local group


Receiving coords from camera each 2-3 seconds
Each time coords are received we call function callback which compares currently known coords and newly received ones
If difference between newly received and known are within accuracy level (threshold), do nothing, else update known coords

How to do this without timer?

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


class Tactics:

    def __init__(self):

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

        # FIXME
        self.move_command_publisher = rospy.Publisher('move_command', String, queue_size=10)

        self.timer = None

        # coords are published as markers in one list according to 91-92 undistort.py
        rospy.Subscriber("pucks", MarkerArray, self.pucks_coords_callback, queue_size=1)

    def pucks_coords_callback(self, data):
        """
        # TODO
        # implement comparing with threshold,
        # if newly received coord differs from old known one less than threshold level,
        # than ignore it and continue collecting pucks.
        # Else change coord of that puck and recalculate

        :param self:
        :param data:
        :return:
        """
        self.mutex.acquire()

        rospy.loginfo("")
        rospy.loginfo("=====================================")
        rospy.loginfo("NEW CMD:\t" + str(data.data))

        if self.timer is not None:
            self.timer.shutdown()

        print("data")
        print(data)

        # TODO
        # all pucks must have unique id so we can compare their previously known coords and newly received
        # let's assume we have that

        # pucks = [(x_.pose.position.x, x_.pose.position.y) for x_ in data] # Egorka

        # FIXME
        # is it even legal?
        # from observation to observation ID are not guaranteed to be the same!!!!!!

        new_obs_of_pucks = [(x_.id, x_.pose.position.x, x_.pose.position.y) for x_ in data]
        new_obs_of_pucks = np.array(new_obs_of_pucks)

        # in first step we just write received coords to list of known pucks
        # in further steps we compare two lists and decide whether to update it or ignore
        if len(self.known_coords_of_pucks) == 0:
            self.known_coords_of_pucks = new_obs_of_pucks
        else:
            self.compare_to_update_or_ignore(new_obs_of_pucks) # in case robot accidentally moved some of pucks

        # self.local_group_of_pucks_to_collect = self.divide_in_local_groups(self.unsorted_list_of_pucks_coords)
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.RATE), self.timer_callback)

        # rostopic pub TODO

        self.mutex.release()

    def timer_callback(self, event):

        hull = self.calculate_convex_hull(self.known_coords_of_pucks)
        # inner_angles = self.calculate_inner_angles(hull)
        landings_coordinates = self.calculate_possible_landing_coords(hull)
        self.sort_landing_coords_wrt_current_pose(landings_coordinates)

    def collect_closest_safe_puck(self):
        """
        we append id of that puck to list of collected pucks and remove it from list of pucks to be collected.
        :return:
        """
        # TODO
        if self.puck_status == "collected":
            np.delete(self.known_coords_of_pucks, 1, 0)

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

    def calculate_convex_hull(self, list_of_pucks_coords):
        """
        :param list_of_pucks_coords: [(id, x, y), ...]
        :return: convex hull, list of coordinates, TODO counter or clockwise, need to figure out
        """
        # jarvis march

        # A = self.local_group_of_pucks_to_collect
        A = list_of_pucks_coords[:, 1:]  # [(x0, y0), (x1, y1), ...]
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
    #
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
    #     # TODO
    #     # need to maintain unique id or order of pucks when deleting
    #
    #     return inner_safe_angles

    def calculate_possible_landing_coords(self, hull):
        """
        Calculates offset a hull,
        cacl outer bissectrisa to all angles
        search intersection point between orbit around centre of puck and this bissectrisa between orig and offset

        There are many safe landing coords, not just on outer bissectrissa

        TODO
        we can start by calculating area of a hull

        :return: list of landing coordinates for all angles [[x_l, y_l], ...]
        """

        # assert np.all(inner_safe_angles[:, 2]) > 0

        # this method can be applied when pucks are close to each other
        # calculate offset
        # calc angle of bissectrisa wich it outer wrt original hull
        # calc point that is const vector away from centre of puck

        landings_coordinates = []
        mc = np.mean(hull)
        offset = hull.copy()
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

    def sort_pucks_coords_wrt_current_pose(self):
        """
        To make operation more quick there is no need to choose the sharpest angle,
        instead better choose the closest one from angles that are marked as safe.
        :return: format [[(id0, x0, y0), d0], ...]
        """

        distances_from_robot_to_pucks = []
        for puck in self.known_coords_of_pucks:
            dist, _ = calculate_distance(self.robot_coords, puck)  # return deltaX and deltaY coords
            dist_norm = np.linalg.norm(dist)
            distances_from_robot_to_pucks.append(dist_norm)
        distances_from_robot_to_pucks = np.array(distances_from_robot_to_pucks)
        pucks_wrt_robot_sorted = zip(self.known_coords_of_pucks, distances_from_robot_to_pucks)
        pucks_wrt_robot_sorted.sort(key=lambda t: t[1])
        return pucks_wrt_robot_sorted


if __name__ == "__main__":
    tactics = Tactics()
    rospy.spin()
