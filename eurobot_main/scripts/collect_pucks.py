import numpy as np

# TODO
from core_functions import calculate_distance


"""
This algorithm knows nothing about obstacles and etc, it just generates sorted list of options.
And path-planner will choose from these options according to obstacles around

TODO:
    - add module to get self-coords
    - publish in response that current puck was collected and robot is ready for next one
    - subscribe to camera_pucks topic to get coords of pucks
    - when atom is grabbed and collected, publish DONE in response
    - when we receive DONE, re-calculate convex hull and generate new sorted list of closest pucks wrt current pose

Input: list of n coordinates of pucks, that belong to one local group

Inside:
    - calculates the convex hull of these points
    - calculates inner angles and sorts them
    - calculates bissectrisa of the angle and adds offset
    -
Output: ready-steady coordinate and go coordinate

Publishes
"""

class Tactics:

    def __init__(self):

        critical_angle = np.pi * 2/3
        approach_dist = 0.06  # meters, distance from robot to puck where robot will try to grab it
        self.coords = None
        self.unsorted_list_of_pucks_coords = []
        self.local_group_of_pucks_to_collect = []

        rospy.Subscriber("pucks_coords", String, self.pucks_coords_callback, queue_size=1)

    def pucks_coords_callback(self, data):
        """

        :param self:
        :param data:
        :return:
        """
        self.mutex.acquire()

        self.unsorted_list_of_pucks_coords = []
        self.local_group_of_pucks_to_collect = []

        self.mutex.release()

    @staticmethod
    def rotate(A, B, C):
        return (B[0]-A[0])*(C[1]-B[1])-(B[1]-A[1])*(C[0]-B[0])

    def calculate_convex_hull(self):
        # jarvis march
        A = self.local_group_of_pucks_to_collect
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
        return H

    def calculate_inner_angles(self, hull, threshold):
        """
        Calculates inner angles of a convex hull
        Sorts them
        Removes angles that are below threshold safe level

        :return: sorted list of angles starting with sharpest one
        """
        inner_angles = []

        return inner_angles

    def calculate_possible_landing_coords(self, inner_angles, approach_dist):
        """
        Calculates basis vector of inner bissectrisa of an angle,
        adds pi to make it a basis vector of outer bissectrisa
        and multiplyes by approach_dist to get landing coordinate, where robot should be standing to grab this atom

        :return: sorted list of landing coordinates for all angles
        """
        landing_coordinates = []

        return landing_coordinates

    def sort_landing_coords_wrt_current_pose(self, landing_coordinates):
        """
        To make operation more quick there is no need to choose the sharpest angle,
        instead better choose the closest one from angles that are marked as safe.
        :param landing_coordinates:
        :return:
        """
        calculate_distance
        self.coords
        sorted_landing_coordinates = []

        return sorted_landing_coordinates


if __name__ == "__main__":

    while len(self.local_group_of_pucks_to_collect) > 0:
        hull = calculate_convex_hull(local_group_of_pucks_to_collect)
        inner_angles = calculate_inner_angles(hull)
        landing_coordinates = calculate_possible_landing_coords(inner_angles, approach_dist)
        sorted_landing_coordinates = sort_landing_coords_wrt_current_pose(landing_coordinates)
