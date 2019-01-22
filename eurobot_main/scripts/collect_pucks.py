import rospy
import numpy as np
import tf2_ros
import time
from threading import Lock

from std_msgs.msg import String
from visualization_msgs.msg import MarkerArray

# TODO
from core_functions import calculate_distance

"""
This algorithm knows nothing about obstacles and etc, it just generates sorted list of options.
And path-planner will choose from these options according to obstacles around

TODO:
    - add module to get self-coords
    - publish in response that current puck was collected and robot is ready for next one
    - when atom is grabbed and collected, publish DONE in response
    - when we receive DONE, re-calculate convex hull and generate new sorted list of closest pucks wrt current pose
    - when we get a response that puck is collected, we remove it's coord from list of known coords

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

class Tactics:

    def __init__(self):

        self.critical_angle = np.pi * 2/3
        self.approach_dist = 0.06  # meters, distance from robot to puck where robot will try to grab it
        self.coords = None
        self.unsorted_list_of_pucks_coords = []
        # self.local_group_of_pucks_to_collect = []
        self.mutex = Lock()
        self.coords_threshold = 0.01 # meters, this is variance of detecting pucks coords using camera
        self.sorted_landing_coordinates = None

        self.known_coords_of_pucks = []
        # TODO
        # dictionary?

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
        pucks = [(x_.id, x_.pose.position.x, x_.pose.position.y) for x_ in data]

        self.unsorted_list_of_pucks_coords = np.array(pucks)

        # in first step we just write received coords to list of known pucks
        # in further steps we compare two lists and decide whether to update it or ignore
        if len(self.known_coords_of_pucks) == 0:
            self.known_coords_of_pucks = self.unsorted_list_of_pucks_coords
        else:
            self.compare_to_update_or_ignore()


        # self.local_group_of_pucks_to_collect = self.divide_in_local_groups(self.unsorted_list_of_pucks_coords)
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.RATE), self.timer_callback)

        self.mutex.release()

    def timer_callback(self, event):
        self.calculate_environment()

        # TODO
        # add some action

    def calculate_environment(self):
        hull = self.calculate_convex_hull(self.unsorted_list_of_pucks_coords)
        inner_angles = self.calculate_inner_angles(hull)
        landing_coordinates = self.calculate_possible_landing_coords(inner_angles, self.approach_dist)
        self.sorted_landing_coordinates = self.sort_landing_coords_wrt_current_pose(landing_coordinates)


    def compare_to_update_or_ignore(self):
        """
        format: (id, x, y)
        
        Old
        [(1, x1, y1), 
         (2, x2, y2)
         (3, x3, y3)]
         
         New
         [(2, x2', y2'),
          (3, x3', y3')]
          
        In a new list first puck may be absent for at least three reasons:
        - it was collected by our robot
        - it is not visible (but it's still there)
        - it was collected by enemy robot (and it's not there anymore)
        
        In first case we append id of that puck to list of collected pucks and remove it from list of pucks to be collected.
        
          
        """
        for puck in new:
            puck_id = puck[0]
            known_ids = known[:, 0]
            if puck_id in known_ids:
                # ind = index
                # x_new, y_new
                # x_old, y_old
                if np.sqrt((x_new - x_old)**2 + (y_new - y_old)**2) > self.threshold:
                    known[ind][1:] = puck[1:]
            else:
                # wtf happened? blik from sun? wasn't recognised at first time?
                known.append(puck)
                
                

    # def divide_in_local_groups(self):
    #     local_group_of_pucks_to_collect = []
    #     return local_group_of_pucks_to_collect

    @staticmethod
    def rotate(A, B, C):
        return (B[0]-A[0])*(C[1]-B[1])-(B[1]-A[1])*(C[0]-B[0])

    def calculate_convex_hull(self, unsorted_list_of_pucks_coords):
        # jarvis march

        # A = self.local_group_of_pucks_to_collect
        A = unsorted_list_of_pucks_coords
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
    tactics = Tactics()
    rospy.spin()
