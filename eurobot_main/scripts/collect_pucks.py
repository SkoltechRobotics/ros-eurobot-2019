import rospy
import numpy as np
import tf2_ros
import time
from tf.transformations import euler_from_quaternion
from std_msgs.msg import String
from visualization_msgs.msg import MarkerArray
from std_msgs.msg import String
from threading import Lock
from stm.Manipulator import collect_puck

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

    # assert
    # TODO

    distance_map_frame = coords2[:2] - coords1[:2]
    theta_diff = wrap_angle(coords2[2] - coords1[2])
    return distance_map_frame, theta_diff


# noinspection SpellCheckingInspection,SpellCheckingInspection
class TacticsNode:
    def __init__(self):
        rospy.init_node('TacticsNode')

        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer)
        self.robot_name = "secondary_robot"

        self.critical_angle = np.pi * 2/3
        self.approach_dist = 0.1  # meters, distance from robot to puck where robot will try to grab it # FIXME
        self.robot_coords = None
        self.mutex = Lock()
        self.coords_threshold = 0.01  # meters, this is variance of detecting pucks coords using camera, used in update
        self.sorted_landing_coordinates = None  # FIXME
        self.known_coords_of_chaos_pucks = np.array([])  # (id, x, y)
        self.puck_status = None
        self.ScaleFactor_ = 2  # used in calculating outer bissectrisa for hull's angles
        # status of a puck we're currently working on, read from response.
        # When status becomes "collected" -- remove from self.known_coords_of_pucks
        self.atoms_placed = 0  # to preliminary calculate our score

        # self.arm_ready_flag = False
        self.robot_reached_goal_flag = False
        self.puck_collected_and_arm_ready_flag = False

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
        rospy.Subscriber('response', String, self.callback_response, queue_size=10)

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

        if len(self.known_coords_of_chaos_pucks) == 0:
            self.known_coords_of_chaos_pucks = new_obs_of_pucks

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

        # Calculate pucks configuration once
        hull = self.calculate_convex_hull()
        # inner_angles = self.calculate_inner_angles(hull)
        landings_coordinates = self.calculate_possible_landing_coords(hull)
        self.sort_landing_coords_wrt_current_pose(landings_coordinates)

        self.timer = rospy.Timer(rospy.Duration(1.0 / self.RATE), self.timer_callback, oneshot=False)  # FIXME ask Misha

        self.mutex.release()

    def update_task_status(self, cmd_id, cmd_type):
        rospy.loginfo("Current cmd:\t" + str(cmd_type))
        rospy.loginfo("=====================================")
        rospy.loginfo("")

        self.cmd_id = cmd_id
        self.cmd_type = cmd_type

    # noinspection PyUnusedLocal
    def timer_callback(self, event):

        # TODO add Try-Except in case coords are not yet received

        if self.cmd_type == "collect_chaos" and len(self.known_coords_of_chaos_pucks) > 0:
            self.approach_and_collect_closest_safe_puck()
            self.robot_reached_goal_flag = False

        # elif self.cmd_type == "collect_wall_6" and len(self.known_coords_of_pucks) > 0:
        #     grab_from_wall
        # elif self.cmd_type == "collect_wall_3" and len(self.known_coords_of_pucks) > 0:
        #     grab_from_wall
        # elif self.cmd_type == "collect_accel_blue_and_hold_up" and len(self.known_coords_of_pucks) > 0:
        #     func()
        # elif self.cmd_type == "take_goldenium_and_hold_middle" and len(self.known_coords_of_pucks) > 0:
        #     func()
        # place_atom_on_weights

        else:
            self.stop_collecting()

    def callback_response(self, data):
        if data.data == 'finished':
            self.robot_reached_goal_flag = True
        if data.data == 'puck_collected':  # TODO Alexey to publish in response
            self.puck_collected_and_arm_ready_flag = True
            # self.arm_ready_flag = True

    def approach_and_collect_closest_safe_puck(self):
        """
        1. Approach it
        2. Grab and load it
        3. Remove from list of known
        4. Add to list of collected and count points TODO

        we append id of that puck to list of collected pucks and remove it from list of pucks to be collected.
        :return:
        """
        landing = self.sorted_landing_coordinates[0]
        x, y, theta = landing[0], landing[1], landing[2]
        if self.robot_reached_goal_flag is False:
            self.move_command_publisher.publish(x, y, theta)  # TODO add command id
            # here when robot reaches the goal  it'll publish in response topic "finished" and in this code
            # function callback_response will fire and change self.robot_reached_goal_flag to True
        else:
            collect_puck()  # TODO load_inside=False
            # TODO Alexey to publish in response
            # if result == True than proceed to next puck and remove puck coord from known
            # returns False when busy or what? TODO

        # TODO this part with deleting probably should be a separate function
            if self.puck_collected_and_arm_ready_flag is True:
                np.delete(self.known_coords_of_chaos_pucks, 1, 0)
                np.delete(self.sorted_landing_coordinates, 1, 0)  # FIXME path_planner should decide which puck to collect and to remove from known
                print("result is True and puck collected!")
                print("pucks left: ", len(self.known_coords_of_chaos_pucks))
            self.puck_collected_and_arm_ready_flag = False

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

        known_ids = self.known_coords_of_chaos_pucks[:, 0]
        known_pucks = self.known_coords_of_chaos_pucks
        for puck in new_obs_of_pucks:
            puck_id = puck[0]
            if puck_id in known_ids:
                ind = known_ids.index(puck_id)  # correct? FIXME
                x_new, y_new = puck[1], puck[2]
                x_old, y_old = known_pucks[1], known_pucks[2]
                if np.sqrt((x_new - x_old)**2 + (y_new - y_old)**2) > self.coords_threshold:
                    self.known_coords_of_chaos_pucks[ind][1:] = puck[1:]
            else:
                # wtf happened? blink from sun? wasn't recognised at first time?
                self.known_coords_of_chaos_pucks = np.stack((self.known_coords_of_chaos_pucks, puck), axis=0)
                # self.known_coords_of_pucks.append(puck)

    @staticmethod
    def rotate(a, b, c):
        return (b[0] - a[0]) * (c[1] - b[1]) - (b[1] - a[1]) * (c[0] - b[0])

    def calculate_convex_hull(self):
        """
        jarvis march
        :return: convex hull, list of coordinates, TODO counter or clockwise, need to figure out
        """
        a = self.known_coords_of_chaos_pucks[:, 1:]  # [(x0, y0), (x1, y1), ...]
        n = len(a)
        p = range(n)
        # start point
        for i in range(1, n):
            if a[p[i]][0] < a[p[0]][0]:
                p[i], p[0] = p[0], p[i]
        h = [p[0]]
        del p[0]
        p.append(h[0])
        while True:
            right = 0
            for i in range(1, len(p)):
                if self.rotate(a[h[-1]], a[p[right]], a[p[i]]) < 0:
                    right = i
            if p[right] == h[0]:
                break
            else:
                h.append(p[right])
                del p[right]
        print("h is")
        print(h)
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

    def calculate_possible_landing_coords(self, hull):
        """
        Calculates offset a hull, calculates outer bissectrisa to all angles,
        search intersection point between orbit around centre of puck and this bissectrisa between orig and offset
        There are many safe landing coords, not just on outer bissectrisa, but for now only the intersection

        :return: list of landing coordinates for all angles [[x_l, y_l, gamma], ...]
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
            # need wrap_angle? FIXME
            # FIXME: gamma != -1
            try:
                const = np.sqrt(self.approach_dist**2 / (gamma + 1))
                x_landing = const + orig[0]
                y_landing = gamma * const + orig[1]
                landings_coordinates.append([x_landing, y_landing, gamma])
            except ZeroDivisionError:
                print("gamma = -1 which is UNACCEPTABLE !!!1")

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

        distances_from_robot_to_landings = []
        for landing in landings_coordinates:
            dist, _ = calculate_distance(self.robot_coords, landing)  # return deltaX and deltaY coords
            dist_norm = np.linalg.norm(dist)
            distances_from_robot_to_landings.append(dist_norm)
        a = zip(landings_coordinates, distances_from_robot_to_landings)
        a.sort(key=lambda t: t[1])
        self.sorted_landing_coordinates = [v[0] for v in a]

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
            q = [trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z,
                 trans.transform.rotation.w]
            angle = euler_from_quaternion(q)[2] % (2 * np.pi)
            self.robot_coords = np.array([trans.transform.translation.x, trans.transform.translation.y, angle])
            rospy.loginfo("Robot coords:\t" + str(self.robot_coords))
            return True
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as msg:
            rospy.logwarn(str(msg))
            return False


if __name__ == "__main__":
    tactics = TacticsNode()
    rospy.spin()
