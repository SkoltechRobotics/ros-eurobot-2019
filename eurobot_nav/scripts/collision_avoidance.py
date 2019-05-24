#!/usr/bin/env python
# coding: utf-8

import rospy
import numpy as np
from core_functions import cvt_global2local, cvt_local2global, wrap_angle
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import PoseStamped, Point

LIDAR_DELTA_ANGLE = (np.pi / 180) / 4
LIDAR_START_ANGLE = -(np.pi / 2 + np.pi / 4)


class CollisionAvoidanceMainRobot(object):
    def __init__(self):
        self.robot_name = rospy.get_param("robot_name")
#       collision params
        self.p = 1
        self.sensor_coords = np.array(rospy.get_param("collision/sensor_position"))
        self.min_dist_to_obstacle_lidar = rospy.get_param("collision/min_dist_to_obstacle_lidar")
        self.min_dist_to_obstacle_sensor = rospy.get_param("collision/min_dist_to_obstacle_sensor")
        self.angle_sensor = rospy.get_param("collision/angle_sensor")
        self.cos = np.cos(self.angle_sensor)
#       map params
        self.length_x = 300
        self.length_y = 200
        self.resolution = 0.01
        self.map = np.zeros((self.length_y, self.length_x))
        self.map[144:200, 35:265] = 100
        self.map[0:12, 45:255] = 100
        self.map[192:200, 3:43] = 100
        self.map[192:200, 257:298] = 100
        self.map[129:200, 143:157] = 100
        self.map[25:125, 250:300] = 100
        self.map[25:125, 0:50] = 100
#       set params to default
        self.obstacle_points_lidar = np.zeros((0, 2))
        self.obstacle_points_sensor = np.zeros((0, 2))
        self.obstacle_points = np.zeros((0, 2))
        self.collision_area = None
        self.default_obstacle_point = np.array([100., 100.])
        self.num_lidar_collision_points = None
        self.num_sensor_collision_points = None
#       init publishers
        self.point_publisher = rospy.Publisher("obstacle", MarkerArray, queue_size=10)
        self.collision_area_publisher = rospy.Publisher("collision_area", Marker, queue_size=10)
#       init subscribers
        rospy.Subscriber("/%s/scan" % self.robot_name, LaserScan, self.scan_callback, queue_size=1)
        rospy.Subscriber("/%s/stm/proximity_status" % self.robot_name, String, self.proximity_callback, queue_size=10)

    @staticmethod
    def filter_scan(scan):
        ranges = np.array(scan.ranges)
        index0 = ranges < 2
        index = index0
        return np.where(index, ranges, 0)

    def get_landmarks(self, scan):
        """Returns filtrated lidar data"""
        ranges = np.array(scan.ranges)
        ind = self.filter_scan(scan)
        final_ind = np.where((np.arange(ranges.shape[0]) * ind) > 0)[0]
        angles = (LIDAR_DELTA_ANGLE * final_ind + LIDAR_START_ANGLE) % (2 * np.pi)
        distances = ranges[final_ind]
        return angles, distances

    @staticmethod
    def get_landmarks_inside_table(landmarks):
        landmarks = landmarks[np.where(landmarks[:, 0] > 0.15)]
        landmarks = landmarks[np.where(landmarks[:, 0] < 2.85)]
        landmarks = landmarks[np.where(landmarks[:, 1] > 0.15)]
        landmarks = landmarks[np.where(landmarks[:, 1] < 1.85)]
        return landmarks

    def get_points_outside_map(self, points):
        ind = np.array(points / self.resolution).astype(int)
        final_points = np.array([[0., 0.]])
        for i in range(points.shape[0]):
            if self.map[ind[i, 1], ind[i, 0]] == 0:
                final_points = np.append(final_points, np.array([points[i]]), axis=0)
        final_points = np.delete(final_points, 0, 0)
        return final_points

    def get_points_inside_collision_area(self, points, coords, goal):
        goal = cvt_global2local(goal, coords)
        angle = np.arctan2(goal[1], goal[0])
        index = np.ones(points.shape[0])
        area = cvt_global2local(self.collision_area, np.array([coords[0], coords[1], wrap_angle(coords[2] + angle)]))
        obstacles = cvt_global2local(points, np.array([coords[0], coords[1], wrap_angle(coords[2] + angle)]))
        index *= obstacles[:, 0] > area[0, 0]
        index *= obstacles[:, 0] < area[2, 0]
        index *= obstacles[:, 1] > area[0, 1]
        index *= obstacles[:, 1] < area[2, 1]
        return np.where(index > 0)

    def proximity_callback(self, data):
        distances = (np.array((data.data.split())).astype(float))/100
        rospy.loginfo(distances)
        distances[np.where(distances == 0.5)] = 1
        points = np.zeros((0, 2))
        for i in range(self.sensor_coords.shape[0]):
            points_in_sensor_frame = np.array([cvt_local2global(np.array([distances[i], 0]), self.sensor_coords[i, :])])
            points = np.append(points, points_in_sensor_frame, axis=0)
        self.obstacle_points_sensor = points

    def scan_callback(self, scan):
        scan = scan
        angles, distances = self.get_landmarks(scan)
        x = distances * np.cos(angles)
        y = distances * np.sin(angles)
        self.obstacle_points_lidar = np.zeros((0, 2))
        landmarks = (np.array([x, y])).T
        if landmarks.size > 0:
            self.obstacle_points_lidar = landmarks

    def get_collision_area(self, coords, goal):
        rospy.loginfo("GOAL %s", goal)
        rospy.loginfo("COORDS %s", coords)
        goal_in_robot_frame = cvt_global2local(goal, coords)
        rospy.loginfo("GOAL IN ROBOT FRAME %s", goal_in_robot_frame)
        dist_to_goal = np.linalg.norm(goal[:2] - coords[:2])
        rospy.loginfo("DIST TO GOAL POINT %s", dist_to_goal)
        points = np.array([[-0.15, -0.15], [dist_to_goal+0.15, -0.15], [dist_to_goal+0.15, 0.15], [-0.15, 0.15], [-0.15, -0.15]])
        self.collision_area = cvt_local2global(points, np.array(
            [coords[0], coords[1], coords[2] + wrap_angle(np.arctan2(goal_in_robot_frame[1], goal_in_robot_frame[0]))]))
        self.num_lidar_collision_points = len(self.obstacle_points_lidar)
        self.num_sensor_collision_points = len(self.obstacle_points_sensor)
        self.obstacle_points = np.concatenate((self.obstacle_points_lidar.copy(), self.obstacle_points_sensor.copy()), axis=0)
        self.obstacle_points = self.filter_points(self.obstacle_points.copy(), coords.copy(), goal.copy())
        self.set_collision_point(self.obstacle_points)

    def filter_points(self, points, coords, goal):
        filtered_points = cvt_local2global(points.copy(), coords.copy())
        filtered_points = self.get_landmarks_inside_table(filtered_points.copy())
        filtered_points = self.get_points_outside_map(filtered_points.copy())
        filtered_points = filtered_points[self.get_points_inside_collision_area(filtered_points.copy(), coords.copy(), goal.copy())]
        return filtered_points

    def get_collision_status(self, coords, goal):
        self.p = 1
        p_lidar = 1
        p_sensor = 1
        self.get_collision_area(coords, goal)
        rospy.loginfo(self.obstacle_points)
        if self.obstacle_points.shape[0] > 0:
            distances = np.linalg.norm(coords[:2] - self.obstacle_points, axis=1)
            min_ind = np.argmin(distances)
            min_dist_to_obstacle = distances[min_ind]
            obstacle_point = self.obstacle_points[min_ind]
            rospy.loginfo("Lidar dist %s", min_dist_to_obstacle)
            if min_ind < self.num_lidar_collision_points:
                if min_dist_to_obstacle < self.min_dist_to_obstacle_lidar:
                    rospy.loginfo("COLLISION")
                    return True, self.p, obstacle_point
                else:
                    if min_dist_to_obstacle > 0.7:
                        p_lidar = 1
                    else:
                        p_lidar = min_dist_to_obstacle/0.7
            else:
                if min_dist_to_obstacle < self.min_dist_to_obstacle_sensor:
                    rospy.loginfo("COLLISION")
                    return True, self.p, obstacle_point
                else:
                    if min_dist_to_obstacle > 0.4:
                        p_sensor = 1
                    else:
                        p_sensor = min_dist_to_obstacle/0.4
            return False, min(p_lidar, p_sensor), obstacle_point
        else:
            return   False, self.p, self.default_obstacle_point

    def set_collision_point(self, positions):
        marker = []
        for i, position in enumerate(positions):
            point = Marker()
            point.header.stamp = rospy.Time.now()
            point.header.frame_id = "map"
            point.id = i
            point.type = 1
            point.ns = "obstacle"
            #
            #rospy.loginfo(position)
            point.pose.position.x = position[0]
            point.pose.position.y = position[1]
            point.pose.position.z = 0
            point.pose.orientation.w = 1
            point.scale.x = 0.1
            point.scale.y = 0.1
            point.scale.z = 0.1
            point.color.a = 1
            point.color.r = 1
            point.lifetime = rospy.Duration(0.7)
            marker.append(point)
        self.point_publisher.publish(marker)

    def set_collision_area(self, points):
        marker = Marker()
        marker.type = marker.LINE_STRIP
        marker.action = marker.ADD
        marker.scale.x = 0.05
        marker.scale.y = 0.05
        marker.scale.z = 0.05
        marker.color.a = 1
        marker.color.g = 1
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.1
        for i, position in enumerate(points):
            rospy.loginfo(position)
            point = Point()
            point.x = position[0]
            point.y = position[1]
            point.z = 0.2
            marker.points.append(point)
        self.collision_area_publisher.publish(marker)


class CollisionAvoidanceSecondaryRobot(object):
    def __init__(self):
        self.robot_name = rospy.get_param("robot_name")
#       collision params
        self.p = 1
        self.sensor_coords = np.array(rospy.get_param("collision/sensor_position"))
        self.min_dist_to_obstacle_lidar = rospy.get_param("collision/min_dist_to_obstacle_lidar")
        self.min_dist_to_obstacle_sensor = rospy.get_param("collision/min_dist_to_obstacle_sensor")
        self.angle_sensor = rospy.get_param("collision/angle_sensor")
        self.cos = np.cos(self.angle_sensor)
#       map params
        self.length_x = 300
        self.length_y = 200
        self.resolution = 0.01
        self.map = np.zeros((self.length_y, self.length_x))
        self.map[144:200, 35:265] = 100
        self.map[0:12, 45:255] = 100
        self.map[192:200, 3:43] = 100
        self.map[192:200, 257:298] = 100
        self.map[129:200, 143:157] = 100
        self.map[25:125, 250:300] = 100
        self.map[25:125, 0:50] = 100
#       set params to default
        self.obstacle_points_lidar = np.zeros((0, 2))
        self.obstacle_points_sensor = np.zeros((0, 2))
        self.obstacle_points = np.zeros((0, 2))
        self.collision_area = None
        self.num_lidar_collision_points = None
        self.num_sensor_collision_points = None
        self.default_obstacle_point = []
#       init subscribers
        rospy.Subscriber("/%s/scan" % self.robot_name, LaserScan, self.scan_callback, queue_size=1)
        rospy.Subscriber("/%s/stm/proximity_status" % self.robot_name, String, self.proximity_callback, queue_size=10)
        #rospy.Subscriber("obstacle_point", String, self.obstacle_callback, queue_size=1)
#       init publishers
        self.point_publisher = rospy.Publisher("obstacle", MarkerArray, queue_size=10)
        self.collision_area_publisher = rospy.Publisher("collision_area", Marker, queue_size=10)
        #self.point_publisher = rospy.Publisher("obstacle_point_", MarkerArray, queue_size=1)

    def obstacle_callback(self, point):
        self.obstacle_points = np.array([point.data.split()]).astype(float)
        rospy.loginfo("!!!!!!!!!!!!!")
        rospy.loginfo(self.obstacle_points)
        # self.set_collision_point(self.obstacle_points)

    @staticmethod
    def filter_scan(scan):
        ranges = np.array(scan.ranges)
        index0 = ranges < 2
        index1 = ranges > 0.2
        index = index0
        return np.where(index, ranges, 0)

    def get_landmarks(self, scan):
        """Returns filtrated lidar data"""
        ranges = np.array(scan.ranges)
        ind = self.filter_scan(scan)
        final_ind = np.where((np.arange(ranges.shape[0]) * ind) > 0)[0]
        angles = (LIDAR_DELTA_ANGLE * final_ind + LIDAR_START_ANGLE) % (2 * np.pi)
        distances = ranges[final_ind]
        distances -= 0.05
        return angles, distances

    @staticmethod
    def get_landmarks_inside_table(landmarks):
        landmarks = landmarks[np.where(landmarks[:, 0] > 0.15)]
        landmarks = landmarks[np.where(landmarks[:, 0] < 2.85)]
        landmarks = landmarks[np.where(landmarks[:, 1] > 0.15)]
        landmarks = landmarks[np.where(landmarks[:, 1] < 1.85)]
        return landmarks

    def get_points_outside_map(self, points):
        ind = np.array(points / self.resolution).astype(int)
        final_points = np.array([[0., 0.]])
        for i in range(points.shape[0]):
            if self.map[ind[i, 1], ind[i, 0]] == 0:
                final_points = np.append(final_points, np.array([points[i]]), axis=0)
        final_points = np.delete(final_points, 0, 0)
        return final_points

    def get_points_inside_collision_area(self, points, coords, goal):
        goal = cvt_global2local(goal, coords)
        angle = np.arctan2(goal[1], goal[0])
        index = np.ones(points.shape[0])
        area = cvt_global2local(self.collision_area, np.array([coords[0], coords[1], wrap_angle(coords[2] + angle)]))
        obstacles = cvt_global2local(points, np.array([coords[0], coords[1], wrap_angle(coords[2] + angle)]))
        index *= obstacles[:, 0] > area[0, 0]
        index *= obstacles[:, 0] < area[2, 0]
        index *= obstacles[:, 1] > area[0, 1]
        index *= obstacles[:, 1] < area[2, 1]
        return np.where(index > 0)

    def proximity_callback(self, data):
        distances = (np.array((data.data.split())).astype(float))/100
        #distances[np.where(distances == 0.5)] = 1
        points = np.zeros((0, 2))
        for i in range(self.sensor_coords.shape[0]):
            points_in_sensor_frame = np.array([cvt_local2global(np.array([distances[i], 0]), self.sensor_coords[i, :])])
            points = np.append(points, points_in_sensor_frame, axis=0)
        self.obstacle_points_sensor = points

    @staticmethod
    def alpha_filter(cloud, min_sin_angle):
        x, y = cloud.T
        x0, y0 = 0, 0
        x1, y1 = np.roll(x, 1), np.roll(y, 1)
        cos_angle = ((x - x0) * (x - x1) + (y - y0) * (y - y1)) / (np.sqrt((x - x0) * (x - x0) + (y - y0) * (y - y0))
                                                                   * np.sqrt((x - x1) * (x - x1) + (y - y1) * (y - y1)))
        sin_angle = np.sqrt(1 - cos_angle * cos_angle)
        index = sin_angle >= min_sin_angle
        return index

    def scan_callback(self, scan):
        scan = scan
        angles, distances = self.get_landmarks(scan)
        x = distances * np.cos(angles)
        y = distances * np.sin(angles)
        self.obstacle_points_lidar = np.zeros((0, 2))
        landmarks = (np.array([x, y])).T
        #rospy.loginfo(landmarks)
        if landmarks.size > 0:
            self.obstacle_points_lidar = landmarks

    def get_collision_area(self, coords, goal):
        rospy.loginfo("GOAL %s", goal)
        rospy.loginfo("COORDS %s", coords)
        goal_in_robot_frame = cvt_global2local(goal, coords)
        rospy.loginfo("GOAL IN ROBOT FRAME %s", goal_in_robot_frame)
        dist_to_goal = np.linalg.norm(goal[:2] - coords[:2])
        rospy.loginfo("DIST TO GOAL POINT %s", dist_to_goal)
        points = np.array([[-0.15, -0.15], [dist_to_goal+0.15, -0.15], [dist_to_goal+0.15, 0.15], [-0.15, 0.15], [-0.15, -0.15]])
        self.collision_area = cvt_local2global(points, np.array(
            [coords[0], coords[1], coords[2] + wrap_angle(np.arctan2(goal_in_robot_frame[1], goal_in_robot_frame[0]))]))
        self.num_lidar_collision_points = len(self.obstacle_points_lidar)
        self.num_sensor_collision_points = len(self.obstacle_points_sensor)
        self.obstacle_points = np.concatenate((self.obstacle_points_lidar.copy(), self.obstacle_points_sensor.copy()), axis=0)
        self.obstacle_points = self.filter_points(self.obstacle_points.copy(), coords.copy(), goal.copy())
        self.set_collision_point(self.obstacle_points)


    def filter_points(self, points, coords, goal):
        rospy.loginfo("POINTS")
        filtered_points = cvt_local2global(points.copy(), coords.copy())
        filtered_points = self.get_landmarks_inside_table(filtered_points.copy())
        filtered_points = self.get_points_outside_map(filtered_points.copy())
        filtered_points = filtered_points[self.get_points_inside_collision_area(filtered_points.copy(), coords.copy(), goal.copy())]
        rospy.loginfo(filtered_points)
        return filtered_points


    def get_collision_status(self, coords, goal):
        self.p = 1
        p_lidar = 1
        p_sensor = 1
        self.get_collision_area(coords, goal)
        rospy.loginfo(self.obstacle_points)
        if self.obstacle_points.shape[0] > 0:
            distances = np.linalg.norm(coords[:2] - self.obstacle_points, axis=1)
            min_ind = np.argmin(distances)
            min_dist_to_obstacle = distances[min_ind]
            obstacle_point = self.obstacle_points[min_ind]
            rospy.loginfo("Lidar dist %s", min_dist_to_obstacle)
            if min_ind < self.num_lidar_collision_points:
                if min_dist_to_obstacle < self.min_dist_to_obstacle_lidar:
                    rospy.loginfo("COLLISION")
                    return True, self.p, obstacle_point
                else:
                    if min_dist_to_obstacle > 0.7:
                        p_lidar = 1
                    else:
                        p_lidar = min_dist_to_obstacle/0.7
            else:
                if min_dist_to_obstacle < self.min_dist_to_obstacle_sensor:
                    rospy.loginfo("COLLISION")
                    return True, self.p, obstacle_point
                else:
                    if min_dist_to_obstacle > 0.4:
                        p_sensor = 1
                    else:
                        p_sensor = min_dist_to_obstacle/0.4
            return False, min(p_lidar, p_sensor), obstacle_point
        else:
            return   False, self.p, self.default_obstacle_point

    def set_collision_point(self, positions):
        marker = []
        for i, position in enumerate(positions):
            point = Marker()
            point.header.stamp = rospy.Time.now()
            point.header.frame_id = "map"
            point.id = i
            point.type = 3
            point.ns = "obstacle"
            #
            #rospy.loginfo(position)
            point.pose.position.x = position[0]
            point.pose.position.y = position[1]
            point.pose.position.z = 0
            point.pose.orientation.w = 1
            point.scale.x = 0.1
            point.scale.y = 0.1
            point.scale.z = 0.1
            point.color.a = 1
            point.color.r = 1
            point.lifetime = rospy.Duration(0.7)
            marker.append(point)
        self.point_publisher.publish(marker)

    def set_collision_area(self, points):
        marker = Marker()
        marker.type = marker.LINE_STRIP
        marker.action = marker.ADD
        marker.scale.x = 0.05
        marker.scale.y = 0.05
        marker.scale.z = 0.05
        marker.color.a = 1
        marker.color.g = 1
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.1
        marker.lifetime = rospy.Duration(1.)
        for i, position in enumerate(points):
            rospy.loginfo(position)
            point = Point()
            point.x = position[0]
            point.y = position[1]
            point.z = 0.2
            marker.points.append(point)
        self.collision_area_publisher.publish(marker)

