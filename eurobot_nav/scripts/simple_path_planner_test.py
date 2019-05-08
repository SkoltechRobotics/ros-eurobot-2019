#!/usr/bin/env python
# coding: utf-8

import rospy
import numpy as np
from nav_msgs.msg import Path
from std_msgs.msg import String
from path_planner import PathPlanning
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker


class SimplePathPlannerTest(object):
    def __init__(self):
        rospy.init_node("motion_planner", anonymous=True)
        self.path_planner = PathPlanning()
        self.robot_radius = 0.35
        self.world_border_bottom = np.array([[0.1, 0.], [0.1, self.robot_radius], [2.9, self.robot_radius], [2.9, 0.]])
        self.world_border_left = np.array([[0., 0.], [0., 2.], [self.robot_radius, 2.], [self.robot_radius, 0.]])
        self.world_border_top = np.array([[0.1, 1.9], [3., 1.9], [3., 2. - self.robot_radius], [0.1, 2. - self.robot_radius]])
        self.world_border_right = np.array([[3., 0.], [3., 2.], [3. - self.robot_radius, 2.], [3. - self.robot_radius, 0.]])
        self.scales = np.array([[0.45 - self.robot_radius, 2.8], [0.45 - self.robot_radius, 0.1543 - self.robot_radius],
                                [0.255 + self.robot_radius, 0.1543 - self.robot_radius],
                                [0.255 + self.robot_radius, 2.8], [0.45 - self.robot_radius, 2.8]])
        self.obstacle_polygon = []
        self.path_publisher = rospy.Publisher('path', Path, queue_size=10)
        self.collision_area = rospy.Publisher('polygons', Marker, queue_size=1)
        self.set_collision_area(self.world_border_bottom)
        self.set_collision_area(self.world_border_left)
        self.set_collision_area(self.world_border_right)
        self.set_collision_area(self.world_border_top)
        rospy.Subscriber("obstacle", String, self.obstacle_callback, queue_size=1)
        rospy.Subscriber("goal_point", String, self.goal_point_callback, queue_size=1)

    def get_polygon_from_point(self, point):
        return np.array([[point[0] - self.robot_radius, point[1] - self.robot_radius],
                         [point[0] - self.robot_radius, point[1] + self.robot_radius],
                         [point[0] + self.robot_radius, point[1] + self.robot_radius],
                         [point[0] + self.robot_radius, point[1] - self.robot_radius],
                         [point[0] - self.robot_radius, point[1] - self.robot_radius]])

    def set_collision_area(self, points):
        marker = Marker()
        marker.type = marker.LINE_STRIP
        marker.action = marker.ADD
        marker.scale.x = 0.02
        marker.scale.y = 0.02
        marker.scale.z = 0.02
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
        marker.pose.position.z = 0.0
        for i, position in enumerate(points):
            rospy.loginfo(position)
            point = Point()
            point.x = position[0]
            point.y = position[1]
            point.z = 0.2
            marker.points.append(point)
        self.collision_area.publish(marker)

    def obstacle_callback(self, obstacle):
        obstacle_point = np.array(obstacle.data.split()).astype(float)
        self.obstacle_polygon = self.get_polygon_from_point(obstacle_point)
        self.set_collision_area(self.obstacle_polygon)
        self.obstacle_polygon = [self.obstacle_polygon]
        print (self.obstacle_polygon)

    def pub_path(self, path):
        path_ = Path()
        path_.header.stamp = rospy.Time.now()
        path_.header.frame_id = 'map'
        for i in path:
            pose = PoseStamped()
            pose.header.stamp = rospy.Time.now()
            pose.header.frame_id = 'map'
            pose.pose.position.x = i[0]
            pose.pose.position.y = i[1]
            path_.poses.append(pose)
        self.path_publisher.publish(path_)

    def goal_point_callback(self, goal):
        goal = np.array(goal.data.split()).astype(float)
        print (self.obstacle_polygon)
        path = self.path_planner.create_path(np.array([0.5, 0.5]), goal, self.obstacle_polygon)
        rospy.loginfo("path %s", path)
        self.pub_path(path)

if __name__ == "__main__":
    test_path_planner = SimplePathPlannerTest()
    rospy.spin()
