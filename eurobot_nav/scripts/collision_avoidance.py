#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64MultiArray, String
from visualization_msgs.msg import MarkerArray, Marker
from core_functions import *


class CollisionAvoidance(object):

    def __init__(self):
        rospy.init_node("collision_avoidance", anonymous=True)
        rospy.Subscriber("/sensors", Float64MultiArray, self.distances_callback, queue_size=10)
        rospy.Subscriber("/point", String, self.point_callback, queue_size=10)
        self.point_publisher = rospy.Publisher("/obstacle_point", MarkerArray, queue_size=1)
        self.min_dist2obstacle = 0.1
        self.sensors_num = 3
        self.min_dist_before_terminating = 0.05
        self.points = np.ones(self.sensors_num) * 100
        self.distances = None
        # self.sensors_coords = np.array([0., 0., 0.], [2, 2, 2], [3, 3, 3]])
        self.obstacle_points = np.zeros((3, 2))
        self.distance_sensors_centre = 0.2

    def set_collision_point(self, positions):
        marker = []
        for i, position in enumerate(positions):
            point = Marker()
            point.header.stamp = rospy.Time.now()
            point.header.frame_id = "map"
            point.id = i
            point.type = 3
            point.ns = "obstacle"
            point.pose.position.x = position[0]
            point.pose.position.y = position[1]
            point.pose.position.z = 0
            point.pose.orientation.w = 1
            point.scale.x = 0.2
            point.scale.y = 0.2
            point.scale.z = 0.35
            point.color.a = 1
            point.color.r = 1
            #point.lifetime = rospy.Duration(0.7)
            marker.append(point)
        self.point_publisher.publish(marker)

    def distances_callback(self, distances):
        for i in range(self.sensors_num):
            if distances[i].data > 0:
                self.distances[i] = float(distances[i].data)
            else:
                self.distances[i] = 100

    def point_callback(self, data):
        data_split = data.data.split()
        x = data_split[0]
        y = data_split[1]
        self.set_collision_point(np.array([[x, y]]).astype(float))

    def cvt_distances2points(self):
        return self.sensors_coords + (self.distances + self.distance_sensors_centre)/self.distances

if __name__ == "__main__":
    collision_avoidance = CollisionAvoidance()
    rospy.spin()




