#!/usr/bin/env python
import numpy as np
import rospy
from nav_msgs.msg import OccupancyGrid


class MapPublisher:
    # noinspection PyTypeChecker
    def __init__(self):
        rospy.loginfo("INIT")
        self.map_publisher = rospy.Publisher("map", OccupancyGrid, queue_size=10)
        self.length_x = 300
        self.length_y = 200
        self.resolution = 0.01
        self.map = np.zeros((self.length_y, self.length_x))
        self.map[154:200, 45:255] = 100
        self.map[0:7, 50:250] = 100
        self.map[197:200, 8:38] = 100
        self.map[197:200, 262:293] = 100
        self.map[134:200, 148:152] = 100
        x, y = np.meshgrid(range(self.length_x), range(self.length_y))
        self.map[((y - 105)**2 + (x - 100)**2) <= 225] = 100
        x, y = np.meshgrid(range(self.length_x), range(self.length_y))
        self.map[((y - 105)**2 + (x - 200)**2) <= 225] = 100
        rospy.Timer(rospy.Duration(1. / 2), self. publish_map)

    def publish_map(self, event):
        grid = OccupancyGrid()
        rospy.loginfo(self.map[39][39])
        grid.header.frame_id = "map"
        grid.header.stamp = rospy.Time.now()
        grid.info.width = self.length_x
        grid.info.height = self.length_y
        grid.info.resolution = self.resolution
        grid.data = self.map.flatten()
        self.map_publisher.publish(grid)


if __name__ == '__main__':
    try:
        rospy.loginfo("START")
        rospy.init_node('map_publisher', anonymous=True)
        map_publisher = MapPublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

