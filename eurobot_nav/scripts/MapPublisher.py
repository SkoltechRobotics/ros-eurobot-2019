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
        self.map[144:200, 35:265] = 100
        self.map[0:12, 45:255] = 100
        self.map[192:200, 3:43] = 100
        self.map[192:200, 257:298] = 100
        self.map[129:200, 143:157] = 100
        self.map[25:125, 250:300] = 100
        self.map[25:125, 0:50] = 100
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

