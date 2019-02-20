#!/usr/bin/env python
# coding: utf-8

# simple go to goal in different methods: odom movement

import rospy
import numpy as np
import tf2_ros
import time
from tf.transformations import euler_from_quaternion
# from geometry_msgs.msg import Twist
from std_msgs.msg import String
from threading import Lock
from geometry_msgs.msg import Twist
from core_functions import *
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

class PathReader:
    def __init__(self):
        self.path_subscriber = rospy.Subscriber("/navigation/path", Path, self.callback_path)
        self.path = np.array([[]])
        self.point = np.array([0., 0., 0.])

    def callback_path(self, data):
        self.path = np.zeros((len(data.poses), 3))
        for i in range(len(data.poses)):
            self.point[0] = data.poses[i].pose.position.x
            self.point[1] = data.poses[i].pose.position.y
            self.point[2] = 0
            self.path[i] = self.point
        print(self.path)


if __name__ == "__main__":
    rospy.init_node("Path_Reader", anonymous=True)
    reader = PathReader()
    rospy.spin()

