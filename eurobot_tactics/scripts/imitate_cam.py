#!/usr/bin/env python

import numpy as np

import argparse

import rospy
from visualization_msgs.msg import MarkerArray, Marker
from std_msgs.msg import String


def publish_pucks(publisher_pucks, coordinates, colors):
    markers = []
    for i in range(len(coordinates)):
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = rospy.Time.now()
        marker.ns = "pucks"
        marker.id = i
        marker.type = 3
        marker.pose.position.x = coordinates[i][0]
        marker.pose.position.y = coordinates[i][1]
        marker.pose.position.z = 0.0125
        marker.pose.orientation.w = 1
        marker.scale.x = 0.075
        marker.scale.y = 0.075
        marker.scale.z = 0.0125
        # redium = [r=1,g=0,b=0], grenium = [r=0,g=1,b=0], blueimium = [r=0,g=0,b=1]
        marker.color.a = 1
        if colors[i] == "red":
            marker.color.r = 1
            marker.color.g = 0
            marker.color.b = 0
        elif colors[i] == "green":
            marker.color.r = 0
            marker.color.g = 1
            marker.color.b = 0
        elif colors[i] == "blue":
            marker.color.r = 0
            marker.color.g = 0
            marker.color.b = 1
        marker.lifetime = rospy.Duration(70)
        markers.append(marker)
    publisher_pucks.publish(markers)


if __name__ == '__main__':
    rospy.init_node('imitate_cam_node', anonymous=True)
    publisher_pucks = rospy.Publisher("/pucks", MarkerArray, queue_size=1)  # /secondary_robot

    parser = argparse.ArgumentParser()
    parser.add_argument("-n", "--number",
                        help="puck collection number",
                        default="5")
    args = parser.parse_args()
    number = int(args.number)

    # tactics_publisher = rospy.Publisher('cmd_tactics', String, queue_size=1)
    coordinates_list = np.zeros((4, 2))
    rospy.sleep(2)
    print (number)
    colors = ["green", "blue", "red", "red"]
    # if number == 1:  # fails
    #     coordinates_list = np.array([[0.7, 0.8],
    #                             [0.9, 0.9],
    #                             [1.1, 0.85],
    #                             [0.9, 1.1],
    #                             [1, 1.1],
    #                             [1, 1.1],
    #                             [0.8, 1]])
    if number == 2:  # works
        coordinates_list = np.array([[0.9, 0.9],
                                    [0.8, 1],
                                    [1, 1.1],
                                    [1.1, 1]])
    elif number == 3:  # works
        coordinates_list = np.array([[0.91, 0.9],
                                    [0.9, 0.82],
                                    [0.99, 0.82],
                                    [1.0, 0.91]])

    elif number == 13:  # fails on the first approach
        coordinates_list = np.array([[0.9, 0.9],
                                    [0.9, 0.82],
                                    [0.98, 0.82],
                                    [0.98, 0.9]])

    elif number == 4:  # works - one puck in center
        coordinates_list = np.array([[0.7, 0.8],
                                    [0.9, 0.9],
                                    [1.1, 0.85],
                                    [0.9, 1.1]])
    elif number == 5:  # works
        coordinates_list = np.array([[0.9, 0.9],
                                    [1.06, 0.98],
                                    [0.98, 0.82],
                                    [0.98, 0.9]])

    elif number == 6:  # works - horizontal line
        coordinates_list = np.array([[0.85, 0.89],
                                    [0.9, 1.02],
                                    [0.88, 1.12],
                                    [0.9, 1.25]])

    # Need to test how it moves when the closest puck is in the middle of the line
    elif number == 16:  # fails - horizontal line
        coordinates_list = np.array([[0.9, 0.9],
                                    [0.9, 1],
                                    [0.9, 1.1],
                                    [0.9, 1.2]])

    elif number == 7:  # works - 1-3 diagonal
        coordinates_list = np.array([[1.2, 0.9],
                                     [1.1, 1],
                                     [1, 1.1],
                                     [0.9, 1.2]])
    elif number == 8:  # works - 1-3 diagonal
        coordinates_list = np.array([[1.21, 0.91],
                                     [1.15, 0.99],
                                     [1.01, 1.09],
                                     [0.91, 1.19]])

    elif number == 9:  # works - 2-4 diagonal
        coordinates_list = np.array([[0.89, 0.93],
                                    [1.03, 1.01],
                                    [1.15, 1.1],
                                    [1.18, 1.21]])

    elif number == 19:  # fails - 2-4 diagonal
        coordinates_list = np.array([[0.9, 0.9],
                                    [1, 1],
                                    [1.1, 1.1],
                                    [1.2, 1.2]])
    elif number == 10:  # works - 2-4 non-perfect diagonal
        coordinates_list = np.array([[0.91, 0.9],
                                    [0.99, 1],
                                    [1.12, 1.1],
                                    [1.18, 1.2]])

    elif number == 11:  # works - arc outer convexivity
        coordinates_list = np.array([[0.9, 0.9],
                                    [0.92, 1],
                                    [0.93, 1.1],
                                    [0.9, 1.2]])

# x = 1.0
# y = 1.05
# r = 0.38
# d = 0.76
# D = 0.3

    if number == 20:  # works  - Real CHAOS
        coordinates_list = np.array([[0.9, 0.9],
                                    [0.8, 1],
                                    [1, 1.1],
                                    [1.1, 1]])


    # coordinates_list = np.array([[0.5, 0.5],
    #                              [0.9, 1.1]])

    publish_pucks(publisher_pucks, coordinates_list, colors)
    # rospy.sleep(2)
    # tactics_publisher.publish('abc collect_chaos')
    # rospy.spin()  # FIXME
