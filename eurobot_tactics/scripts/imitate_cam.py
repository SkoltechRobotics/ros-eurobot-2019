#!/usr/bin/env python

import numpy as np

import argparse

import rospy
from visualization_msgs.msg import MarkerArray, Marker
from std_msgs.msg import String


def publish_pucks(publisher_pucks, coordinates):
    markers = []
    for num, coord in enumerate(coordinates):
        # print (num,"    coord", coord)
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = rospy.Time.now()
        marker.ns = "pucks"
        marker.id = num
        marker.type = 3
        marker.pose.position.x = coord[0]
        marker.pose.position.y = coord[1]
        marker.pose.position.z = 0.0125
        marker.pose.orientation.w = 1
        marker.scale.x = 0.075
        marker.scale.y = 0.075
        marker.scale.z = 0.0125
        marker.color.a = 1
        marker.color.r = 1
        marker.lifetime = rospy.Duration(70)
        markers.append(marker)

    # print (markers)
    publisher_pucks.publish(markers)


if __name__ == '__main__':
    rospy.init_node('imitate_cam_node', anonymous=True)
    publisher_pucks = rospy.Publisher("/secondary_robot/pucks", MarkerArray, queue_size=1)

    parser = argparse.ArgumentParser()
    parser.add_argument("-n", "--number",
                        help="puck collection number",
                        default="1")
    args = parser.parse_args()
    number = int(args.number)

    # tactics_publisher = rospy.Publisher('cmd_tactics', String, queue_size=1)
    coordinates_list = np.zeros((4, 2))
    rospy.sleep(2)
    print (number)

    if number == 1: # fails
        coordinates_list = np.array([[0.7, 0.8],
                                [0.9, 0.9],
                                [1.1, 0.85],
                                [0.9, 1.1],
                                [1, 1.1],
                                [1, 1.1],
                                [0.8, 1]])
    elif number == 2:  # works
        coordinates_list = np.array([[0.9, 0.9],
                                    [0.8, 1],
                                    [1, 1.1],
                                    [1.1, 1]])
    elif number == 3:  # fails
        coordinates_list = np.array([[0.9, 0.9],
                                    [0.9, 0.82],
                                    [0.98, 0.82],
                                    [0.98, 0.9]])
    elif number == 4:  # works
        coordinates_list = np.array([[0.7, 0.8],
                                    [0.9, 0.9],
                                    [1.1, 0.85],
                                    [0.9, 1.1]])
    elif number == 5:  # fails
        coordinates_list = np.array([[0.9, 0.9],
                                    [1.06, 0.98],
                                    [0.98, 0.82],
                                    [0.98, 0.9]])

    # works but only if robot moves from side.
    # Need to test how it moves when the closest puck is in the middle of the line
    elif number == 6:
        coordinates_list = np.array([[0.9, 0.9],
                                    [0.9, 1],
                                    [0.9, 1.1],
                                    [0.9, 1.2]])

    elif number == 7:  # 1-3 diagonal fails
        coordinates_list = np.array([[1.2, 0.9],
                                     [1.1, 1],
                                     [1, 1.1],
                                     [0.9, 1.2]])
    elif number == 8:  # 1-3 diagonal fails
        coordinates_list = np.array([[1.21, 0.91],
                                     [1.15, 0.99],
                                     [1.01, 1.09],
                                     [0.91, 1.19]])

    # 2-4 diagonal ok
    # coordinates_list = np.array([[0.9, 0.9],
    #                             [1, 1],
    #                             [1.1, 1.1],
    #                             [1.2, 1.2]])

    # coordinates_list = np.array([[0.5, 0.5],
    #                              [0.9, 1.1]])

    publish_pucks(publisher_pucks, coordinates_list)
    # rospy.sleep(2)
    # tactics_publisher.publish('abc collect_chaos')
    # rospy.spin()  # FIXME
