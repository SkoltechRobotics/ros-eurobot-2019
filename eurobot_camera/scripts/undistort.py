#!/usr/bin/env python
import numpy as np
import cv2

from cv_bridge import CvBridge, CvBridgeError
import rospy
from sensor_msgs.msg import Image
from visualization_msgs.msg import MarkerArray, Marker

import yaml
import copy
import argparse
import time

from camera import Camera
import image_processing
from contours import Contour

import sys


def read_config(conf_file):
    data_loaded = yaml.load(conf_file)

    DIM = (data_loaded['image_width'], data_loaded['image_height'])
    rospy.loginfo("DIM:", DIM)
    camera_matrix = data_loaded['camera_matrix']
    data = camera_matrix['data']
    rows = camera_matrix['rows']
    cols = camera_matrix['cols']

    _K = []
    for i in range(rows):
        _K.append(data[i * rows:i * rows + cols:1])

    K = np.array(_K)
    rospy.loginfo("CAMERA MATRIX:\n", K)

    distortion_coefficients = data_loaded['distortion_coefficients']
    data = distortion_coefficients['data']
    rows = distortion_coefficients['rows']
    cols = distortion_coefficients['cols']

    _D = []
    for i in range(rows):
        _D.append(data[i * rows:i * rows + cols:1])

    D = np.array([[_D[0][0]], [_D[0][1]], [_D[0][2]], [_D[0][3]]])
    rospy.loginfo("DISTORION COEFFICIENTS:\n", D)

    return DIM, K, D


class CameraUndistortNode():
    def __init__(self, DIM, K, D, template):
        self.templ_path = template

        self.node = rospy.init_node('camera_node', anonymous=True)
        self.publisher_undistorted = rospy.Publisher("/undistorted_image", Image, queue_size=1)
        self.publisher = rospy.Publisher("/recognition_image", Image, queue_size=1)
        self.publisher_gray = rospy.Publisher("/gray_scale_image", Image, queue_size=1)
        self.publisher_contours = rospy.Publisher("/contours_image", Image, queue_size=1)
        self.publisher_filter_contours = rospy.Publisher("/filtered_contours_image", Image, queue_size=1)
        self.publisher_pucks = rospy.Publisher("/pucks", MarkerArray, queue_size=1)

        self.bridge = CvBridge()
        self.camera = Camera(DIM, K, D)
        self.contour = Contour()

        self.subscriber = rospy.Subscriber("/usb_cam/image_raw", Image,
                                           self.__callback, queue_size=1)

    def publish_pucks(self, coordinates, colors):
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
            marker.lifetime = rospy.Duration(3)
            markers.append(marker)
        self.publisher_pucks.publish(markers)

    def __callback(self, data):
        start_time = time.time()
        image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        rospy.loginfo(rospy.get_caller_id())

        # Process image
        image = image_processing.equalize_histogram(image, 1.0, (21,21))
        undistorted_image = self.camera.undistort(image)
        image = undistorted_image

        # Align image using field template
        if self.camera.align_image(image, self.templ_path):
            image = image_processing.decrease_noise(image, 15, 100, 100)
            # image = image_processing.equalize_histogram(image)


            # Find all contours on the image
            image_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            self.contour.find(image_gray)
            image_contours = copy.copy(image)
            image_contours = self.contour.draw(image_contours, self.contour.all_contours)

            # Filter contours
            contours_filtered = self.contour.filter(1000, 100, 200)
            image_filter_contours = copy.copy(image)
            image_filter_contours = self.contour.draw(image_filter_contours, contours_filtered)

            # Find pucks coordinates
            image_pucks = copy.copy(image)
            coordinates = self.contour.find_pucks_coordinates()
            # Detect contours colors
            colors = self.contour.detect_color(contours_filtered, image)
            # Draw ellipse contours around pucks
            image_pucks = self.contour.draw_ellipse(image_pucks, contours_filtered, coordinates, colors)

            # Publish all images to topics
            self.publisher_undistorted.publish(self.bridge.cv2_to_imgmsg(image, "bgr8"))
            self.publisher_gray.publish(self.bridge.cv2_to_imgmsg(image_gray))
            self.publisher_contours.publish(self.bridge.cv2_to_imgmsg(image_contours, "bgr8"))
            self.publisher_filter_contours.publish(self.bridge.cv2_to_imgmsg(image_filter_contours, "bgr8"))
            self.publisher.publish(self.bridge.cv2_to_imgmsg(image_pucks, "bgr8"))

            # Publish pucks coordinates
            self.publish_pucks(coordinates, colors)

            res_time = time.time() - start_time
            rospy.loginfo("RESULT TIME = " + str(res_time))


if __name__ == '__main__':
    sys.argv = rospy.myargv()
    parser = argparse.ArgumentParser()
    parser.add_argument("-c", "--config",
                        help="path to camera config yaml file",
                        default="../configs/calibration.yaml")
    parser.add_argument("-t", "--template",
                        help="path to field's template file",
                        default="../configs/field.png")
    args = parser.parse_args()

    print ("ARGS.config", args.config)
    try:
        conf_file = open(args.config, 'r')
    except IOError as err:
        sys.exit("Couldn't find config file")

    DIM, K, D = read_config(conf_file)

    time.sleep(1)
    undistort_node = CameraUndistortNode(DIM, K, D, args.template)
    undistort_node.camera.find_vertical_projection()

rospy.spin()
