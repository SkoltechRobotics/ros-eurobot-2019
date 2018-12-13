#!/usr/bin/env python

from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import cv2
import roslib
import rospy
import sensor_msgs
import sys

def callback(data):
    try:
	print(10)
        br = CvBridge()
        cv_image = br.imgmsg_to_cv2(data, "bgr8")
        print(cv_image)
	rospy.loginfo(rospy.get_caller_id())
    except CvBridgeError as e:
        print(e)

    cv2.imshow("Image window", cv_image)
    cv2.waitKey(3)

def main(args):
    print(1)
    rospy.init_node('camera_test_node', anonymous=True)
    print(2)
    rospy.Subscriber("/usb_cam/image_raw", sensor_msgs.msg.Image, callback)
    print(3)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()


if __name__ == '__main__':
    print(0)
    main(sys.argv)


