#!/usr/bin/env python

from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import cv2
import roslib
import rospy
import sensor_msgs
from sensor_msgs.msg import Image
import sys

#FIXME: get coeff from file
DIM=(2448, 2048)
K=np.array([[677.6729654637016, 0.0, 1235.371481335309], [0.0, 679.4736576804626, 1043.9887347932538], [0.0, 0.0, 1.0]])
D=np.array([[0.026055346132657364], [0.006035766757842894], [0.005666324884814231], [-0.0015661403498746557]])

#FIXME: create class to remove global values
image_pub = rospy.Publisher("MY_IMAGE_TOPIC",Image)

def distortion(img,balance=0.0, dim2=None, dim3=None):
    dim1 = img.shape[:2][::-1]  #dim1 is the dimension of input image to un-distort
    assert dim1[0]/dim1[1] == DIM[0]/DIM[1], "Image to undistort needs to have same aspect ratio as the ones used in calibration"
    if not dim2:
        dim2 = dim1
    if not dim3:
        dim3 = dim1
    scaled_K = K * dim1[0] / DIM[0]  # The values of K is to scale with image dimension.
    scaled_K[2][2] = 1.0  # Except that K[2][2] is always 1.0
    # This is how scaled_K, dim2 and balance are used to determine the final K used to un-distort image. OpenCV document failed to make this clear!
    new_K = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(scaled_K, D, dim2, np.eye(3), balance=balance)
    map1, map2 = cv2.fisheye.initUndistortRectifyMap(scaled_K, D, np.eye(3), new_K, dim3, cv2.CV_16SC2)
    undistorted_img = cv2.remap(img, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
    return undistorted_img

def callback(data):
    try:
	print(10)
        br = CvBridge()
        cv_image = br.imgmsg_to_cv2(data, "bgr8")
        print(cv_image)
	rospy.loginfo(rospy.get_caller_id())
    except CvBridgeError as e:
        print(e)

    image = distortion(cv_image)
    try:
        image_pub.publish(br.cv2_to_imgmsg(image, "bgr8"))
    except CvBridgeError as e:
        print(e)

def talker():
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(30) # 30hz
    while not rospy.is_shutdown():
       hello_str = "hello world %s" % rospy.get_time()
       rospy.loginfo(hello_str)
       pub.publish(hello_str)
       rate.sleep()

rospy.init_node('camera_main_node', anonymous=True)
rospy.Subscriber("/usb_cam/image_raw", sensor_msgs.msg.Image, callback)
try:
    rospy.spin()
except KeyboardInterrupt:
    print("Shutting down")
cv2.destroyAllWindows()


