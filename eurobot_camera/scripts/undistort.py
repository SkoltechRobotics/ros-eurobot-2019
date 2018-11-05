#!/usr/bin/env python

import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError
import rospy
import sensor_msgs
from sensor_msgs.msg import Image
import sys, re
import yaml

def read_config(conf_file):
    data_loaded = yaml.load(conf_file)

    DIM = (data_loaded['image_width'], data_loaded['image_height'])
    print 'DIM:',DIM
    camera_matrix = data_loaded['camera_matrix']
    data = camera_matrix['data']
    rows = camera_matrix['rows']
    cols = camera_matrix['cols']

    _K = []
    for i in range(rows):
        _K.append(data[i * rows:i * rows + cols:1])

    K = np.array(_K)
    print 'camera matrix:', K
    
    distortion_coefficients = data_loaded['distortion_coefficients']
    data = distortion_coefficients['data']
    rows = distortion_coefficients['rows']
    cols = distortion_coefficients['cols']

    _D = []
    for i in range(rows):
        _D.append(data[i * rows:i * rows + cols:1])

    D = np.array([[_D[0][0]],[_D[0][1]],[_D[0][2]],[_D[0][3]]])
    print 'distortion coefficents:', D
    
    return DIM, K, D


class undistortion():
    def __init__(self, DIM, K, D, balance=1.0):
        self.node = rospy.init_node('undistort_node', anonymous=True)
        self.publisher = rospy.Publisher("/undistort_image", Image)
        self.subscriber = rospy.Subscriber("/usb_cam/image_raw", sensor_msgs.msg.Image, self.__callback)
        self.bridge = CvBridge()
        self.DIM = DIM
        self.K = K
        self.D = D
        self.balance = balance

    def undistort(self, img):
        Knew = self.K.copy()
        Knew[(0,1), (0,1)] = self.balance * Knew[(0,1), (0,1)] # coeff to change dimension
        undistorted_img = cv2.fisheye.undistortImage(img, K=self.K, D=self.D, Knew=Knew)
        return undistorted_img

    def __callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            rospy.loginfo(rospy.get_caller_id())
        except CvBridgeError as e:
            print(e)
        image = self.undistort(cv_image)
        try:
            self.publisher.publish(self.bridge.cv2_to_imgmsg(image, "bgr8"))
        except CvBridgeError as e:
            print(e)


if __name__ == '__main__':
    conf_path_reg = re.compile(r'config_path=*')
    balance_reg = re.compile(r'balance=*')
    balance = None
    conf_file = None
    for arg in sys.argv[1:]:
        if conf_path_reg.match(arg):
            try:
                conf_file = open(arg[12:], 'r')
            except IOError as err:
                sys.exit('Incorrect argument balance=')
        if balance_reg.match(arg):
            try:
                balance = float(arg[8:])
            except ValueError as err: 
                sys.exit('Incorrect argument balance=')
                
    if not conf_file:
        print('Using default config file path')
        try:
            conf_file = open('/home/alexey/CatkinWorkspace/src/ros-eurobot-2019/eurobot_camera/configs/calibration.yaml', 'r')
        except IOError as err:
            sys.exit("Couldn't find config file")
    if not balance:
        print('Using default balance value, balance=1.0')
        balance = 1.0
        
    DIM, K, D = read_config(conf_file)

    try:
        undistortion = undistortion(DIM, K, D, balance)
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()


