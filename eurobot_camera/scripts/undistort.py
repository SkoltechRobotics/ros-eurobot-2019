#!/usr/bin/env python

import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError
import rospy
import sensor_msgs
from sensor_msgs.msg import Image
import sys, re
import yaml

import time
import homogeneous

# DIM=(2448, 2048)
# K=np.array([[677.6729654637016, 0.0, 1235.371481335309], [0.0, 679.4736576804626, 1043.9887347932538], [0.0, 0.0, 1.0]])
# D=np.array([[0.026055346132657364], [0.006035766757842894], [0.005666324884814231], [-0.0015661403498746557]])

# FIXME::add camera_info

ly=2.0
lx=3.0
lz=1.0
hx=2448.
hy=2048.
rx=400.
ry=300.
scale_x = hx/rx
scale_y = hy/ry



def read_config(conf_file):
    data_loaded = yaml.load(conf_file)

    DIM = (data_loaded['image_width'], data_loaded['image_height'])
    print 'DIM:', DIM
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

    D = np.array([[_D[0][0]], [_D[0][1]], [_D[0][2]], [_D[0][3]]])
    print 'distortion coefficents:', D

    return DIM, K, D

class undistortion():
    def __init__(self, DIM, K, D, balance=1.0):
        self.node = rospy.init_node('undistort_node', anonymous=True)
        self.publisher = rospy.Publisher("/undistort_image", Image, queue_size=10)
        self.subscriber = rospy.Subscriber("/usb_cam/image_raw", sensor_msgs.msg.Image, self.__callback)
        self.bridge = CvBridge()
        self.DIM = DIM
        self.K = K
        self.D = D
        self.balance = balance
        self.warp_matrix = np.eye(3, 3, dtype=np.float32)
        self.it = 0
        
    def undistort(self, img):
        Knew = self.K.copy()
        Knew[(0, 1), (0, 1)] = self.balance * Knew[(0, 1), (0, 1)]  # coeff to change dimension
        Knew = self.vertical_proj()
        undistorted_img = cv2.fisheye.undistortImage(img, K=self.K, D=self.D, Knew=Knew)
        # If first time find homogeneous matrix
        if np.array_equal(self.warp_matrix,np.eye(3, 3, dtype=np.float32)):
            self.warp_matrix = homogeneous.homogeneous(img=undistorted_img, rx=int(rx), ry=int(ry))
        return undistorted_img

    def vertical_proj(self):
        rotation_matrix = self.__eulerAnglesToRotationMatrix((3 * np.pi / 4 - 0.1, 0, 0))
        camera_position = np.array([[1.5, 0.0, 1.0]]).T
        t = -rotation_matrix.dot(camera_position)
        M = np.concatenate((rotation_matrix, t), axis=1)
        L = np.array([[lx/hx, 0, 0], [0, -ly/hy, ly], [0, 0, 0], [0, 0, 1]])
        K_new = np.linalg.inv(M.dot(L)) 
        
        if not (np.array_equal(self.warp_matrix,np.eye(3, 3, dtype=np.float32))):
            C = np.zeros((3,3))
            C[(0,1,2),(0,1,2)] =scale_x,scale_y,1
            W = self.warp_matrix
            C_inv = np.linalg.inv(C)
            W_inv = np.linalg.inv(W)
            K1 = C_inv.dot(K_new)
            K2 = W_inv.dot(K1)
            K3 = C.dot(K2)
            K_new = K3
            
        return K_new
        
    
    def horizontal_proj(self):
        rotation_matrix = self.__eulerAnglesToRotationMatrix((3 * np.pi / 4 - 0.1, 0, 0))
        camera_position = np.array([[1.5, 0.0, 1.0]]).T
        t = -rotation_matrix.dot(camera_position)
        print t
        M = np.concatenate((rotation_matrix, t), axis=1)
        L = np.array([[lx/hx, 0, 0], [0, 0, ly], [0, -lz/hy, lz], [0, 0, 1.0]])	
        K_new = np.linalg.inv(M.dot(L))
        
        return K_new

    def __eulerAnglesToRotationMatrix(self,theta):
        R_x = np.array([[1, 0, 0], [0, np.cos(theta[0]), -np.sin(theta[0])],
                        [0, np.sin(theta[0]),
                         np.cos(theta[0])]])
        R_y = np.array([[np.cos(theta[1]), 0,
                         np.sin(theta[1])], [0, 1, 0],
                        [-np.sin(theta[1]), 0,
                         np.cos(theta[1])]])
        R_z = np.array([[np.cos(theta[2]), -np.sin(theta[2]), 0],
                        [np.sin(theta[2]), np.cos(theta[2]), 0], [0, 0, 1]])
        R = np.dot(R_x, np.dot(R_y, R_z))
        # R = np.dot(R_z, np.dot(R_y, R_x))
        return R

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
            conf_file = open(
                '/home/alexey/CatkinWorkspace/src/ros-eurobot-2019/eurobot_camera/configs/calibration.yaml', 'r')
        except IOError as err:
            sys.exit("Couldn't find config file")
    if not balance:
        print('Using default balance value, balance=1.0')
        balance = 1.0

    DIM, K, D = read_config(conf_file)

    try:
        time.sleep(1)
        undistortion = undistortion(DIM, K, D, balance)
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()


