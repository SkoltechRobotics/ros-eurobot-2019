#!/usr/bin/env python

import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError
import rospy
import sensor_msgs
from sensor_msgs.msg import Image
import sys, re
import yaml
import matplotlib.pyplot as plt

import time
import homogeneous

# DIM=(2448, 2048)
# K=np.array([[677.6729654637016, 0.0, 1235.371481335309], [0.0, 679.4736576804626, 1043.9887347932538], [0.0, 0.0, 1.0]])
# D=np.array([[0.026055346132657364], [0.006035766757842894], [0.005666324884814231], [-0.0015661403498746557]])

# FIXME::add camera_info

TABLE_LENGTH_Y=2.0
TABLE_LENGTH_X=3.0
TABLE_LENGTH_Z=1.0

CAMERA_WIDTH=2448.
CAMERA_HEIGHT=2048.
HOMO_IMAGE_WIDTH=400.
HOMO_IMAGE_HEIGHT=300.
SCALE_X = CAMERA_WIDTH/HOMO_IMAGE_WIDTH
SCALE_Y = CAMERA_HEIGHT/HOMO_IMAGE_HEIGHT

CAMERA_X = 1.5
CAMERA_Y = 0.0
CAMERA_Z = 1.0
CAMERA_ANGLE = (3 * np.pi / 4 - 0.1, 0, 0)

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

def find_rotation_matrix(theta):
    R_x = np.array([[1, 0, 0],
                    [0, np.cos(theta[0]),-np.sin(theta[0]) ],
                    [0, np.sin(theta[0]), np.cos(theta[0])]])
    R_y = np.array([[np.cos(theta[1]), 0, np.sin(theta[1])], 
                    [0, 1, 0],
                    [-np.sin(theta[1]), 0, np.cos(theta[1])]])
    R_z = np.array([[np.cos(theta[2]), -np.sin(theta[2]), 0],
                    [np.sin(theta[2]), np.cos(theta[2]), 0],
                    [0, 0, 1]])
    R = np.linalg.multi_dot([R_x, R_y, R_z])
    return R

class Camera():
    def __init__(self, DIM, K, D):
        self.DIM = DIM
        self.K_camera = K
        self.K_projection = K
        self.D = D
        self.warp_matrix = np.eye(3, 3, dtype=np.float32)
        
    def find_vertical_projection(self):
        rotation_matrix = find_rotation_matrix(CAMERA_ANGLE)
        camera_position = np.array([ [CAMERA_X],
                                     [CAMERA_Y],
                                     [CAMERA_Z] ])
        t = -np.dot(rotation_matrix, camera_position)
        # t = -rotation_matrix.dot(camera_position)
        M = np.concatenate((rotation_matrix, t), axis=1)
        L = np.array([ [TABLE_LENGTH_X/CAMERA_WIDTH, 0, 0],
                       [0, -TABLE_LENGTH_Y/CAMERA_HEIGHT, TABLE_LENGTH_Y],
                       [0, 0, 0],
                       [0, 0, 1] ])
        H = np.dot(M, L)
        
        K_new = np.linalg.inv(H)
        
        self.K_projection = K_new
        
        return K_new        
        
    def undistort(self, img):
        undistorted_img = cv2.fisheye.undistortImage(distorted=img,
                                                     K=self.K_camera,
                                                     D=self.D,
                                                     Knew=self.K_projection)
        return undistorted_img
    
    def find_warp_matrix_not_feature(self, undistorted_img):
        warp_matrix = homogeneous.homogeneous(undistorted_img,
                                              HOMO_IMAGE_WIDTH,
                                              HOMO_IMAGE_HEIGHT)
        self.warp_matrix = warp_matrix
        print warp_matrix
        return warp_matrix
    
    def find_warp_matrix_feature(self, undistorted_img):
        warp_matrix = homogeneous.alignImages(undistorted_img,
                                              HOMO_IMAGE_WIDTH,
                                              HOMO_IMAGE_HEIGHT)
        self.warp_matrix = warp_matrix
        print warp_matrix
        return warp_matrix
    
    
    def find_vertical_warp_projection(self, warp_matrix):
        print warp_matrix
        C = np.zeros((3,3))
        C[(0,1,2),(0,1,2)] = SCALE_X, SCALE_Y, 1
        W = warp_matrix
        C_inv = np.linalg.inv(C)
        W_inv = np.linalg.inv(W)
        K1 = C_inv.dot(self.K_projection)
        K2 = W_inv.dot(K1)
        K3 = C.dot(K2)
        K_new = K3
        self.K_projection = K_new
        return K_new
    
class CameraUndistortNode():
    def __init__(self, DIM, K, D):
        self.node = rospy.init_node('camera_undistort_node', anonymous=True)
        self.publisher = rospy.Publisher("/undistorted_image", Image)
        self.subscriber = rospy.Subscriber("/usb_cam/image_raw", sensor_msgs.msg.Image, self.__callback)
        self.bridge = CvBridge()
        self.camera = Camera(DIM, K, D)
        self.it = 0
        
    
    
#     def horizontal_proj(self):
#         rotation_matrix = self.__eulerAnglesToRotationMatrix((3 * np.pi / 4 - 0.1, 0, 0))
#         camera_position = np.array([[1.5, 0.0, 1.0]]).T
#         t = -rotation_matrix.dot(camera_position)
#         print t
#         M = np.concatenate((rotation_matrix, t), axis=1)
#         L = np.array([[lx/hx, 0, 0], [0, 0, ly], [0, -lz/hy, lz], [0, 0, 1.0]])	
#         K_new = np.linalg.inv(M.dot(L))
        
#         return K_new


    def __callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            rospy.loginfo(rospy.get_caller_id())
        except CvBridgeError as e:
            print(e)
        
        cv_image = cv2.medianBlur(cv_image,5)
        #cv_image = cv2.GaussianBlur(cv_image,(5,5),0)
        undistorted_image = self.camera.undistort(cv_image)
        image = undistorted_image
#         warp_matrix = self.camera.find_warp_matrix_feature(undistorted_image)
#         self.camera.find_vertical_warp_projection(self.camera.warp_matrix)
        if self.it == 0:
            warp_matrix = self.camera.find_warp_matrix_feature(undistorted_image)
            self.camera.find_vertical_warp_projection(self.camera.warp_matrix)
            self.it += 1
            return

#         if self.it == 1:
#             field = cv2.imread("/home/alexey/Desktop/field.png")
#             field = cv2.resize(field, (2448,2048))
#             image1 = undistorted_image*0.5+field*0.5
#             cv2.imwrite("field_img_feature.jpg", image1)
#             self.it += 1
        try:
            field = cv2.imread("/home/alexey/Desktop/field.png")
            field = cv2.resize(field, (2448,2048))
            image1 = undistorted_image // 2 + field // 2
            self.publisher.publish(self.bridge.cv2_to_imgmsg(image1, "bgr8"))
        except CvBridgeError as e:
            print(e)


if __name__ == '__main__':
    conf_path_reg = re.compile(r'config_path=*')
    conf_file = None
    for arg in sys.argv[1:]:
        if conf_path_reg.match(arg):
            try:
                conf_file = open(arg[12:], 'r')
            except IOError as err:
                sys.exit('Incorrect argument balance=')

    if not conf_file:
        print('Using default config file path')
        try:
            conf_file = open(
                '/home/alexey/CatkinWorkspace/src/ros-eurobot-2019/eurobot_camera/configs/calibration.yaml', 'r')
        except IOError as err:
            sys.exit("Couldn't find config file")

            
    DIM, K, D = read_config(conf_file)

    try:
        undistort_node = CameraUndistortNode(DIM, K, D)
        undistort_node.camera.find_vertical_projection()
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()


