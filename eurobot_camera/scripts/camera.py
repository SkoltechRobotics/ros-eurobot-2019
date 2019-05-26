#!/usr/bin/env python
import cv2
import numpy as np

import rospy

import transform

from lib.core_functions import find_rotation_matrix

TABLE_LENGTH_X = rospy.get_param("TABLE_LENGTH_X")
TABLE_LENGTH_Y = rospy.get_param("TABLE_LENGTH_Y")
TABLE_LENGTH_Z = rospy.get_param("TABLE_LENGTH_Z")

CAMERA_WIDTH = rospy.get_param("CAMERA_WIDTH")
CAMERA_HEIGHT = rospy.get_param("CAMERA_HEIGHT")
HOMO_IMAGE_SCALE = rospy.get_param("HOMO_IMAGE_SCALE")
HOMO_IMAGE_WIDTH = CAMERA_WIDTH/HOMO_IMAGE_SCALE
HOMO_IMAGE_HEIGHT = CAMERA_HEIGHT/HOMO_IMAGE_SCALE

CAMERA_X = rospy.get_param("CAMERA_X")
CAMERA_Y = rospy.get_param("CAMERA_Y")
CAMERA_Z = rospy.get_param("CAMERA_Z")
CAMERA_ANGLE = rospy.get_param("CAMERA_ANGLE")

PUCK_RADIUS = rospy.get_param("PUCK_RADIUS")
pixel_scale_x = CAMERA_WIDTH/TABLE_LENGTH_X
pixel_scale_y = CAMERA_HEIGHT/TABLE_LENGTH_Y
PUCK_RADIUS_pixel_x = pixel_scale_x*PUCK_RADIUS
PUCK_RADIUS_pixel_y = pixel_scale_y*PUCK_RADIUS

MAX_FEATURES = rospy.get_param("MAX_FEATURES")
GOOD_MATCH_PERCENT = rospy.get_param("GOOD_MATCH_PERCENT")


class Camera():
    def __init__(self, DIM, K, D):
        self.DIM = DIM
        self.K_camera = K
        self.K_projection = K
        self.D = D
        self.is_aligned = False
        self.warp_matrix = np.eye(3, 3, dtype=np.float32)

    def find_vertical_projection(self):
        rotation_matrix = find_rotation_matrix(CAMERA_ANGLE)
        camera_position = np.array([ [CAMERA_X],
                                     [CAMERA_Y],
                                     [CAMERA_Z] ])
        t = -np.dot(rotation_matrix, camera_position)
        M = np.concatenate((rotation_matrix, t), axis=1)
        L = np.array([ [TABLE_LENGTH_X/CAMERA_WIDTH, 0, 0],
                       [0, -TABLE_LENGTH_Y/CAMERA_HEIGHT, TABLE_LENGTH_Y],
                       [0, 0, 0],
                       [0, 0, 1] ])
        H = np.dot(M, L)

        K_new = np.linalg.inv(H)

        self.K_projection = K_new

        return K_new

    def find_horizontal_projection(self):
        rotation_matrix = find_rotation_matrix(CAMERA_ANGLE)
        camera_position = np.array([ [CAMERA_X],
                                     [CAMERA_Y],
                                     [CAMERA_Z] ])
        t = -rotation_matrix.dot(camera_position)
        print t
        M = np.concatenate((rotation_matrix, t), axis=1)
        L = np.array([[TABLE_LENGTH_X/CAMERA_WIDTH, 0, 0],
                      [0, 0, TABLE_LENGTH_Y-0.3],
                      [0, -TABLE_LENGTH_Z/CAMERA_HEIGHT, TABLE_LENGTH_Z],
                      [0, 0, 1.0]])
        K_new = np.linalg.inv(M.dot(L))

        self.K_projection = K_new

        return K_new

    def align_image(self, undistorted_image, templ_path):
        if not self.is_aligned:
            self.find_warp_matrix_feature(undistorted_image, templ_path)
            self.find_vertical_warp_projection(self.warp_matrix)
            self.is_aligned = True
        return self.is_aligned

    def undistort(self, image):
        undistorted_image = cv2.fisheye.undistortImage(distorted=image,
                                                     K=self.K_camera,
                                                     D=self.D,
                                                     Knew=self.K_projection)
        return undistorted_image

    def find_warp_matrix_not_feature(self, undistorted_image, templ_path):
        self.warp_matrix = transform.find_transform_ecc(undistorted_image,
                                                   HOMO_IMAGE_WIDTH,
                                                   HOMO_IMAGE_HEIGHT,
                                                   templ_path)
        return self.warp_matrix

    def find_warp_matrix_feature(self, undistorted_image, templ_path):
        self.warp_matrix = transform.find_transform_features(undistorted_image,
                                                        HOMO_IMAGE_WIDTH,
                                                        HOMO_IMAGE_HEIGHT,
                                                        MAX_FEATURES,
                                                        GOOD_MATCH_PERCENT,
                                                        templ_path)
        return self.warp_matrix

    def find_vertical_warp_projection(self, warp_matrix):
        C = np.zeros((3,3))
        C[(0,1,2),(0,1,2)] = HOMO_IMAGE_SCALE, HOMO_IMAGE_SCALE, 1
        W = warp_matrix
        C_inv = np.linalg.inv(C)
        W_inv = np.linalg.inv(W)

        rotation_matrix = find_rotation_matrix(CAMERA_ANGLE)
        camera_position = np.array([ [CAMERA_X],
                                     [CAMERA_Y],
                                     [CAMERA_Z] ])
        t = -np.dot(rotation_matrix, camera_position)
        M = np.concatenate((rotation_matrix, t), axis=1)
        L = np.array([ [TABLE_LENGTH_X/CAMERA_WIDTH, 0, 0],
                       [0, -TABLE_LENGTH_Y/CAMERA_HEIGHT, TABLE_LENGTH_Y],
                       [0, 0, 0.025],
                       [0, 0, 1] ])
        H = np.dot(M, L)

        K_new = np.linalg.inv(H)

        K1 = C_inv.dot(K_new)
        K2 = W_inv.dot(K1)
        K3 = C.dot(K2)
        K_new = K3
        self.K_projection = K_new
        return self.K_projection







