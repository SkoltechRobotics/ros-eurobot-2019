#!/usr/bin/env python
import numpy as np

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
