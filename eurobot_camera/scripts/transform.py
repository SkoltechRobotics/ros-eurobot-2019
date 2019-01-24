#!/usr/bin/env python

import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError
import rospy
import sensor_msgs
from sensor_msgs.msg import Image

import time

def find_transform_ecc(img,rx,ry,templ_path):
    start_time = time.time()
    print ('Start homogeneous')
    
    tmp =  cv2.imread(templ_path);
    tmp = cv2.resize(tmp, (int(rx),int(ry)), interpolation = cv2.INTER_AREA)
    img = cv2.resize(img, (int(rx),int(ry)), interpolation = cv2.INTER_AREA)

    # Convert images to grayscale
    tmp_gray = cv2.cvtColor(tmp,cv2.COLOR_BGR2GRAY)
    img_gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

    # create a CLAHE object (Arguments are optional).
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
    img_gray = clahe.apply(img_gray)

    # Define the motion model
    warp_mode = cv2.MOTION_HOMOGRAPHY

    warp_matrix = np.eye(3, 3, dtype=np.float32)

    # Specify the number of iterations.
    number_of_iterations = 5000;

    # Specify the threshold of the increment
    # in the correlation coefficient between two iterations
    termination_eps = 1e-6;

    # Define termination criteria
    criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, number_of_iterations,  termination_eps)

    # Run the ECC algorithm. The results are stored in warp_matrix.
    (cc, warp_matrix) = cv2.findTransformECC (tmp_gray,img_gray,warp_matrix, warp_mode, criteria)

    result_time = time.time() - start_time
    print ('Homogeneous result time= ', result_time)
    #img_aligned = cv2.warpPerspective (img, self.warp_matrix, (img.shape[1],img.shape[0]), flags=cv2.INTER_LINEAR)
    return warp_matrix
    

def find_transform_features(im1,rx,ry,MAX_FEATURES,GOOD_MATCH_PERCENT,templ_path):
    start_time = time.time()
    print ('Start homogeneous by features')

    im2 = cv2.imread(templ_path);
    im1 = cv2.resize(im1, (int(rx),int(ry)))
    im2 = cv2.resize(im2, (int(rx),int(ry)))

    # Convert images to grayscale
    im1Gray = cv2.cvtColor(im1, cv2.COLOR_BGR2GRAY)
    im2Gray = cv2.cvtColor(im2, cv2.COLOR_BGR2GRAY)

    # create a CLAHE object (Arguments are optional).
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
    im1Gray = clahe.apply(im1Gray)

    # Detect ORB features and compute descriptors.
    orb = cv2.ORB_create(MAX_FEATURES)
    
    (keypoints1, descriptors1) = orb.detectAndCompute(im1Gray, None)
    (keypoints2, descriptors2) = orb.detectAndCompute(im2Gray, None)

    # Match features.
    matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
    matches = matcher.match(descriptors1, descriptors2)

    # Sort matches by score
    matches.sort(key=lambda x: x.distance, reverse=False)

    # Remove not so good matches
    numGoodMatches = int(len(matches) * GOOD_MATCH_PERCENT)
    matches = matches[:numGoodMatches]

    # Draw top matches
    imMatches = cv2.drawMatches(im1, keypoints1, im2, keypoints2, matches, None)
    cv2.imwrite("matches.jpg", imMatches)

    # Extract location of good matches
    points1 = np.zeros((len(matches), 2), dtype=np.float32)
    points2 = np.zeros((len(matches), 2), dtype=np.float32)

    for i, match in enumerate(matches):
        points1[i, :] = keypoints1[match.queryIdx].pt
        points2[i, :] = keypoints2[match.trainIdx].pt

    # Find homography
    h, mask = cv2.findHomography(points1, points2, cv2.RANSAC)
    h = np.linalg.inv(h)
    
    # Use homography
    height, width, channels = im2.shape
    #im1Reg = cv2.warpPerspective(im1, h, (width, height))

    result_time = time.time() - start_time
    print ('Homogeneous result time= ', result_time)

    return h
