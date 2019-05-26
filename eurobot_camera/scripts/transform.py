#!/usr/bin/env python
import cv2
import numpy as np

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
    # img_gray = clahe.apply(img_gray)
    # img_gray = clahe.apply(img_gray)

    # Define the motion model
    warp_mode = cv2.MOTION_HOMOGRAPHY

    warp_matrix = np.eye(3, 3, dtype=np.float32)

    # Specify the number of iterations.
    number_of_iterations = 500

    # Specify the threshold of the increment
    # in the correlation coefficient between two iterations
    termination_eps = 1e-5

    # Define termination criteria
    criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, number_of_iterations,  termination_eps)

    # Run the ECC algorithm. The results are stored in warp_matrix.
    (cc, warp_matrix) = cv2.findTransformECC (tmp_gray, img_gray, warp_matrix, warp_mode, criteria)

    result_time = time.time() - start_time
    print ('Homogeneous result time= ', result_time)
    # img_aligned = cv2.warpPerspective (img, self.warp_matrix, (img.shape[1],img.shape[0]), flags=cv2.INTER_LINEAR)
    return warp_matrix
    

def find_transform_features(im1,rx,ry,MAX_FEATURES,GOOD_MATCH_PERCENT,templ_path):
    start_time = time.time()
    print ('Start homogeneous by features')

    MIN_MATCH_COUNT = 20
    im2 = cv2.imread(templ_path)
    im1 = cv2.resize(im1, (int(rx),int(ry)))
    im2 = cv2.resize(im2, (int(rx),int(ry)))
    cv2.imwrite("asdad.png", im1)
    # Convert images to grayscale
    im1Gray = cv2.cvtColor(im1, cv2.COLOR_BGR2GRAY)
    im2Gray = cv2.cvtColor(im2, cv2.COLOR_BGR2GRAY)

    # # create a CLAHE object (Arguments are optional).
    clahe = cv2.createCLAHE(clipLimit=1.0, tileGridSize=(21,21))
    # im1Gray = clahe.apply(im1Gray)



    # im1Gray = cv2.addWeighted(im1Gray, 1, laplace, -1, 0)


    # # Create our shapening kernel, it must equal to one eventually
    # kernel_sharpening = np.array([[-1, -1, -1],
    #                               [-1, 9, -1],
    #                               [-1, -1, -1]])
    # # applying the sharpening kernel to the input image & displaying it.
    # im1Gray = cv2.filter2D(im1Gray, -1, kernel_sharpening)


    sift = cv2.xfeatures2d.SIFT_create()


    (keypoints1, descriptors1) = sift.detectAndCompute(im1Gray, None)
    print("Descriptor1 {} with shape {}".format(descriptors1, descriptors1.shape))
    (keypoints2, descriptors2) = sift.detectAndCompute(im2Gray, None)

    # # BFMatcher with default params
    # bf = cv2.BFMatcher()
    # matches = bf.knnMatch(descriptors1, descriptors2, k=2)


    FLANN_INDEX_KDTREE = 1
    index_params = dict(algorithm=FLANN_INDEX_KDTREE, trees=5)
    search_params = dict(checks=100)


    # FLANN_INDEX_LSH = 6
    # index_params = dict(algorithm=FLANN_INDEX_LSH,
    #                     table_number=6,  # 12
    #                     key_size=12,  # 20
    #                     multi_probe_level=1)  # 2
    #
    # search_params = dict(checks=100)

    flann = cv2.FlannBasedMatcher(index_params, search_params)
    matches = flann.knnMatch(descriptors1, descriptors2, k=2)

    good = []
    for m, n in matches:
        img2_idx = m.trainIdx
        img1_idx = m.queryIdx
        pt1 = np.array(keypoints1[img1_idx].pt)
        pt2 = np.array(keypoints2[img2_idx].pt)
        dist = np.linalg.norm(pt1-pt2)
        if m.distance < 0.8 * n.distance and dist < 50:
            good.append(m)

    if len(good) > MIN_MATCH_COUNT:
        src_pts = np.float32([keypoints1[m.queryIdx].pt for m in good]).reshape(-1, 1, 2)
        dst_pts = np.float32([keypoints2[m.trainIdx].pt for m in good]).reshape(-1, 1, 2)
        M, mask = cv2.findHomography(src_pts, dst_pts)
        matchesMask = mask.ravel().tolist()
        h, w, d = im1.shape
        pts = np.float32([[0, 0], [0, h - 1], [w - 1, h - 1], [w - 1, 0]]).reshape(-1, 1, 2)
        dst = cv2.perspectiveTransform(pts, M)
        img2 = cv2.polylines(im2, [np.int32(dst)], True, 255, 3, cv2.LINE_AA)
    else:
        print("Not enough matches are found - {}/{}".format(len(good), MIN_MATCH_COUNT))
        matchesMask = None

    draw_params = dict(matchColor=(0, 255, 0),  # draw matches in green color
                       singlePointColor=None,
                       matchesMask=matchesMask,  # draw only inliers
                       flags=2)
    img3 = cv2.drawMatches(im1Gray, keypoints1, im2Gray, keypoints2, good, None, **draw_params)
    cv2.imwrite("matches.jpg", img3)


    result_time = time.time() - start_time
    print ('Homogeneous result time= ', result_time)

    M = np.linalg.inv(M)
    return M

# def find_transform_features(im1, rx, ry, MAX_FEATURES, GOOD_MATCH_PERCENT, templ_path):
#     start_time = time.time()
#     print ('Start homogeneous by features')
#
#     im2 = cv2.imread(templ_path)
#     im1 = cv2.resize(im1, (int(rx), int(ry)))
#     im2 = cv2.resize(im2, (int(rx), int(ry)))
#
#     # Convert images to grayscale
#     im1Gray = cv2.cvtColor(im1, cv2.COLOR_BGR2GRAY)
#     im2Gray = cv2.cvtColor(im2, cv2.COLOR_BGR2GRAY)
#
#     # create a CLAHE object (Arguments are optional).
#     clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
#     # im1Gray = clahe.apply(im1Gray)
#     # im1Gray = clahe.apply(im1Gray)
#     # im2Gray = clahe.apply(im2Gray)
#     # im2Gray = clahe.apply(im2Gray)
#     #
#     # Detect ORB features and compute descriptors.
#     orb = cv2.ORB_create(MAX_FEATURES)
#
#     (keypoints1, descriptors1) = orb.detectAndCompute(im1Gray, None)
#     (keypoints2, descriptors2) = orb.detectAndCompute(im2Gray, None)
#
#     # Match features.
#     matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
#     matches = matcher.match(descriptors1, descriptors2)
#     print ("MATCHES=", matches)
#     # store all the good matches as per Lowe's ratio test.
#
#     # Sort matches by score
#     matches.sort(key=lambda x: x.distance, reverse=False)
#
#     # Remove not so good matches
#     numGoodMatches = int(len(matches) * GOOD_MATCH_PERCENT)
#     matches = matches[:numGoodMatches]
#
#     # Draw top matches
#     imMatches = cv2.drawMatches(im1Gray, keypoints1, im2Gray, keypoints2, matches, None)
#     cv2.imwrite("matches.jpg", imMatches)
#
#     # Extract location of good matches
#     points1 = np.zeros((len(matches), 2), dtype=np.float32)
#     points2 = np.zeros((len(matches), 2), dtype=np.float32)
#
#     for i, match in enumerate(matches):
#         points1[i, :] = keypoints1[match.queryIdx].pt
#         points2[i, :] = keypoints2[match.trainIdx].pt
#
#     # Find homography
#     h, mask = cv2.findHomography(points1, points2, cv2.RANSAC)
#     h = np.linalg.inv(h)
#
#     # Use homography
#     height, width, channels = im2.shape
#     # im1Reg = cv2.warpPerspective(im1, h, (width, height))
#
#     result_time = time.time() - start_time
#     print ('Homogeneous result time= ', result_time)
#
#     return h