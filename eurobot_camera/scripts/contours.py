#!/usr/bin/env python
import cv2
import skimage
import skimage.morphology
import skimage.segmentation
import numpy as np

import rospy

import image_processing

TABLE_LENGTH_X = rospy.get_param("TABLE_LENGTH_X")
TABLE_LENGTH_Y = rospy.get_param("TABLE_LENGTH_Y")
TABLE_LENGTH_Z = rospy.get_param("TABLE_LENGTH_Z")

CAMERA_WIDTH = rospy.get_param("CAMERA_WIDTH")
CAMERA_HEIGHT = rospy.get_param("CAMERA_HEIGHT")

PUCK_RADIUS = rospy.get_param("PUCK_RADIUS")
pixel_scale_x = CAMERA_WIDTH/TABLE_LENGTH_X
pixel_scale_y = CAMERA_HEIGHT/TABLE_LENGTH_Y
PUCK_RADIUS_pixel_x = pixel_scale_x*PUCK_RADIUS/2.0
PUCK_RADIUS_pixel_y = pixel_scale_y*PUCK_RADIUS/2.0

REDIUM_COLOR = rospy.get_param("REDIUM_COLOR")
GREENIUM_COLOR = rospy.get_param("GREENIUM_COLOR")
BLUEIUM_COLOR = rospy.get_param("BLUEIUM_COLOR")

LAB_REDIUM = rospy.get_param("LAB_REDIUM")
LAB_GREENIUM = rospy.get_param("LAB_GREENIUM")
LAB_BLUEIUM = rospy.get_param("LAB_BLUEIUM")
LAB_COLORS = np.array( (LAB_BLUEIUM, LAB_GREENIUM, LAB_REDIUM), dtype=np.int8)

class Contour():
    def __init__(self):
        self.hierarchy = []
        self.all_contours = []
        self.filtered_contours = []

    def find(self, image_gray):
        # th = cv2.adaptiveThreshold(image_gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,cv2.THRESH_BINARY, 25, 0)
        # ret, th = cv2.threshold(image_gray,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
        ret, th = cv2.threshold(image_gray, 127, 255, 0)

        # kernel = np.ones((3,3), np.uint8)
        # res = cv2.morphologyEx(th, cv2.MORPH_OPEN, kernel)

        # th1 = skimage.morphology.closing(th)
        # cleared = skimage.segmentation.clear_border(th1)
        image, contours, hierarchy = cv2.findContours(th, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

        # contours = skimage.morphology.convex_hull_image(contours)
        self.hierarchy = hierarchy
        self.all_contours = contours
        self.filtered_contours = []
        return image

    def filter(self, area_epsilon, perimeter_epsilon, radius_epsilon):
        for cnt in self.all_contours:
            perimeter = cv2.arcLength(cnt, True)

            # radius = perimeter/(2*np.pi)
            # area = np.pi*radius*radius

            area = cv2.contourArea(cnt)
            print ("AREA=", area)
            print ("NEED TO BE=", np.pi*PUCK_RADIUS_pixel_x*PUCK_RADIUS_pixel_y)

            # if area <= (np.pi*PUCK_RADIUS_pixel_x*PUCK_RADIUS_pixel_y + area_epsilon) and \
            #         area >= (np.pi *PUCK_RADIUS_pixel_x *PUCK_RADIUS_pixel_y - area_epsilon) and \
            #         perimeter <= (2* np.pi * PUCK_RADIUS_pixel_x + perimeter_epsilon) and \
            #         perimeter >= (2 * np.pi * PUCK_RADIUS_pixel_x - perimeter_epsilon) and \
            #         self.is_contour_circle(cnt, radius_epsilon) and \
            #         self.is_contour_puck(cnt):
            if area <= (np.pi*PUCK_RADIUS_pixel_x*PUCK_RADIUS_pixel_y + area_epsilon) and \
                     area >= (np.pi *PUCK_RADIUS_pixel_x *PUCK_RADIUS_pixel_y - area_epsilon):
                self.filtered_contours.append(cnt)
        return self.filtered_contours

    def draw(self, image, contours):
        # hull = []
        # for i in range(len(contours)):
        #     # creating convex hull object for each contour
        #     hull.append(cv2.convexHull(contours[i], False))
        # cv2.drawContours(image, contours, -1, color_contours, 1, 8, hierarchy)
        # # draw ith convex hull object
        # cv2.drawContours(image, hull, i, color, 1, 8)
        return cv2.drawContours(image, contours, -1, (255, 0, 0), 3)

    def draw_ellipse(self, image, contours, coordinates, colors):
        print (image.shape)
        for cnt, coord, color in zip(contours, coordinates, colors):
            # (x, y), radius = cv2.minEnclosingCircle(cnt)
            # center = (int(x), int(y))
            # radius = int(radius)
            # cv2.circle(image, center, radius, (0, 255, 0), 2)

            ellipse  = cv2.fitEllipse(cnt)
            # area = cv2.contourArea(ellipse)
            image = cv2.ellipse(image,ellipse,(0,255,0),2)

            cx = coord[0]
            cy = coord[1]

            cx_1 = cx*pixel_scale_x/2-100
            cy_1 = (2-cy)*pixel_scale_y/2-106

            image = cv2.circle(image,(int(cx_1),int(cy_1)), 2, (0, 0, 255), 3)

            print ("cx_1=",cx_1)
            print ("cy_1=", cy_1)
            # Cx = cx/pixel_scale_x
            # Cy = 2-cy/pixel_scale_y
            image = cv2.putText(img=image,
                                text="Cx=%.3f,Cy=%.3f,%s" % (cx, cy, color),
                                org=(int(cx_1),int(cy_1)),
                                fontFace=cv2.FONT_HERSHEY_SIMPLEX,
                                fontScale=1,
                                color=(255,255,255),
                                thickness=1,
                                lineType=4)
        return image

    def find_pucks_coordinates(self):
        coordinates = []
        for cnt in self.filtered_contours:
            M = cv2.moments(cnt)
            cx = M['m10']/M['m00']
            cx = 2*cx+200
            cy = M['m01']/M['m00']
            cy = 2*cy+212
            Cx = cx/pixel_scale_x
            Cy = 2-cy/pixel_scale_y
            coordinates.append((Cx,Cy))
        print ("COORDINATES=",coordinates)
        return coordinates

    def detect_color_pucks_hough(self,image, circles):
        pucks_colors = []

        for i in circles[0,:]:
            mask = np.zeros(image.shape[:2], np.uint8)

            circles = np.uint16(np.around(circles))

            cv2.circle(mask,(i[0],i[1]),i[2],255,-1)

            mean_val = cv2.mean(image, mask)
            MEAN_COLORS = np.array((mean_val[0], mean_val[1], mean_val[2]), dtype=np.uint8)
            LAB_MEAN_COLORS = skimage.color.rgb2lab(MEAN_COLORS[np.newaxis, np.newaxis, :])
            deltaE = skimage.color.deltaE_cmc(LAB_MEAN_COLORS, LAB_COLORS)

            color = np.argmin(deltaE)
            if color == 0:
                pucks_colors.append("red")
            elif color == 1:
                pucks_colors.append("green")
            elif color == 2:
                pucks_colors.append("blue")

        return pucks_colors

    def find_pucks_coordinates_hough(self, circles):
        coordinates = []

        for i in circles[0, :]:
            cx = i[0]
            cy = i[1]
            cx = 2 * cx + 200
            cy = 2 * cy + 212

            Cx = cx/pixel_scale_x
            Cy = 2-cy/pixel_scale_y
            coordinates.append((Cx,Cy))

        print ("COORDINATES=",coordinates)
        return coordinates

    def hough_detect_pucks(self, image):
        circles = cv2.HoughCircles(image, cv2.HOUGH_GRADIENT, 1, 20,
                                   param1=1, param2=7, minRadius=18, maxRadius=22)

        return circles




    def is_contour_circle(self, contour, epsilon):
        M = cv2.moments(contour)
        if (np.abs(M["mu20"]/M["m00"]-M["mu02"]/M["m00"]) <= epsilon):
            return True
        else:
            return False

    def is_contour_puck(self, contour):
        M = cv2.moments(contour)
        cx = M['m10'] / M['m00']
        cy = M['m01'] / M['m00']
        Cx = cx / pixel_scale_x
        Cy = 2 - cy / pixel_scale_y
        if Cx >= 0.5 and Cx <= 2.5 and Cy >= 0.5 and Cy <= 1.5:
            return True
        else:
            return False

    def detect_color(self, contours, image):
        image = image_processing.increase_saturation(image)

        contours_colors = []

        for cnt in contours:
            mask = np.zeros(image.shape[:2], np.uint8)
            cv2.drawContours(mask,[cnt],0,255,-1)

            cv2.imwrite("./data/images/mask.png", mask)

            mean_val = cv2.mean(image, mask)
            MEAN_COLORS = np.array((mean_val[0],mean_val[1],mean_val[2]), dtype=np.uint8)
            LAB_MEAN_COLORS = skimage.color.rgb2lab(MEAN_COLORS[np.newaxis,np.newaxis,:])
            deltaE = skimage.color.deltaE_cmc(LAB_MEAN_COLORS, LAB_COLORS)

            color = np.argmin(deltaE)
            if color == 0:
                contours_colors.append("red")
            elif color == 1:
                contours_colors.append("green")
            elif color == 2:
                contours_colors.append("blue")

        return contours_colors