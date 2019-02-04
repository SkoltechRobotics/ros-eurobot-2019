#!/usr/bin/env python
import cv2
import skimage
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
PUCK_RADIUS_pixel_x = pixel_scale_x*PUCK_RADIUS
PUCK_RADIUS_pixel_y = pixel_scale_y*PUCK_RADIUS

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
        ret, th = cv2.threshold(image_gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        image, contours, hierarchy = cv2.findContours(th, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

        self.hierarchy = hierarchy
        self.all_contours = contours
        self.filtered_contours = []

    def filter(self, area_epsilon, perimeter_epsilon, radius_epsilon):
        for cnt in self.all_contours:
            perimeter = cv2.arcLength(cnt, True)
            area = np.fabs(cv2.contourArea(cnt))
            if area <= np.pi *PUCK_RADIUS_pixel_x *PUCK_RADIUS_pixel_y +area_epsilon and \
                    area >= np.pi *PUCK_RADIUS_pixel_x *PUCK_RADIUS_pixel_y -area_epsilon and \
                    perimeter <= 2* np.pi * PUCK_RADIUS_pixel_x + perimeter_epsilon and \
                    perimeter >= 2 * np.pi * PUCK_RADIUS_pixel_x - perimeter_epsilon and \
                    self.is_contour_circle(cnt, radius_epsilon) and \
                    len(cnt) > 15:
                self.filtered_contours.append(cnt)
        return self.filtered_contours

    def draw(self, image, contours):
        return cv2.drawContours(image, contours, -1, (255, 0, 0), 3)

    def draw_ellipse(self, image, contours, coordinates, colors):
        for cnt, coord, color in zip(contours, coordinates, colors):
            ellipse = cv2.fitEllipse(cnt)
            image = cv2.ellipse(image,ellipse,(0,255,0),2)
            cx = coord[0]
            cy = coord[1]
            Cx = cx/pixel_scale_x
            Cy = 2-cy/pixel_scale_y
            image = cv2.putText(img=image,
                                text="Cx=%.2f,Cy=%.2f,%s" % (Cx, Cy, color),
                                org=(int(cx),int(cy)),
                                fontFace=cv2.FONT_HERSHEY_SIMPLEX,
                                fontScale=2,
                                color=(255,255,255),
                                thickness=5,
                                lineType=5)
        return image

    def find_pucks_coordinates(self):
        coordinates = []
        for cnt in self.filtered_contours:
            M = cv2.moments(cnt)
            cx = M['m10']/M['m00']
            cy = M['m01']/M['m00']
            coordinates.append((cx,cy))
        return coordinates

    def is_contour_circle(self, contour, epsilon):
        M = cv2.moments(contour)
        if np.abs(M["mu20"]/M["m00"]-M["mu02"]/M["m00"]) <= epsilon:
            return True
        else:
            return False

    def detect_color(self, contours, image):
        image = image_processing.increase_saturation(image)

        contours_colors = []

        for cnt in contours:
            mask = np.zeros(image.shape[:2], np.uint8)
            cv2.drawContours(mask,[cnt],0,255,-1)
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