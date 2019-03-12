#!/usr/bin/env python
import cv2
import skimage

def equalize_histogram(image, clipLimit=1.0, tileGridSize=(21, 21)):
    clahe = cv2.createCLAHE(clipLimit=clipLimit, tileGridSize=tileGridSize)
    channels = cv2.split(image)

    eq_channels = []
    for ch, color in zip(channels, ['B', 'G', 'R']):
        eq_channels.append(clahe.apply(ch))

    eq_image = cv2.merge(eq_channels)

    return eq_image

def decrease_noise(image, d=15, sigmaColor=100, sigmaSpace=100):
    filtered_image = cv2.bilateralFilter(image,d,sigmaColor,sigmaSpace)
    return filtered_image

def increase_saturation(image):
    hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    h, s, v = cv2.split(hsv_image)

    s_cutoff, v_cutoff = 0.1, 0.5
    s_gain, v_gain = 20, 10

    s = skimage.exposure.adjust_sigmoid(s, s_cutoff, s_gain)
    v = skimage.exposure.adjust_sigmoid(v, v_cutoff, v_gain)

    hsv_image[:, :, 1] = s
    hsv_image[:, :, 2] = v

    bgr_image = cv2.cvtColor(hsv_image, cv2.COLOR_HSV2BGR)
    return bgr_image

