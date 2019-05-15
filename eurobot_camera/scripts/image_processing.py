#!/usr/bin/env python
import cv2
import skimage
import numpy as np

from scipy.ndimage import label

def rotate_image(image, angle):
    (height, weight) = image.shape[:2]
    center = (weight / 2, height / 2)

    M = cv2.getRotationMatrix2D(center, angle, 1.0)
    rotated180 = cv2.warpAffine(image, M, (weight, height))
    return rotated180


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

def increase_saturation_3(image):
    hsvImg = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    hsvImg[:, :, 1] = cv2.multiply(hsvImg[:, :, 1], 2)
    hsvImg[:,:,1] = cv2.multiply(hsvImg[:,:,1], 2)
    background = cv2.cvtColor(hsvImg, cv2.COLOR_HSV2BGR)
    return background


def crop_image(img):
    print("Before crop", img.shape)
    cropped_img = img[724:1748, 200:2248]
    print("After crop", cropped_img.shape)
    background = cropped_img
    background = cv2.resize(cropped_img, (1024, 512), interpolation=cv2.INTER_LANCZOS4)
    print("After resize", background.shape)
    background = np.pad(background, ((256, 256), (0, 0), (0, 0)), mode="constant", constant_values=0)
    print("After padding", background.shape)
    # hsvImg = cv2.cvtColor(background, cv2.COLOR_BGR2HSV)
    # hsvImg[:, :, 1] = cv2.multiply(hsvImg[:, :, 1], 2)
    # hsvImg[:,:,1] = cv2.multiply(hsvImg[:,:,1], 2)
    # background = cv2.cvtColor(hsvImg, cv2.COLOR_HSV2BGR)
    return background

def crop_immage_1(img):
    cropped_img = img[200:2248, 200:2048]
    return cropped_img

def segment_on_dt(a, img):
    border = cv2.dilate(img, None, iterations=5)
    border = border - cv2.erode(border, None)

    dt = cv2.distanceTransform(img, 2, 3)
    dt = ((dt - dt.min()) / (dt.max() - dt.min()) * 255).astype(np.uint8)
    _, dt = cv2.threshold(dt, 180, 255, cv2.THRESH_BINARY)
    lbl, ncc = label(dt)
    lbl = lbl * (255/ncc)
    # Completing the markers now.
    lbl[border == 255] = 255

    lbl = lbl.astype(np.int32)
    cv2.watershed(a, lbl)

    lbl[lbl == -1] = 0
    lbl = lbl.astype(np.uint8)
    return 255 - lbl

def watersherd(img):
    # Pre-processing.
    img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    _, img_bin = cv2.threshold(img_gray, 0, 255,
                               cv2.THRESH_OTSU)
    img_bin = cv2.morphologyEx(img_bin, cv2.MORPH_OPEN,
                               np.ones((3, 3), dtype=int))

    result = segment_on_dt(img, img_bin)

    result[result != 255] = 0
    result = cv2.dilate(result, None)
    img[result == 255] = (0, 0, 255)

    img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    cv2.imwrite("./data/images/image_gray.png", img)

    return img
