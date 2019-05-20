# coding: utf-8

import numpy as np
import imageio
import sys
import time
import torchvision.transforms.functional as TF
from model import UNet


def find_pucks(image):
    # Prepare the network
    net = UNet(weights='/home/nuka-cola/catkin_ws/src/ros-eurobot-2019/eurobot_camera/scripts/weights/fedge_detector_nuc_unet.pth.tar')

    # Load image from arguments
    image = TF.to_tensor(image).float().unsqueeze_(0)
    start = time.time()
    mask = net.eval_predict(image)
    end = time.time()
    print("Time taken:", end-start)
    return mask.astype(np.uint8)
    # imageio.imwrite("output.png", mask.astype(np.uint8))
