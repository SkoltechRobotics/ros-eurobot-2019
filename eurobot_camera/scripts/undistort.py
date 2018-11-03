#!/usr/bin/env python


import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError
import roslib, rospy
import sensor_msgs
from sensor_msgs.msg import Image
import sys, re
import yaml

# DIM=(2448, 2048)
# K=np.array([[677.6729654637016, 0.0, 1235.371481335309], [0.0, 679.4736576804626, 1043.9887347932538], [0.0, 0.0, 1.0]])
# D=np.array([[0.026055346132657364], [0.006035766757842894], [0.005666324884814231], [-0.0015661403498746557]])


def read_config(conf_file):
    data_loaded = yaml.load(conf_file)
    
    DIM = (data_loaded['image_width'], data_loaded['image_height'])
    print DIM
    camera_matrix = data_loaded['camera_matrix']
    data = camera_matrix['data']
    rows = camera_matrix['rows']
    cols = camera_matrix['cols']

    _K = []
    for i in range(rows):
        _K.append(data[i*rows:i*rows+cols:1])
    print _K

    K = np.array(_K)

    distortion_coefficients = data_loaded['distortion_coefficients']
    data = distortion_coefficients['data']
    rows = distortion_coefficients['rows']
    cols = distortion_coefficients['cols']

    _D = []
    for i in range(rows):
        _D.append(data[i*rows:i*rows+cols:1])
    print _D

    D=np.array(_D)
    return DIM, K, D


class undistortion():
    def __init__(self, balance = 0.0, dim2=None, dim3=None):
        self.node = rospy.init_node('undistort_node', anonymous=True)
        self.publisher = image_pub = rospy.Publisher("undistort_image",Image)
        self.subscriber = rospy.Subscriber("/usb_cam/image_raw", sensor_msgs.msg.Image, callback)
        self.bridge = CvBridge()
        self.balance = balance
        self.dim2 = dim2 # This is the dimension of the box you want to keep after un-distorting the image.
                         # When balance = 0,OpenCV will keep the best part of the image for you. 
 			 #  Whereas balance = 1 tells OpenCV to keep every single pixel of the original image, which means a lot of black filling areas and overstretched corners.
        self.dim3 = dim3 # Dimension of the final box where OpenCV will put the undistorted image. 
#                        # It can be any size and any aspect ratio.
#                        # But most of the times you probably want to make it the same as dim2 unless you want to do some cropping to the un-distorted image.
        self.new_K, self.map1, self.map2 = __calculation()
        
    def __calculation(self):
        dim1 = img.shape[:2][::-1]  # dim1 is the dimension of input image to un-distort
        assert dim1[0]/dim1[1] == DIM[0]/DIM[1], "Image to undistort needs to have same aspect ratio as the ones used in calibration"
        if not dim2:
            dim2 = dim1
        if not dim3:
            dim3 = dim1
        scaled_K = K * dim1[0] / DIM[0]  # The values of K is to scale with image dimension.
        scaled_K[2][2] = 1.0  # Except that K[2][2] is always 1.0
        # This is how scaled_K, dim2 and balance are used to determine the final K used to un-distort image. OpenCV document failed to make this clear!
        new_K = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(scaled_K, D, dim2, np.eye(3), balance=balance)
        map1, map2 = cv2.fisheye.initUndistortRectifyMap(scaled_K, D, np.eye(3), new_K, dim3, cv2.CV_16SC2)
        return new_K, map1, map2
        
    def undistort(self, img):
        undistorted_img = cv2.remap(img, self.map1, self.map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
        return undistorted_img
    
    #FIXME::Undistort without map, only use cv2.undistortImage()
    def undistort2(self, img):
        pass

    def __callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            rospy.loginfo(rospy.get_caller_id())
        except CvBridgeError as e:
            print(e)
        image = undistort(cv_image)
        try:
            self.publisher.publish(self.bridge.cv2_to_imgmsg(image, "bgr8"))
        except CvBridgeError as e:
            print(e)


if __name__ == '__main__':
    conf_path_reg = re.compile(r'config_path=*')
    if sys.argv[1:]:
        if conf_path_reg.match(sys.argv[0]):
            try:
                conf_file = open(sys.argv[1:][12:],'r')
            except IOError as err:
                sys.exit('Incorrect argument config_path=')
    else:
        print('Using default config file path')
        try:
            conf_file = open('../configs/calibration.yaml','r')
        except IOError as err:
            sys.exit("Couldn't find config file")
    read_config(conf_file)

    try:
        undistortion = undistortion()
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()


