import cv2
import numpy as np
import os, sys
import re
import glob

CHECKERBOARD = (7,4)

#TERM_CRITERIA_EPS = precision
#TERM_CRITERIA_MAX_ITER = max number of iteration to find corners
criteria = (cv2.TERM_CRITERIA_EPS+cv2.TERM_CRITERIA_MAX_ITER, 100, 1e-6)
#CALIB_USE_INTRINSIC_GUESS cameraMatrix contains valid initial values of fx, fy, cx, cy that are optimized further.
#Otherwise, (cx, cy) is initially set to the image center ( imageSize is used), and focal distances are computed in a least-squares fashion.
#CALIB_RECOMPUTE_EXTRINSIC Extrinsic will be recomputed after each iteration of intrinsic optimization.
#CALIB_CHECK_COND The functions will check validity of condition number.
#CALIB_FIX_SKEW Skew coefficient (alpha) is set to zero and stay zero.
calibration_flags = cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC+cv2.fisheye.CALIB_CHECK_COND+cv2.fisheye.CALIB_FIX_SKEW
def calibration(images, config_file):
	objp = np.zeros((1, CHECKERBOARD[0]*CHECKERBOARD[1], 3), np.float32)
	objp[0,:,:2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)

	_img_shape = None
	objpoints = [] # 3d point in real world space
	imgpoints = [] # 2d points in image plane.

	for fname in images:
	    img = cv2.imread(fname)
	    if _img_shape == None:
		_img_shape = img.shape[:2]
	    else:
		assert _img_shape == img.shape[:2], "All images must share the same size."
	    gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
	    ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, None) # Find the chess board corners
	    if ret == True: # If found, add object points, image points (after refining them)
		objpoints.append(objp)
		cv2.cornerSubPix(gray,corners,(3,3),(-1,-1),criteria)
		imgpoints.append(corners)

	K = np.zeros((3, 3)) #camera matrix
	D = np.zeros((4, 1)) #distorion coefficents matrix
	rvecs = [np.zeros((1, 1, 3), dtype=np.float64) for i in range(len(objpoints))] #rotation matrix for each frame
	tvecs = [np.zeros((1, 1, 3), dtype=np.float64) for i in range(len(objpoints))] #transaltion matrix for each frame

	rms, K, D, rvecs, tvecs = cv2.fisheye.calibrate(objpoints,imgpoints,gray.shape[:2],K,D,
		                                        rvecs,tvecs,calibration_flags,criteria)

	config_file.write('image_width: 2448\r\n' +
		          'image_height: 2048\r\n' +
		          'camera_name: head_camera\r\n' +
		          'camera_matrix:\r\n' +
		          '  rows: 3\r\n' +
		          '  cols: 3\r\n' +
		          '  data: ' +  '[%s]' % ', '.join(( str(num) for num in K.flatten() ))   + '\r\n' +
		          'distortion_model: fish_eye\r\n' +
		          'distortion_coefficients:\r\n' +
		          '  rows: 1\r\n' +
		          '  cols: 5\r\n' +
		          '  data: ' +  '[%s]' % ', '.join(( str(num) for num in D.flatten() ))   + '\r\n' +
		          'rectification_matrix:\r\n' +
		          '  rows: 3\r\n' +
		          '  cols: 3\r\n' +
		          '  data: [1.000000, 0.000000, 0.000000, 0.000000, 1.000000, 0.000000, 0.000000, 0.000000, 1.000000]\r\n' +
		          'projection_matrix:\r\n' +
		          '  rows: 3\r\n' +
		          '  cols: 4\r\n' +
		          '  data: [503.124786, 0.000000, 1199.591668, 0.000000, 0.000000, 494.065338, 1043.002703, 0.000000, 0.000000, 0.000000, 1.000000, 0.000000]\r\n')

	config_file.close()

if __name__ == '__main__':
	images = None
	config_file = None
    
	img_path_reg = re.compile(r'img_path=*')
	conf_path_reg = re.compile(r'config_path=*')
	for elem in sys.argv[1:]:
		if img_path_reg.match(elem):
			images = glob.glob(elem[9:] + '*.png')
			if not (images):
				if config_file:
						try:
							os.remove(config_file.name)
						except OSError:
							print("Couldn't delete " + congif_file.name + " file")
				sys.exit('Incorrect argument img_path=. It is the directory of images. Also, images need to be png format')
		if conf_path_reg.match(elem):
			try:
				config_file = open(elem[12:],'w') #Create camera configfile
			except IOError as err:
				sys.exit('Incorrect argument config_path=')

	if not images:
		print('Using default path for calibration images')
		images = glob.glob('./calibration_images/*.png')

	if not config_file:
		print('Using default path for config file')
		config_file = open('../configs/calibration.yaml','w') #Create camera configfile
	
	calibration(images, config_file)

