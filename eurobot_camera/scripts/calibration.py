import cv2
import numpy as np
import sys
import re
import glob

CHECKERBOARD = (7,4)

# TERM_CRITERIA_EPS = precision
# TERM_CRITERIA_MAX_ITER = max number of iteration to find corners
criteria = (cv2.TERM_CRITERIA_EPS+cv2.TERM_CRITERIA_MAX_ITER, 100, 1e-6)
# CALIB_USE_INTRINSIC_GUESS cameraMatrix contains valid initial values of fx, fy, cx, cy that are optimized further.
# Otherwise, (cx, cy) is initially set to the image center ( imageSize is used), and focal distances are computed in a least-squares fashion.
# CALIB_RECOMPUTE_EXTRINSIC Extrinsic will be recomputed after each iteration of intrinsic optimization.
# CALIB_CHECK_COND The functions will check validity of condition number.
# CALIB_FIX_SKEW Skew coefficient (alpha) is set to zero and stay zero.
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

	K = np.zeros((3, 3)) # camera matrix
	D = np.zeros((4, 1)) # distorion coefficents matrix
	rvecs = [np.zeros((1, 1, 3), dtype=np.float64) for i in range(len(objpoints))] # rotation matrix for each frame
	tvecs = [np.zeros((1, 1, 3), dtype=np.float64) for i in range(len(objpoints))] # transaltion matrix for each frame

	rms, K, D, rvecs, tvecs = cv2.fisheye.calibrate(objpoints,imgpoints,gray.shape[:2],K,D,
		                                        rvecs,tvecs,calibration_flags,criteria)

    # FIXME::Projection matrix was given from old config file. Need to calculate it
	config_file.write('image_width: '+ str(gray.shape[1]) +'\n' +
		          'image_height: '+ str(gray.shape[0]) +'\n' +
		          'camera_name: head_camera\n' +
		          'camera_matrix:\n' +
		          '  rows:  '+ str(K.shape[1]) +'\n' +
		          '  cols:  '+ str(K.shape[0]) +'\n' +
		          '  data: ' +  '[%s]' % ', '.join(( str(num) for num in K.flatten() ))   + '\n' +
		          'distortion_model: fish_eye\n' +
		          'distortion_coefficients:\n' +
		          '  rows: '+ str(D.shape[1]) +'\n' +
		          '  cols: '+ str(D.shape[0]+1) +'\n' +
		          '  data: ' +  '[%s, 0.000000]' % ', '.join(( str(num) for num in D.flatten() ))   + '\n' +
		          'rectification_matrix:\n' +
		          '  rows: 3\n' +
		          '  cols: 3\n' +
		          '  data: [1.000000, 0.000000, 0.000000, 0.000000, 1.000000, 0.000000, 0.000000, 0.000000, 1.000000]\n' +
		          'projection_matrix:\n' +
		          '  rows: 3\n' +
		          '  cols: 4\n' +
		          '  data: [503.124786, 0.000000, 1199.591668, 0.000000, 0.000000, 494.065338, 1043.002703, 0.000000, 0.000000, 0.000000, 1.000000, 0.000000]\n')

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
				if config_file: # If config file was created before need to delete
						try:
							os.remove(config_file.name)
						except OSError:
							print("Couldn't delete " + congif_file.name + " file")
				sys.exit('Incorrect argument img_path=. It is the directory of images. Also, images need to be png format')
		if conf_path_reg.match(elem):
			try:
				config_file = open(elem[12:],'w') # Create camera configfile
			except IOError as err:
				sys.exit('Incorrect argument config_path=')

	if not images:
		print('Using default path for calibration images')
		images = glob.glob('./calibration_images/*.png')
        if not images:
            sys.exit("Couldn't find images")
        

	if not config_file:
		print('Using default path for config file')
		try:
            		config_file = open('../configs/calibration.yaml','w') # Create camera configfile
        	except IOError as err:
            		print (err)
	
	calibration(images, config_file)

