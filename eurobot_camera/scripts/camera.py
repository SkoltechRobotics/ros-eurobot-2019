import cv2
import numpy as np
import skimage

import transform


# DIM=(2448, 2048)
# K=np.array([[677.6729654637016, 0.0, 1235.371481335309], [0.0, 679.4736576804626, 1043.9887347932538], [0.0, 0.0, 1.0]])
# D=np.array([[0.026055346132657364], [0.006035766757842894], [0.005666324884814231], [-0.0015661403498746557]])

TABLE_LENGTH_Y=2.0
TABLE_LENGTH_X=3.0
TABLE_LENGTH_Z=1.0

CAMERA_WIDTH=2448.
CAMERA_HEIGHT=2048.
HOMO_IMAGE_SCALE=2.
HOMO_IMAGE_WIDTH=CAMERA_WIDTH/HOMO_IMAGE_SCALE
HOMO_IMAGE_HEIGHT=CAMERA_HEIGHT/HOMO_IMAGE_SCALE

CAMERA_X = 1.5
CAMERA_Y = 0.0
CAMERA_Z = 1.0
CAMERA_ANGLE = (3 * np.pi / 4 - 0.1, 0, 0)

PUCK_RADIUS=0.038
pixel_scale_x=2448.0/3.0
pixel_scale_y=2048.0/2.0
PUCK_RADIUS_pixel_x=pixel_scale_x*PUCK_RADIUS
PUCK_RADIUS_pixel_y=pixel_scale_y*PUCK_RADIUS

MAX_FEATURES=10000
GOOD_MATCH_PERCENT=0.1

REDIUM_COLOR = [230, 0, 0]
GREENIUM_COLOR = [77, 255, 25]
BLUEIUM_COLOR = [25, 153, 255]

LAB_REDIUM = 0
LAB_GREENIUM = 0
LAB_BLUEIUM = 0

def find_rotation_matrix(theta):
    R_x = np.array([[1, 0, 0],
                    [0, np.cos(theta[0]),-np.sin(theta[0]) ],
                    [0, np.sin(theta[0]), np.cos(theta[0])]])
    R_y = np.array([[np.cos(theta[1]), 0, np.sin(theta[1])], 
                    [0, 1, 0],
                    [-np.sin(theta[1]), 0, np.cos(theta[1])]])
    R_z = np.array([[np.cos(theta[2]), -np.sin(theta[2]), 0],
                    [np.sin(theta[2]), np.cos(theta[2]), 0],
                    [0, 0, 1]])
    R = np.linalg.multi_dot([R_x, R_y, R_z])
    return R

def find_color_difference(color):
    print color.shape
    distances = []
    distances.append(skimage.color.deltaE_cmc(color, LAB_REDIUM,2,1))
    distances.append(skimage.color.deltaE_cmc(color, LAB_GREENIUM,2,1))
    distances.append(skimage.color.deltaE_cmc(color, LAB_BLUEIUM,2,1))
    return distances

class Camera():
    def __init__(self, DIM, K, D):
        self.DIM = DIM
        self.K_camera = K
        self.K_projection = K
        self.D = D
        self.is_aligned = False
        self.warp_matrix = np.eye(3, 3, dtype=np.float32)
        
    def find_vertical_projection(self):
        rotation_matrix = find_rotation_matrix(CAMERA_ANGLE)
        camera_position = np.array([ [CAMERA_X],
                                     [CAMERA_Y],
                                     [CAMERA_Z] ])
        t = -np.dot(rotation_matrix, camera_position)
        M = np.concatenate((rotation_matrix, t), axis=1)
        L = np.array([ [TABLE_LENGTH_X/CAMERA_WIDTH, 0, 0],
                       [0, -TABLE_LENGTH_Y/CAMERA_HEIGHT, TABLE_LENGTH_Y],
                       [0, 0, 0],
                       [0, 0, 1] ])
        H = np.dot(M, L)
        
        K_new = np.linalg.inv(H)
        
        self.K_projection = K_new
        
        return K_new
        
        #     def horizontal_proj(self):
#         rotation_matrix = self.__eulerAnglesToRotationMatrix((3 * np.pi / 4 - 0.1, 0, 0))
#         camera_position = np.array([[1.5, 0.0, 1.0]]).T
#         t = -rotation_matrix.dot(camera_position)
#         print t
#         M = np.concatenate((rotation_matrix, t), axis=1)
#         L = np.array([[lx/hx, 0, 0], [0, 0, ly], [0, -lz/hy, lz], [0, 0, 1.0]])	
#         K_new = np.linalg.inv(M.dot(L))
        
#         return K_new

    def align_image(self, undistorted_image, templ_path):
        if not self.is_aligned:
            self.find_warp_matrix_feature(undistorted_image, templ_path)
            self.find_vertical_warp_projection(self.warp_matrix)
            self.is_aligned = True
        return self.is_aligned
    
    def filter_image(self, img):
        return cv2.bilateralFilter(img,9,75,75)
        
    def find_thresholds(self, image_gray):
        image = cv2.adaptiveThreshold(image_gray,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,cv2.THRESH_BINARY,11,2)
        return image
        
    def find_contours(self, image_gray):
        ret, thresh = cv2.threshold(image_gray, 127, 255, 0)
        image, contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        return contours
        
    def filter_contours(self, contours):
        for cnt in contours:
            #epsilon = 0.1*cv2.arcLength(cnt,True)
            #approx = cv2.approxPolyDP(cnt,epsilon,True)
            cnt = cv2.convexHull(cnt)
        cnts = []
        for cnt in contours:
            area = cv2.contourArea(cnt)
            perimeter = cv2.arcLength(cnt,True)
            if area <= np.pi*PUCK_RADIUS_pixel_x*PUCK_RADIUS_pixel_y+300 and area >= np.pi*PUCK_RADIUS_pixel_x*PUCK_RADIUS_pixel_y-300 and perimeter <= 2*np.pi*PUCK_RADIUS_pixel_x+50 and perimeter >= 2*np.pi*PUCK_RADIUS_pixel_x-50 and len(cnt)>=5:
                cnts.append(cnt)
        return cnts
        
    def draw_contours(self, image, contours):
        img = cv2.drawContours(image, contours, -1, (255,0,0), 3)
        return img
        
    def watersherd(self, image):
        gray = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
        ret, thresh = cv2.threshold(gray,0,255,cv2.THRESH_BINARY_INV+cv2.THRESH_OTSU)
        # noise removal
        kernel = np.ones((3,3),np.uint8)
        opening = cv2.morphologyEx(thresh,cv2.MORPH_OPEN,kernel, iterations = 2)
        # sure background area
        sure_bg = cv2.dilate(opening,kernel,iterations=3)
        # Finding sure foreground area
        dist_transform = cv2.distanceTransform(opening,cv2.DIST_L2,5)
        ret, sure_fg = cv2.threshold(dist_transform,0.7*dist_transform.max(),255,0)
        # Finding unknown region
        sure_fg = np.uint8(sure_fg)
        unknown = cv2.subtract(sure_bg,sure_fg)
        
        # Marker labelling
        ret, markers = cv2.connectedComponents(sure_fg)
        # Add one to all labels so that sure background is not 0, but 1
        markers = markers+1
        # Now, mark the region of unknown with zero
        markers[unknown==255] = 0
        
        markers = cv2.watershed(image,markers)
        image[markers == -1] = [255,0,0]
        cv2.imwrite("markers.jpg",markers)
        return image
        
    def undistort(self, img):
        undistorted_img = cv2.fisheye.undistortImage(distorted=img,
                                                     K=self.K_camera,
                                                     D=self.D,
                                                     Knew=self.K_projection)
        return undistorted_img
    
    def find_warp_matrix_not_feature(self, undistorted_img, templ_path):
        warp_matrix = transform.find_transform_ecc(undistorted_img,
                                                               HOMO_IMAGE_WIDTH,
                                                               HOMO_IMAGE_HEIGHT, templ_path)
        self.warp_matrix = warp_matrix
        print ("WARP MATRIX=", warp_matrix)
        return warp_matrix
    
    def find_warp_matrix_feature(self, undistorted_img, templ_path):
        warp_matrix = transform.find_transform_features(undistorted_img,HOMO_IMAGE_WIDTH,HOMO_IMAGE_HEIGHT,MAX_FEATURES,GOOD_MATCH_PERCENT, templ_path)
        self.warp_matrix = warp_matrix
        print ("WARP MATRIX=", warp_matrix)
        return warp_matrix
    
    
    def find_vertical_warp_projection(self, warp_matrix):
        print warp_matrix
        C = np.zeros((3,3))
        C[(0,1,2),(0,1,2)] = HOMO_IMAGE_SCALE, HOMO_IMAGE_SCALE, 1
        W = warp_matrix
        C_inv = np.linalg.inv(C)
        W_inv = np.linalg.inv(W)
        K1 = C_inv.dot(self.K_projection)
        K2 = W_inv.dot(K1)
        K3 = C.dot(K2)
        K_new = K3
        self.K_projection = K_new
        return K_new
    
    def find_pucks_coordinates(self, contours):
        coordinates = []
        
        for cnt in contours:
            M = cv2.moments(cnt)
            cx = M['m10']/M['m00']
            cy = M['m01']/M['m00']
#            Cx = cx/pixel_scale_x
#            Cy = 2-cy/pixel_scale_y
            coordinates.append((cx,cy))
            
        return coordinates
   
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

    def detect_contours_color(self, contours, img):
        COLORS = np.array((REDIUM_COLOR,GREENIUM_COLOR,BLUEIUM_COLOR), dtype=np.uint8)
	print ("COLORS", COLORS)
        LAB_COLORS = skimage.color.rgb2lab(COLORS[np.newaxis,:,:])
        print (LAB_COLORS)
        LAB_REDIUM = LAB_COLORS[0,0]
        LAB_GREENIUM = LAB_COLORS[0,1]
        LAB_BLUEIUM = LAB_COLORS[0,2]
        
        result = []
        
	avg = np.zeros(3,dtype=np.float32)
	i = 0
	img_gray = cv2.cvtColor(img,cv2.COLOR_RGB2GRAY)
        for cnt in contours:
	    print ("CONTOURS NUM", len(contours))
		
            mask = np.zeros(img_gray.shape, np.uint8)
	    print ("MASK SHAPE=", mask.shape)
            cv2.drawContours(mask,[cnt],0,255,-1)
	    pixelpoints = cv2.findNonZero(mask)
	    
	    mean_val = cv2.mean(img,mask = mask)
	    print ("MEAN_VAL_______", mean_val)
            for pixel in pixelpoints:
                avg += img[pixel[0,0],pixel[0,1]]

	    avg = np.divide(avg,len(pixelpoints))
	    print ("avg=", avg)

	    MEAN_COLORS = np.array((mean_val[2],mean_val[1],mean_val[0]), dtype=np.uint8)
       	    AVG_COLORS = np.array((avg[2],avg[1],avg[0]), dtype=np.uint8)
	    print("AVG_COLORS", AVG_COLORS)
	    print("MEAN_COLORS", MEAN_COLORS)

            LAB_MEAN_COLORS = skimage.color.rgb2lab(MEAN_COLORS[np.newaxis,np.newaxis,:])
            print ("LAB_MEAN_COLORS",LAB_MEAN_COLORS)
            LAB_AVG_COLORS = skimage.color.rgb2lab(AVG_COLORS[np.newaxis,np.newaxis,:])
            print ("LAB_AVG_COLORS",LAB_AVG_COLORS)
	    
	    deltaE = skimage.color.deltaE_cie76(LAB_MEAN_COLORS, LAB_COLORS)
            print ("(LAB_AVG_COLORS)RESULT=",skimage.color.deltaE_cie76(LAB_AVG_COLORS, LAB_COLORS))
            print ("(LAB_MEAN_COLORS)RESULT=",deltaE)
	    
	    color = np.argmin(deltaE)
	    if color == 0:
		result.append("red")
	    if color == 1:
		result.append("green")
	    if color == 2:
		result.append("blue")


        return result
            

