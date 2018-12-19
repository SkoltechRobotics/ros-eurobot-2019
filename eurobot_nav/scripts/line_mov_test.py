#!/usr/bin/env python
# coding: utf-8

# simple go to goal in different methods: odom movement

import rospy
import numpy as np
import tf2_ros
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from threading import Lock
from std_msgs.msg import Int32MultiArray


class MotionPlanner:
    def __init__(self):
        rospy.init_node("motion_planner", anonymous=True)

        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer)

        #FIXME
        self.robot_name="secondary_robot"
        
        self.vel = np.zeros(3)

        self.mutex = Lock()

        self.cmd_id = None
        self.cmd_type = None
        self.cmd_args = None
        self.t_prev = None
        self.goal = None
        self.mode = None
        self.RATE = 10
        self.dt = 0.01
        
        self.V_MAX = 0.15 # m/s
        self.V_MIN = 0.05
        self.W_MAX = 0.3
        
        self.Kp_rho = 0.5
        self.Kp = 30
        
        #w = 1.5
        self.k = 0.8
        self.Kv = 1
        self.THRESHOLD_XY = 0.1
        self.THRESHOLD_YAW = 0.1
        
        # calculated distance and rot to goal from initial point, stays unchanged
        self.d_init = 0
        self.alpha_init = 0
        
        self.cmd_stop_robot_id = None
        self.stop_id = 0

        self.pub_response = rospy.Publisher("response", String, queue_size=10)
        self.pub_cmd = rospy.Publisher("/secondary_robot/stm_command", String, queue_size=1)
        rospy.Subscriber("move_command", String, self.cmd_callback, queue_size=1)

        # start the main timer that will follow given goal points
        #rospy.Timer(rospy.Duration(1.0 / self.RATE), self.arc_move)
        rospy.Timer(rospy.Duration(1.0 / self.RATE), self.line_move)

        # self.timer.shutdown()

    def cmd_callback(self, data):
        self.mutex.acquire()
        rospy.loginfo("========================================")
        rospy.loginfo("NEW CMD:\t" + str(data.data))

        # parse name, type
        data_splitted = data.data.split()
        self.cmd_id = data_splitted[0] # unique identificator, string type, for ex. abc
        self.cmd_type = data_splitted[1]
        self.cmd_args = data_splitted[2:]
        rospy.loginfo(self.cmd_args)
        
        # need to calculate initial distance to goal once and then use it to calculate setpoint velocity
        # -------------------------
        while not self.update_coords():
            rospy.sleep(0.05)
            
        x = self.coords[0]
        y = self.coords[1]
        theta = self.coords[2]
	#print ("cmd_callback got coords", x, y, theta)
        
        x_goal = float(self.cmd_args[0])
        y_goal = float(self.cmd_args[1])
        theta_goal = float(self.cmd_args[2])
	#rospy.loginfo('callback goal (%.4f %.4f %.4f)', x_goal, y_goal, theta_goal)
        
        self.d_init = np.sqrt((x_goal - x)**2 + (y_goal - y)**2)
        self.alpha_init = self.wrap_angle(theta_goal - theta) # theta_diff
	#rospy.loginfo('callback d_init %.4f', self.d_init)	
	#print ("calculated init dist", self.d_init, self.alpha_init)
    
	# kill timer
	# send responce
        # ---------------------------
        # ---------------------------
        
        """
        if cmd_type == "arc_move":  # arc-movement by odometry
            inp = np.array(cmd_args).astype('float')
            inp[2] %= 2 * np.pi
            self.arc_move(cmd_id, *inp)

            
        elif cmd_type == "stop":
            self.cmd_id = cmd_id
            self.mode = cmd_type 
            #self.terminate_following()
            rospy.loginfo("Setting robot speed to zero.")
            self.stop_robot()
            rospy.loginfo("Robot has stopped.")

        self.mutex.release()        
        """
    @staticmethod    
    def vel(path_done, path_left, V_MIN, V_MAX, k, Kp):
	rospy.loginfo('VEL FUNC')
        if path_done < path_left:
	    #rospy.loginfo('path done', path_done)
            v = min(V_MAX * np.e**(-1 / (path_done / k + 0.1)) + V_MIN, V_MAX) 
            
	    
            # use linear ACC
            #v = min(V_MAX, Kp*path_done + V_MIN)
	    rospy.loginfo('acc vel %.4f', v)
        else:
            # expo DCL
            v = min(V_MAX * np.e**(-1 / (path_left / k + 0.1)) + V_MIN, V_MAX)
	    rospy.loginfo('dcl vel %.4f', v)
        return v
    
    
    @staticmethod
    def distance(coords1, coords2):
        ans = coords2 - coords1
        if abs(coords1[2] - coords2[2]) > np.pi:
            if coords2[2] > coords1[2]:
                ans[2] -= 2 * np.pi
            else:
                ans[2] += 2 * np.pi
        return ans
    
    @staticmethod
    def rotation_transform(vec, angle):
        M = np.array([[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]])
        ans = vec.copy()
        ans[:2] = np.matmul(M, ans[:2].reshape((2, 1))).reshape(2)
        return ans
    
    
    
    def wrap_angle(self,angle):
        """
        Wraps the given angle to the range [-pi, +pi].

        :param angle: The angle (in rad) to wrap (can be unbounded).
        :return: The wrapped angle (guaranteed to in [-pi, +pi]).
        """
	return (angle + np.pi) % (np.pi * 2) - np.pi
    

    def stop_robot(self):
        self.cmd_stop_robot_id = "stop_" + self.robot_name + str(self.stop_id)
        self.stop_id += 1
        self.robot_stopped = False
        cmd = "8 0 0 0"
        rospy.loginfo("Sending cmd: " + cmd)
        self.pub_cmd.publish(cmd)
        for i in range(20):
            if self.robot_stopped:
                self.cmd_stop_robot_id = None
                rospy.loginfo("Robot stopped.")
                return
            rospy.sleep(1.0 / 40)
        rospy.loginfo("Have been waiting for response for .5 sec. Stopped waiting.")
        self.cmd_stop_robot_id = None
        

    def set_speed(self, vel):
        vx, vy, wz = vel
        tw = Twist()
        tw.linear.x = vx
        tw.linear.y = vy
        tw.angular.z = wz
        self.pub_twist.publish(tw)
        self.vel = vel
        
        
    def line_move(self, vel = 0.3):
        rospy.loginfo(' ------- START LINE MOVE ------- ')
        
        try:
            x_goal = float(self.cmd_args[0])
            y_goal = float(self.cmd_args[1])
            theta_goal = float(self.cmd_args[2])
            while not self.update_coords():
                rospy.sleep(0.05)

            x = self.coords[0]
            y = self.coords[1]
            theta = self.coords[2]

            x_diff = x_goal- x
            y_diff = y_goal - y

            gamma = np.arctan2(y_diff, x_diff)
            print("gamma=", gamma)             
                # vector norm, simply saying: an amplitude of a vector
            d = np.sqrt(x_diff**2 + y_diff**2) # rho, np.linalg.norm(d_map_frame[:2])
            alpha = self.wrap_angle(theta_goal - theta) # theta_di
            w = 0
            v = 0
            rospy.loginfo('alpha wrapped %.4f', alpha)
            #rospy.loginfo(' %.4f', )
            while alpha > self.THRESHOLD_YAW:
                w = self.rotate_odom(alpha)
                v = 0
            rospy.loginfo('------------ ROTATING FINISHED -------------  %.4f', alpha)
            self.stop_robot() 

            while d > self.THRESHOLD_XY:
                v = self.translate_odom(d)
                w = 0
            rospy.loginfo('------------ TRANSLATION FINISHED -------------  %.4f', d)
            self.stop_robot() 

            if d < self.THRESHOLD_XY and alpha < self.THRESHOLD_YAW:
                rospy.loginfo(' ------- THRESHOLD REACHED -------- ')
                self.stop_robot() 

        except:
            pass
        
        
    def rotate_odom(self, alpha):
        rospy.loginfo("-------NEW ROTATIONAL MOVEMENT-------")
        
        while not self.update_coords():
            rospy.sleep(0.05)
        
        vx = v * np.cos(gamma)
        vy = v * np.sin(gamma)
        w = 0
        
        v_cmd = np.array([vx, vy, w])
        rospy.loginfo("v_cmd:\t" + str(v_cmd))
        cmd = " 8 " + str(v_cmd[0]) + " " + str(v_cmd[1]) + " " + str(v_cmd[2])
        rospy.loginfo("Sending cmd: " + cmd)
        self.pub_cmd.publish(cmd)

        
    def translate_odom(self, d):
        
        rospy.loginfo("-------NEW ROTATIONAL MOVEMENT-------")
        
        while not self.update_coords():
            rospy.sleep(0.05)
        
        rospy.loginfo("v_cmd:\t" + str(v_cmd))
        cmd = " 8 " + 0 + " " + 0 + " " + str(alpha)
        rospy.loginfo("Sending cmd: " + cmd)
        self.pub_cmd.publish(cmd)
        
        
        
#     def line_move(self, vel = 0.3):
#         #print ("START LINE MOVE")
# 	try:
# 		x_goal = float(self.cmd_args[0])
#         	y_goal = float(self.cmd_args[1])
#         	theta_goal = float(self.cmd_args[2])
#         	#print("000")
#         	while not self.update_coords():
#         		rospy.sleep(0.05)

# 	        x = self.coords[0]
# 		y = self.coords[1]
# 		theta = self.coords[2]

# 		x_diff = x_goal- x
# 		y_diff = y_goal - y
        
# 		gamma = np.arctan2(y_diff, x_diff)
# 		print("gamma=", gamma)             
#             # vector norm, simply saying: an amplitude of a vector
#  		d = np.sqrt(x_diff**2 + y_diff**2) # rho, np.linalg.norm(d_map_frame[:2])
#  		alpha = self.wrap_angle(theta_goal - theta) # theta_di
# 		w = 0
# 		v = 0
# 		print("alpha=", alpha)     
# 		if alpha > self.THRESHOLD_YAW:
# 			w = alpha/5 + 0.02
# 		else:
# 			if d > self.THRESHOLD_XY:
# 				w = 0
# 				v = d/5 + 0.02
# 			else:
# 				w = 0
# 				v = 0
#           			self.stop_robot() 
# 		vx = v * np.cos(gamma)
# 		vy = v * np.sin(gamma)
# 		print("vx, vy", vx, vy)
# 		v_cmd = np.array([vx, vy, w])
# 		rospy.loginfo("v_cmd:\t" + str(v_cmd))
# 		cmd = " 8 " + str(v_cmd[0]) + " " + str(v_cmd[1]) + " " + str(v_cmd[2])
# 		rospy.loginfo("Sending cmd: " + cmd)
# 		self.pub_cmd.publish(cmd)
#     	except:
# 	    pass

#     def arc_move(self, vel=0.3):
#         "go to goal in one movement by arc path"
#         "t chosen to be 0.1 for testing, meaning 10 Hz"
#         "positioning args vel and wmax are not used"
        
#         # one array is better than 3 float
#         # self.goal initilize in cmd_callback

#         try:
#             x_goal = float(self.cmd_args[0])
#             y_goal = float(self.cmd_args[1])
#             theta_goal = float(self.cmd_args[2])
            
#             goal = np.array([x_goal, y_goal, theta_goal])
#             rospy.loginfo("-------NEW ARC MOVEMENT-------")
#             rospy.loginfo("Goal:\t" + str(goal))
            
#             while not self.update_coords():
#                 rospy.sleep(0.05)
#             #rospy.loginfo('!-----------NEXT STEp--------------!') 
#             x = self.coords[0]
#             y = self.coords[1]
#             theta = self.coords[2]
            
#             x_diff = x_goal- x
#             y_diff = y_goal - y

#             gamma = np.arctan2(y_diff, x_diff)
            
#             # vector norm, simply saying: an amplitude of a vector
#             d = np.sqrt(x_diff**2 + y_diff**2) # rho, np.linalg.norm(d_map_frame[:2])
#             alpha = self.wrap_angle(theta_goal - theta) # theta_diff
            
# # create func for this 

#             path_done = np.sqrt(self.d_init**2 + self.alpha_init**2) - np.sqrt(d**2 + alpha**2)
#             path_left = np.sqrt(d**2 + alpha**2)

# 	    rospy.loginfo('d_init is %.4f', self.d_init)
# 	    rospy.loginfo('alpha_init is %.4f', self.alpha_init)
# 	    rospy.loginfo("d_norm left is %.4f", d)
# 	    rospy.loginfo('alpha left is %.4f', alpha)
# 	    rospy.loginfo('path_done %.4f', path_done)
# 	    rospy.loginfo('path_left %.4f', path_left)

#             beta = self.wrap_angle(gamma - alpha/2)
#             #rospy.loginfo('NEXT FUNC?') 
#             v = self.Kv * MotionPlanner.vel(path_done, path_left, self.V_MIN, self.V_MAX, self.k, self.Kp)
# 	    #print ("calculated velocity is", v)
#             rospy.loginfo('SPEED %.4f', v)
#             if d < self.THRESHOLD_XY and alpha < self.THRESHOLD_YAW:
#                 v = 0
#                 w = 0
#                 self.stop_robot()
# 		rospy.loginfo('THRESHHOLDS REACHED')
#             elif abs(d) < 1e-4:
#                 v = 0
#                 w = self.W_MAX # change ******************
# 		rospy.loginfo('trans = 0, w = W_MAX')
#             elif abs(alpha) < 1e-4:
#                 w = 0
# 		rospy.loginfo('alpha close to zero, w = 0 and v is %.4f', v)
#             else:
#                 R = 0.5 * d / np.sin(alpha/2)
#                 w = v / R  # must be depended on v such way so path becomes an arc
# 	        rospy.loginfo('regular speed calc v/R')
	    
#             vx = v * np.cos(beta)
#             vy = v * np.sin(beta)

# 	    #print("result velocity is", vx, vy, w)
#             rospy.loginfo('!--------------------SEND SPEED---------')
#             v_cmd = np.array([vx, vy, w])
# 	    rospy.loginfo("v_cmd:\t" + str(v_cmd))
#             cmd = " 8 " + str(v_cmd[0]) + " " + str(v_cmd[1]) + " " + str(v_cmd[2])
#             rospy.loginfo("Sending cmd: " + cmd)
#             self.pub_cmd.publish(cmd)
            
#         except:
#             pass
        

    def update_coords(self):
        try:
            trans = self.tfBuffer.lookup_transform('map', self.robot_name, rospy.Time())
            q = [trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w]
            angle = euler_from_quaternion(q)[2] % (2 * np.pi)
            #self.coords = np.array( (0,0,0) )
            self.coords = np.array([trans.transform.translation.x, trans.transform.translation.y, angle])
            rospy.loginfo("Robot coords:\t" + str(self.coords))
            return True
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.loginfo("MotionPlanner failed to lookup tf2.")
            return False


if __name__ == "__main__":
    
    planner = MotionPlanner()
    rospy.spin()
 

