#!/usr/bin/env python
import numpy as np

from threading import Lock

import rospy
import tf2_ros
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Int32MultiArray

import yaml

RATE=20
Kp_rho=0.5
XY_GOAL_TOLERANCE=0.02
YAW_GOAL_TOLERANCE=0.1

def parse_args(data):
    data_splitted = data.split()
    cmd_id = data_splitted[0] # unique identificator, string type, for ex. abc
    cmd_type = data_splitted[1]
    cmd_args = data_splitted[2:]
    if cmd_type =="arc_move":
        cmd_args = np.array(cmd_args).astype('float')
        cmd_args[2] %= 2 * np.pi
    return (cmd_id, cmd_type, cmd_args)

class NavigationNode():
    def __init__(self):
        rospy.init_node("motion_planner", anonymous=True)
        self.pub_response = rospy.Publisher("response", String, queue_size=10)
        self.pub_cmd = rospy.Publisher("/secondary_robot/stm_command", String, queue_size=1)
        rospy.Subscriber("move_command", String, self.cmd_callback, queue_size=1)
        
        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer)
        
        self.mutex = Lock()
        
        self.robot_name="secondary_robot"
        #self.robot_name=rospy.get_param("robot_name")
        
        self.motion_planner = MotionPlanner()
        
        self.RATE = RATE
        # start the main timer that will follow given goal points
        rospy.Timer(rospy.Duration(1.0 / self.RATE), self.motion_planner.plan)
        

    def cmd_callback(self, data):
        self.mutex.acquire()
        rospy.loginfo("================================================")
        rospy.loginfo("NEW CMD:\t" + str(data.data))
        
        self.motion_planner.cmd_id, self.motion_planner.cmd_type, self.motion_planner.cmd_args = parse_args(data.data)
        #print (self.cmd_id)
	#print (self.cmd_type)
	print ('!-----------JOPA----------------------------!')
	print (self.motion_planner.cmd_args)
	self.motion_planner.goal=self.motion_planner.cmd_args
        if self.motion_planner.cmd_type == "arc_move":  # arc-movement by odometry
            self.motion_planner.arc_move()
        elif self.motion_planner.cmd_type == "stop":
            self.motion_planner.stop_robot()
        elif self.motion_planner.cmd_type == "line_move":
            self.motion_planner.line_move(self.motion_planner.cmd_args)

        self.mutex.release()        
        
class Calculator():
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
    
    
    @staticmethod
    def wrap_angle(angle):
        return (angle + np.pi) % (2 * np.pi) - np.pi
    

#     @staticmethod
#     def wrap_angle(self,angle):
#         pi2 = 2 * np.pi
#         while angle < -np.pi:
#             angle += pi2
#         while angle >= np.pi:
#             angle -= pi2
#         return angle
    
        
class MotionPlanner:
    def __init__(self):
        self.speeds = np.zeros(3) 
        
        self.coords=np.array([0,0,0])
        
        self.pub_cmd = rospy.Publisher("/secondary_robot/stm_command", String, queue_size=1)


        self.cmd_id = None
        self.cmd_type = None
        self.cmd_args = None
        self.goal = None
        self.mode = None
        self.RATE = RATE
        self.Kp_rho = Kp_rho
        self.XY_GOAL_TOLERANCE = XY_GOAL_TOLERANCE
        self.YAW_GOAL_TOLERANCE = YAW_GOAL_TOLERANCE
        self.cmd_stop_robot_id = None
        self.stop_id = 0
        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer)
 
	
 
        self.mutex = Lock()


    def plan(self, event):
        self.mutex.acquire()
    	print self.coords
	print('!----------------------------------!')
	print self.goal
	print('!-----------GOAL-------------------!')
        if self.cmd_id is None:
            self.mutex.release()
            return

        rospy.loginfo("-------NEW MOTION PLANNING ITERATION-------")

        if not self.update_coords():
            self.terminate_following()
            rospy.loginfo('Stopping because of tf2 lookup failure.')
            self.mutex.release()
            return

        # current linear and angular goal distance
        goal_distance = np.zeros(3)
        goal_distance = Calculator.distance(self.coords, self.goal)
        rospy.loginfo('Goal distance:\t' + str(goal_distance))
        #FIXME
        goal_d = np.linalg.norm(goal_distance[:2])
        rospy.loginfo('Goal d:\t' + str(goal_d))

        # stop and publish response if we have reached the goal with the given tolerance
        if ((self.mode == 'line_move') and
            goal_d <= self.XY_GOAL_TOLERANCE and
            goal_distance[2] <= self.YAW_GOAL_TOLERANCE):
            rospy.loginfo(self.cmd_id + " finished, reached the goal")
            self.terminate_following()
            self.mutex.release()
            return
        
        elif self.mode == 'line_move':
              self.line_move()
        
	
        self.mutex.release()
    
    #?!?!?!?!
    def terminate_following(self):
        self.stop_robot()
        rospy.loginfo("Robot has stopped.")

        self.cmd_id = None
        self.goal = None
        self.mode = None

    def stop_robot(self):
        rospy.loginfo("Setting robot speed to zero.")
        self.robot_stopped = False
        cmd = "8 0 0 0"
        rospy.loginfo("Sending cmd: " + cmd)
        self.pub_cmd.publish(cmd)
        
        #FIXME::CHECK STOPPED OR NOT
        
        rospy.loginfo("Sending cmd: " + cmd)
        
    #FIXME:CAHNGE SIGNATURE
    def arc_move(self, vel=0.3, wmax=1.5, t = 0.1):
        "go to goal in one movement by arc path"
        "t chosen to be 0.1 for testing, meaning 10 Hz"
        "positioning args vel and wmax are not used"
    
        x_goal = float(self.cmd_args[0])
        y_goal = float(self.cmd_args[1])
        theta_goal = float(self.cmd_args[2])
        
        goal = np.array([x_goal, y_goal, theta_goal])
        rospy.loginfo("-------NEW ARC MOVEMENT-------")
        rospy.loginfo("Goal:\t" + str(goal))

        while not self.update_coords():
            rospy.sleep(0.05)

        x = self.coords[0]
        y = self.coords[1]
        theta = self.coords[2]

        x_diff = x_goal- x
        y_diff = y_goal - y

        gamma = np.arctan2(y_diff, x_diff)
        d = np.sqrt(x_diff**2 + y_diff**2) # rho, np.linalg.norm(d_map_frame[:2])

        """
        Restrict alpha and beta (angle differences) to the range
        [-pi, pi] to prevent unstable behavior e.g. difference going
        from 0 rad to 2*pi rad with slight turn
        """
        alpha = self.wrap_angle(theta_goal - theta) # theta_diff
        
#         if d < 0.05:
#             self.stop_robot()
#         else:

        v = self.Kp_rho * d

        if alpha == 0:
            w = 0
        else:
            R = 0.5 * d / np.sin(alpha/2)
            w = v / R  # must be depended on v such way so path becomes an arc

        beta = self.wrap_angle(gamma - alpha/2)

        vx = v * np.cos(beta)
        vy = v * np.sin(beta)

        v_cmd = np.array([vx, vy, w])        
        rospy.loginfo("v_cmd:\t" + str(v_cmd))
        cmd = " 8 " + str(v_cmd[0]) + " " + str(v_cmd[1]) + " " + str(v_cmd[2])
        rospy.loginfo("Sending cmd: " + cmd)
        self.pub_cmd.publish(cmd)

    def line_move(self, cmd_args):
        x_goal = float(cmd_args[0])
        y_goal = float(cmd_args[1])
        theta_goal = float(cmd_args[2])
        
	goal = np.array([x_goal, y_goal, theta_goal])
	self.goal = goal
        rospy.loginfo("-------NEW LINE MOVEMENT-------")
        rospy.loginfo("Goal:\t" + str(goal))
        
        x = self.coords[0]
        y = self.coords[1]
        theta = self.coords[2]

        x_diff = x_goal- x
        y_diff = y_goal - y
        theta_diff = Calculator.wrap_angle(theta_goal-theta)
        
        cmd = " 8 0 0 "  + str(theta_diff)
        rospy.loginfo("Sending cmd: " + cmd)
        self.pub_cmd.publish(cmd)
        rospy.sleep(1)
        self.terminate_following()
        
        vx = x_diff/2
        vy = y_diff/2
        cmd = " 8 " + str(vx) + " " + str(vy) + "  0"
        rospy.loginfo("Sending cmd: " + cmd)
        self.pub_cmd.publish(cmd)
	rospy.sleep(2)
        self.terminate_following()
        

    def update_coords(self):
        try:
	    print self.coords
            trans = self.tfBuffer.lookup_transform('map', 'secondary_robot_odom', rospy.Time())
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
    navigation_node = NavigationNode()
    rospy.spin()

