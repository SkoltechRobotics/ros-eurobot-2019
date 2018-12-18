#!/usr/bin/env python
# coding: utf-8

# In[1]:



# coding: utf-8

# In[4]:


# simple go to goal in different methods: odom movement
#!/usr/bin/env python
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
        
        #self.coords[:2] /= 1000.0
        
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
        
        self.V_MAX = 0.7 # m/s
        self.V_MIN = 0.15
        self.W_MAX = 3.14
        
        self.Kp_rho = 0.5
        self.Kp = 30
        
        #w = 1.5
        self.k = 0.2
        self.Kv = 1
        self.THRESHOLD_XY = 0.05
        self.THRESHOLD_YAW = 0.02
        
        # calculated distance and rot to goal from initial point, stays unchanged
        self.d_init = 0
        self.alpha_init = 0
        
        self.cmd_stop_robot_id = None
        self.stop_id = 0

#         self.pub_twist = rospy.Publisher("cmd_vel", Twist, queue_size=1)
        self.pub_response = rospy.Publisher("response", String, queue_size=10)
        self.pub_cmd = rospy.Publisher("/secondary_robot/stm_command", String, queue_size=1)
        #rospy.Subscriber("move_command", String, self.cmd_callback, queue_size=1)
        rospy.Subscriber("move_command", String, self.cmd_callback, queue_size=1)
#         rospy.Subscriber("response", String, self.response_callback, queue_size=1)
        # rospy.Subscriber("barrier_rangefinders_data", Int32MultiArray, self.rangefinder_data_callback, queue_size=1)

        # start the main timer that will follow given goal points
        rospy.Timer(rospy.Duration(1.0 / self.RATE), self.arc_move)


    def cmd_callback(self, data):
        self.mutex.acquire()
        rospy.loginfo("========================================================================================================")
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
        
        x_goal = float(self.cmd_args[0])
        y_goal = float(self.cmd_args[1])
        theta_goal = float(self.cmd_args[2])
        
        self.d_init = np.sqrt((x_goal - x_start)**2 + (y_goal - y_start)**2)
        self.alpha_init = wrap_angle(theta_goal - theta_start) # theta_diff
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
        
    def vel(path_done, path_left, V_MIN, V_MAX, k, Kp):
        if path_done < path_left:
            #v = min(V_MAX * np.e**(-1 / (path_done / k + 0.1)) + V_MIN, V_MAX) 
            
            # use linear ACC
            v = min(V_MAX, Kp*path_done + V_MIN)
        else:
            # expo DCL
            v = min(V_MAX * np.e**(-1 / (path_left / k + 0.1)) + V_MIN, V_MAX)
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

        pi2 = 2 * np.pi

        while angle < -np.pi:
            angle += pi2

        while angle >= np.pi:
            angle -= pi2

        return angle
    

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
        
        
    def arc_move(self, vel=0.3, wmax=1.5, t = 0.1):
        "go to goal in one movement by arc path"
        "t chosen to be 0.1 for testing, meaning 10 Hz"
        "positioning args vel and wmax are not used"
        
        try:
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
            
            # vector norm, simply saying: an amplitude of a vector
            d = np.sqrt(x_diff**2 + y_diff**2) # rho, np.linalg.norm(d_map_frame[:2])
            alpha = wrap_angle(theta_goal - theta) # theta_diff

            # global ***************
            path_done = np.sqrt(self.d_init**2 + self.alpha_init**2) - np.sqrt(d**2 + alpha**2)
            path_left = np.sqrt(d**2 + alpha**2)

            beta = self.wrap_angle(gamma - alpha/2)
            
            v = self.Kv * self.vel(path_done, path_left, self.V_MIN, self.V_MAX, self.k, self.Kp)

            if d < self.THRESHOLD_XY and alpha < self.THRESHOLD_YAW:
                v = 0
                w = 0
                self.stop_robot()
            elif d == 0:
                v = 0
                w = self.W_MAX # change ******************
            elif alpha == 0:
                w = 0
            else:
                R = 0.5 * d / np.sin(alpha/2)
                w = v / R  # must be depended on v such way so path becomes an arc

            vx = v * np.cos(beta)
            vy = v * np.sin(beta)

            v_cmd = np.array([vx, vy, w])        
            rospy.loginfo("v_cmd:\t" + str(v_cmd))
            cmd = " 8 " + str(v_cmd[0]) + " " + str(v_cmd[1]) + " " + str(v_cmd[2])
            rospy.loginfo("Sending cmd: " + cmd)
            self.pub_cmd.publish(cmd)
            
        except:
            pass
        

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
 

