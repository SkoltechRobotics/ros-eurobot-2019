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

        self.robot_name = rospy.get_param("robot_name")
        self.team_color = rospy.get_param("/field/color")
        #self.coords = np.array(rospy.get_param('start_' + self.team_color))
        #self.coords[:2] /= 1000.0
        self.vel = np.zeros(3)

        self.RATE = rospy.get_param("motion_planner/RATE")
        self.XY_GOAL_TOLERANCE = rospy.get_param("motion_planner/XY_GOAL_TOLERANCE")
        self.YAW_GOAL_TOLERANCE = rospy.get_param("motion_planner/YAW_GOAL_TOLERANCE")
        self.V_MAX = rospy.get_param("motion_planner/V_MAX")
        self.W_MAX = rospy.get_param("motion_planner/W_MAX")
        self.ACCELERATION = rospy.get_param("motion_planner/ACCELERATION")
        self.D_DECELERATION = rospy.get_param("motion_planner/D_DECELERATION")
        self.GAMMA = rospy.get_param("motion_planner/GAMMA")
        # for movements to water towers or cube heaps
        self.XY_ACCURATE_GOAL_TOLERANCE = rospy.get_param("motion_planner/XY_ACCURATE_GOAL_TOLERANCE")
        self.YAW_ACCURATE_GOAL_TOLERANCE = rospy.get_param("motion_planner/YAW_ACCURATE_GOAL_TOLERANCE")
        self.D_ACCURATE_DECELERATION = rospy.get_param("motion_planner/D_ACCURATE_DECELERATION")
#         self.NUM_RANGEFINDERS = rospy.get_param("motion_planner/NUM_RANGEFINDERS")
#         self.COLLISION_STOP_DISTANCE = rospy.get_param("motion_planner/COLLISION_STOP_DISTANCE")
#         self.COLLISION_STOP_NEIGHBOUR_DISTANCE = rospy.get_param("motion_planner/COLLISION_STOP_NEIGHBOUR_DISTANCE")
#         self.COLLISION_GAMMA = rospy.get_param("motion_planner/COLLISION_GAMMA")

#         if self.robot_name == "main_robot":
#             # get initial cube heap coordinates
#             self.heap_coords = np.zeros((6, 2))
#             for n in range(6):
#                 self.heap_coords[n, 0] = rospy.get_param("/field/cube" + str(n + 1) + "c_x") / 1000
#                 self.heap_coords[n, 1] = rospy.get_param("/field/cube" + str(n + 1) + "c_y") / 1000
#         else:
#             # get water tower coordinates
#             self.towers = np.array(rospy.get_param("/field/towers"))
#             self.towers[:, :2] /= 1000.0
#             self.tower_approaching_vectors = np.array(rospy.get_param("/field/tower_approaching_vectors"))
#             self.tower_approaching_vectors[:, :2] /= 1000.0

        self.mutex = Lock()

        self.cmd_id = None
        self.t_prev = None
        self.goal = None
        self.mode = None
        # self.rangefinder_data = np.zeros(self.NUM_RANGEFINDERS)
        # self.rangefinder_status = np.zeros(self.NUM_RANGEFINDERS)
        # self.active_rangefinder_zones = np.ones(3, dtype="int")

        self.cmd_stop_robot_id = None
        self.stop_id = 0

        self.pub_twist = rospy.Publisher("cmd_vel", Twist, queue_size=1)
        self.pub_response = rospy.Publisher("response", String, queue_size=10)
        self.pub_cmd = rospy.Publisher("stm_command", String, queue_size=1)
        rospy.Subscriber("move_command", String, self.cmd_callback, queue_size=1)
        rospy.Subscriber("response", String, self.response_callback, queue_size=1)
        # rospy.Subscriber("barrier_rangefinders_data", Int32MultiArray, self.rangefinder_data_callback, queue_size=1)

        # start the main timer that will follow given goal points
        rospy.Timer(rospy.Duration(1.0 / self.RATE), self.plan)



    def cmd_callback(self, data):
        self.mutex.acquire()
        rospy.loginfo("========================================================================================================")
        rospy.loginfo("NEW CMD:\t" + str(data.data))

        # parse name,type
        data_splitted = data.data.split()
        cmd_id = data_splitted[0]
        cmd_type = data_splitted[1]
        cmd_args = data_splitted[2:]

#         if cmd_type == "move" or cmd_type == "move_fast":
#             args = np.array(cmd_args).astype('float')
#             goal = args[:3]
#             goal[2] %= (2 * np.pi)
#             if len(cmd_args) >= 6:
#                 active_rangefinder_zones = np.array(cmd_args[3:7]).astype('float')
#                 self.set_goal(goal, cmd_id, cmd_type, active_rangefinder_zones)
#             else:
#                 self.set_goal(goal, cmd_id, cmd_type)

#         elif cmd_type == "move_heap":
#             self.mode = cmd_type
#             n = int(cmd_args[0])
#             if len(cmd_args) >= 4:
#                 active_rangefinder_zones = np.array(cmd_args[1:4]).astype('float')
#                 self.move_heap(cmd_id, n, active_rangefinder_zones)
#             else:
#                 self.move_heap(cmd_id, n)

        elif cmd_type == "move_odometry":  # simple movement by odometry
            inp = np.array(cmd_args).astype('float')
            inp[2] %= 2 * np.pi
            self.move_odometry(cmd_id, *inp)
        
        elif cmd_type == "translate_odometry":  # simple liner movement by odometry
            inp = np.array(cmd_args).astype('float')
            self.translate_odometry(cmd_id, *inp)

        elif cmd_type == "rotate_odometry":  # simple rotation by odometry
            inp = np.array(cmd_args).astype('float')
            inp[0] %= 2 * np.pi
            self.rotate_odometry(cmd_id, *inp)

#         elif cmd_type == "face_heap":  # rotation (odom) to face cubes
#             n = int(cmd_args[0])
#             if len(cmd_args) > 1:
#                 w = np.array(cmd_args[1]).astype('float')
#                 self.face_heap(cmd_id, n, w)
#             else:
#                 self.face_heap(cmd_id, n)

        elif cmd_type == "stop":
            self.cmd_id = cmd_id
            self.mode = cmd_type 
            #self.terminate_following()
            rospy.loginfo("Setting robot speed to zero.")
            self.stop_robot()
            rospy.loginfo("Robot has stopped.")

        self.mutex.release()        

        
        
        
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
    


    def stop_robot(self):
        self.cmd_stop_robot_id = "stop_" + self.robot_name + str(self.stop_id)
        self.stop_id += 1
        self.robot_stopped = False
        cmd = self.cmd_stop_robot_id + " 8 0 0 0"
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
        

        
    def move_odometry(self, cmd_id, goal_x, goal_y, goal_a, vel=0.3, w=1.5):
        """
        Function 
        """
        goal = np.array([goal_x, goal_y, goal_a])
        rospy.loginfo("-------NEW ODOMETRY MOVEMENT-------")
        rospy.loginfo("Goal:\t" + str(goal))
        while not self.update_coords():
            rospy.sleep(0.05)

        d_map_frame = self.distance(self.coords, goal)
        rospy.loginfo("Distance in map frame:\t" + str(d_map_frame))
        d_robot_frame = self.rotation_transform(d_map_frame, -self.coords[2])
        rospy.loginfo("Distance in robot frame:\t" + str(d_robot_frame))
        d = np.linalg.norm(d_robot_frame[:2])
        rospy.loginfo("Distance:\t" + str(d))

        beta = np.arctan2(d_robot_frame[1], d_robot_frame[0])
        rospy.loginfo("beta:\t" + str(beta))
        da = d_robot_frame[2]
        dx = d * np.cos(beta - da / 2)
        dy = d * np.sin(beta - da / 2)
        d_cmd = np.array([dx, dy, da])
        if da != 0:
            d_cmd[:2] *= da / (2 * np.sin(da / 2))
        rospy.loginfo("d_cmd:\t" + str(d_cmd))

        v_cmd = np.abs(d_cmd) / np.linalg.norm(d_cmd[:2]) * vel
        if abs(v_cmd[2]) > w:
            v_cmd *= w / abs(v_cmd[2])
        rospy.loginfo("v_cmd:\t" + str(v_cmd))
        cmd = cmd_id + " 162 " + str(d_cmd[0]) + " " + str(d_cmd[1]) + " " + str(d_cmd[2]) + " " + str(v_cmd[0]) + " " + str(v_cmd[1]) + " " + str(v_cmd[2])
        rospy.loginfo("Sending cmd: " + cmd)
        self.pub_cmd.publish(cmd)
        
        

    def translate_odometry(self, cmd_id, goal_x, goal_y, vel=0.2):
        goal = np.array([goal_x, goal_y])
        rospy.loginfo("-------NEW LINEAR ODOMETRY MOVEMENT-------")
        rospy.loginfo("Goal:\t" + str(goal))
        while not self.update_coords():
            rospy.sleep(0.05)

        d_map_frame = goal[:2] - self.coords[:2]
        rospy.loginfo("Distance in map frame:\t" + str(d_map_frame))
        d_robot_frame = self.rotation_transform(d_map_frame, -self.coords[2])
        v = np.abs(d_robot_frame) / np.linalg.norm(d_robot_frame) * vel
        cmd = cmd_id + " 162 " + str(d_robot_frame[0]) + ' ' + str(d_robot_frame[1]) + ' 0 ' + str(
            v[0]) + ' ' + str(v[1]) + ' 0'
        rospy.loginfo("Sending cmd: " + cmd)
        self.pub_cmd.publish(cmd)



    def rotate_odometry(self, cmd_id, goal_angle, w=1.0):
        rospy.loginfo("-------NEW ROTATIONAL ODOMETRY MOVEMENT-------")
        rospy.loginfo("Goal angle:\t" + str(goal_angle))
        while not self.update_coords():
            rospy.sleep(0.05)

        delta_angle = goal_angle - self.coords[2]
        rospy.loginfo("Delta angle:\t" + str(delta_angle))
        cmd = cmd_id + " 162 0 0 " + str(delta_angle) + ' 0 0 ' + str(w)
        rospy.loginfo("Sending cmd: " + cmd)
        self.pub_cmd.publish(cmd)


    def update_coords(self):
        try:
            trans = self.tfBuffer.lookup_transform('map', self.robot_name, rospy.Time())
            q = [trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w]
            angle = euler_from_quaternion(q)[2] % (2 * np.pi)
            self.coords = np.array([trans.transform.translation.x, trans.transform.translation.y, angle])
            rospy.loginfo("Robot coords:\t" + str(self.coords))
            return True
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.loginfo("MotionPlanner failed to lookup tf2.")
            return False
