#!/usr/bin/env python
import rospy
from geometry_msgs.msg import TransformStamped, PoseArray, Pose, Quaternion
from sensor_msgs.msg import LaserScan
import numpy as np
from np_Particle import ParticleFilter, cvt_global2local, cvt_local2global, find_src
import tf2_ros
import tf_conversions
import matplotlib as mpl
import tf2_geometry_msgs
import time
mpl.rcParams["figure.figsize"] = (6, 4)
mpl.rcParams["figure.dpi"] = 100
#import matplotlib.pyplot as plt
#import threading

PF_RATE = 20

PF_PARAMS = {"particles_num": 1000,
             "sense_noise": 0.00075,
             "distance_noise": 0.002,
             "angle_noise": 0.0035,
             "min_intens": 3000,
             "max_dist": 3.700,
             "k_angle": 0,
             "beac_dist_thresh": 0.95,
             "num_is_near_thresh": 0.1,
             "distance_noise_1_beacon": 0.0133,
             "angle_noise_1_beacon": 0.002}


class PFNode(object):
    # noinspection PyTypeChecker
    def __init__(self):
        # Init params
        rospy.Subscriber("/secondary_robot/scan", LaserScan, self.scan_callback, queue_size=1)
        self.tf_buffer = tf2_ros.Buffer()
        self.listener_tf = tf2_ros.TransformListener(self.tf_buffer)
        self.color = rospy.get_param("/field/color")
        self.robot_name = rospy.get_param("robot_name")
        self.lidar_point = np.array([rospy.get_param("lidar_x"), rospy.get_param("lidar_y"),
                                     rospy.get_param("lidar_a")])
        self.particle_pub = rospy.Publisher("particles", PoseArray, queue_size=1)
        self.scan = None
        self.buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buffer)
        self.br = tf2_ros.TransformBroadcaster()
        self.lidar_pf_coord = self.lidar_point

        rospy.sleep(1)
        f, robot_odom_point = self.get_odom()
        while not f and not rospy.is_shutdown():
            f, robot_odom_point = self.get_odom()
            rospy.sleep(0.2)
        lidar_odom_point = cvt_local2global(self.lidar_point, robot_odom_point)
        self.prev_lidar_odom_point = lidar_odom_point.copy()
        self.lidar_odom_point_odom = robot_odom_point.copy()
        self.prev_lidar_odom_point_odom = robot_odom_point.copy()
        self.lidar_odom_time = rospy.Time.now()
        rospy.loginfo(self.lidar_odom_point_odom)
        self.prev_lidar_odom_time = rospy.Time.now()
        self.laser_time = rospy.Time.now()
        x, y, a = lidar_odom_point

        if self.robot_name == "secondary_robot":
            k_bad = 0
        else:
            k_bad = 2
        self.pf = ParticleFilter(color=self.color, start_x=x, start_y=y, start_angle=a, k_bad=k_bad, **PF_PARAMS)
        self.last_odom = np.zeros(3)
        self.alpha = 1
        rospy.Subscriber("/tf", TransformStamped, self.callback_frame, queue_size=1)
        rospy.Timer(rospy.Duration(1. / PF_RATE), self.localisation)
        #time.sleep(2)
        #rospy.visualization_timer = rospy.Timer(rospy.Duration(1. / 3), self.vis_particles)

    def scan_callback(self, scan):
        self.scan = np.array([np.array(scan.ranges), scan.intensities]).T
        self.laser_time = scan.header.stamp

    # def interpolate(self):
    #     delta_time_odom = (self.stamp2secs(self.lidar_odom_time) - self.stamp2secs(self.prev_lidar_odom_time))
    #     delta_time_laser = (self.stamp2secs(self.laser_time) - self.stamp2secs(self.lidar_odom_time))
    #     rospy.loginfo("delta time odom %.4f" % delta_time_odom)
    #     rospy.loginfo("delta time laser %.4f" % delta_time_laser)
    #     if np.abs(delta_time_odom) > 1E-6:
    #         delta_point = cvt_global2local(self.lidar_odom_point, self.prev_lidar_odom_point) * \
    #                       (delta_time_laser / delta_time_odom)
    #     else:
    #         delta_point = np.array([0, 0, 0])
    #     rospy.loginfo(cvt_local2global(delta_point, self.prev_lidar_odom_point))
    #     return cvt_local2global(delta_point, self.prev_lidar_odom_point)



    def callback_frame(self, data):
        try:
            if data.transforms[0].header.frame_id == "secondary_robot_odom":
                self.prev_lidar_odom_time = self.lidar_odom_time
                self.lidar_odom_time = data.transforms[0].header.stamp
                #rospy.loginfo(str(self.time_tf_stamp[-1].secs + self.time_tf_stamp[-1].nsecs * 1e-9))
                odom = self.tf_buffer.lookup_transform("secondary_robot_odom", "secondary_robot", self.lidar_odom_time)
                robot_odom_point = self.tf_to_coords(odom)
                self.prev_lidar_odom_point_odom = self.lidar_odom_point_odom.copy()
                self.lidar_odom_point_odom = robot_odom_point
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            pass

    def get_odom_frame(self, robot_pf_point, robot_odom_point):
        odom = find_src(robot_pf_point, robot_odom_point)
        self.last_odom[:2] = (1 - self.alpha) * self.last_odom[:2] + self.alpha * odom[:2]
        self.last_odom[2] = odom[2]
        return self.last_odom

    def tf_to_coords(self, odom):
        q = [odom.transform.rotation.x, odom.transform.rotation.y,
             odom.transform.rotation.z, odom.transform.rotation.w]
        yaw = tf_conversions.transformations.euler_from_quaternion(q)[2]
        x = odom.transform.translation.x
        y = odom.transform.translation.y
        return np.array([x, y, yaw])

    @staticmethod
    def point_extrapolation(point1, point2, stamp1, stamp2, stamp3):
        dt21 = (stamp2 - stamp1).to_sec()
        dt31 = (stamp3 - stamp1).to_sec()
        rospy.loginfo("delta time odom %.4f" % dt21)
        rospy.loginfo("delta time laser %.4f" % dt31)
        if np.abs(dt21) > 1E-6:
            dp = cvt_global2local(point2, point1) * dt31 / dt21
            rospy.loginfo("dp")
            rospy.loginfo(dp)
        else:
            dp = np.array([0, 0, 0])
        rospy.loginfo(cvt_local2global(dp, point1))
        return cvt_local2global(dp, point1)

    # noinspection PyUnusedLocal
    def localisation(self, event):
        f, robot_odom_point = self.get_odom()
        robot_odom_point = self.point_extrapolation(self.prev_lidar_odom_point_odom, self.lidar_odom_point_odom,
                                                    self.prev_lidar_odom_time, self.lidar_odom_time, self.laser_time)
        rospy.loginfo("robot odom point")
        rospy.loginfo(robot_odom_point)
        if f:
            lidar_odom_point = cvt_local2global(self.lidar_point, robot_odom_point)
            delta = cvt_global2local(lidar_odom_point, self.prev_lidar_odom_point)
            rospy.loginfo("delta")
            rospy.loginfo(delta)
            self.prev_lidar_odom_point = lidar_odom_point.copy()

            rospy.loginfo("odom_point %.4f %.4f %.4f" % tuple(lidar_odom_point))

            lidar_pf_point = self.pf.localisation(delta, self.scan)
            rospy.loginfo("pf_point   %.4f %.4f %.4f" % tuple(lidar_pf_point))

            # rospy.loginfo("cost_function " + str(self.pf.min_cost_function))

            robot_pf_point = find_src(lidar_pf_point, self.lidar_point)
            # self.vis_particles(robot_odom_point)
            self.pub_pf(self.get_odom_frame(robot_pf_point, robot_odom_point))
            self.pub_particles()

    def get_odom(self):
        try:
            t = self.buffer.lookup_transform('%s_odom' % self.robot_name, self.robot_name, rospy.Time(0))
            q = [t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w]
            yaw = tf_conversions.transformations.euler_from_quaternion(q)[2]
            return True, np.array([t.transform.translation.x, t.transform.translation.y, yaw])
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logwarn("Transform for PF with error")

        return False, np.array([0, 0, 0])

    def pub_pf(self, point):
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "map"
        t.child_frame_id = "%s_odom" % self.robot_name
        t.transform.translation.x = point[0]
        t.transform.translation.y = point[1]
        t.transform.translation.z = 0.0
        q = tf_conversions.transformations.quaternion_from_euler(0, 0, point[2])
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        self.br.sendTransform(t)

    def particle_to_pose(self, particle):
        pose = Pose()
        pose.position.x = particle[0]
        pose.position.y = particle[1]
        pose.position.z = 1
        pose.orientation = Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, 0, particle[2]))
        return pose

    # def vis_particles(self, event):
    #     points = cvt_local2global(self.pf.landmarks, self.lidar_pf_coord)
    #     plt.cla()
    #     plt.scatter(points[:, 0], points[:, 1], s=1, c='blue')
    #     plt.scatter(self.pf.beacons[:, 0], self.pf.beacons[:, 1], s=20, c='r')
    #     plt.xlim(-0.15, 3.15)
    #     plt.ylim(-0.15, 2.15)
    #     weights = self.pf.weights(self.pf.landmarks, self.pf.particles)
    #     ps = self.pf.particles
    #     plt.scatter(ps[:, 0], ps[:, 1], c="red", s=weights * 1000)
    #     plt.quiver(ps[:, 0], ps[:, 1], np.cos(ps[:, 2]), np.sin(ps[:, 2]), width=0.001, color="red")
    #     plt.pause(0.001)

    def particles_to_poses(self):
        return map(self.particle_to_pose, self.pf.particles)

    def pub_particles(self):
        pose_viz = PoseArray()
        pose_viz.header.stamp = rospy.Time.now()
        pose_viz.header.frame_id = "map"
        pose_viz.poses = self.particles_to_poses()
        self.particle_pub.publish(pose_viz)

    def stamp2secs(self, time):
        try:
            return time.secs + time.nsecs * 1e-9
        except:
            return time


if __name__ == '__main__':
    try:
        rospy.init_node('particle_filter_node', anonymous=True)
        pf_node = PFNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
