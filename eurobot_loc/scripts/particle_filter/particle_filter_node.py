#!/usr/bin/env python
import rospy
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import TransformStamped, PoseArray, Pose, Quaternion
from sensor_msgs.msg import LaserScan
import numpy as np
from core_functions import cvt_global2local, cvt_local2global, find_src, wrap_angle
from np_particle import ParticleFilter
from np_triangulation import find_position_triangulation
from std_msgs.msg import String
import tf2_ros
import tf_conversions
import matplotlib as mpl
import scipy.optimize
import tf2_geometry_msgs
PF_RATE = rospy.get_param("rate")
BEAC_R = rospy.get_param("beacons_radius")
WORLD_X = rospy.get_param("world_x")
WORLD_Y = rospy.get_param("world_y")
WORLD_BORDER = rospy.get_param("world_border")
BEAC_L = rospy.get_param("beac_l")
BEAC_BORDER = rospy.get_param("beac_border")

PURPLE_BEACONS = np.array([[WORLD_X + WORLD_BORDER + BEAC_BORDER + BEAC_L / 2., WORLD_Y / 2.],
                           [-(WORLD_BORDER + BEAC_BORDER + BEAC_L / 2.), WORLD_Y - BEAC_L / 2.],
                           [-(WORLD_BORDER + BEAC_BORDER + BEAC_L / 2.), BEAC_L / 2.]])

YELLOW_BEACONS = np.array([[-(WORLD_BORDER + BEAC_BORDER + BEAC_L / 2.), WORLD_Y / 2.],
                          [WORLD_X + WORLD_BORDER + BEAC_BORDER + BEAC_L / 2., WORLD_Y - BEAC_L / 2.],
                          [WORLD_X + WORLD_BORDER + BEAC_BORDER + BEAC_L / 2., BEAC_L / 2.]])



class PFNode(object):
    # noinspection PyTypeChecker
    def __init__(self):
        # Init params
        self.beacons = []
        self.prev_side_status = None
        self.robot_name = rospy.get_param("robot_name")
        rospy.Subscriber("stm/side_status", String, self.callback_side, queue_size=1)
        rospy.Subscriber("/%s/scan"%self.robot_name, LaserScan, self.scan_callback, queue_size=1)
        self.color = "purple"
        if self.color == "purple":
            beacons = PURPLE_BEACONS
        else:
            self.color = "yellow"
            beacons = YELLOW_BEACONS
        self.beacons_publisher = rospy.Publisher("beacons", MarkerArray, queue_size=2)
        self.landmark_publisher = rospy.Publisher("landmarks", MarkerArray, queue_size=2)
        self.tf_buffer = tf2_ros.Buffer()
        self.beacon_radius = rospy.get_param("beacons_radius")
        self.beacon_range = rospy.get_param("beacon_range")
        self.min_range = rospy.get_param("min_range")
        self.min_points_per_beacon = rospy.get_param("min_point_per_beacon")
        self.listener_tf = tf2_ros.TransformListener(self.tf_buffer)
        self.lidar_point = np.array([rospy.get_param("lidar_x"), rospy.get_param("lidar_y"), rospy.get_param("lidar_a")])
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
        lidar_odom_point = cvt_local2global(self.lidar_point, robot_odom_point)
        self.prev_lidar_odom_point = lidar_odom_point.copy()
        self.lidar_odom_point_odom = robot_odom_point.copy()
        self.prev_lidar_odom_point_odom = robot_odom_point.copy()
        self.lidar_odom_time = rospy.Time.now()
        self.prev_lidar_odom_time = rospy.Time.now()
        self.laser_time = rospy.Time.now()
        self.cost_function = []
        self.world_beacons = []
        if self.color == "purple":
            init_start = np.array(rospy.get_param("start_purple"))
        else:
            init_start = np.array(rospy.get_param("start_yellow"))
        self.world_beacons = beacons
        self.robot_pf_point = init_start
        #buf_pf = ParticleFilter(color=self.color, start_x=init_start[0], start_y=init_start[1], start_angle=init_start[2])
        #angles, distances = buf_pf.get_landmarks(self.scan)
        #x = distances * np.cos(angles)
        #y = distances * np.sin(angles)
        #landmarks = (np.array([x, y])).T
        #start_coords = find_position_triangulation(beacons, landmarks, init_start)
        self.pf = ParticleFilter(color=self.color, start_x=init_start[0], start_y=init_start[1], start_angle=init_start[2])
        self.last_odom = np.zeros(3)
        self.alpha = rospy.get_param("alpha")
        rospy.Subscriber("/tf", TransformStamped, self.callback_frame, queue_size=1)
        rospy.Timer(rospy.Duration(1. / PF_RATE), self.localization)

    def callback_side(self, side):
        if self.prev_side_status != side.data:
            if side.data == "1":
                self.color = "yellow"
                init_start = np.array(rospy.get_param("start_yellow"))
                beacons = YELLOW_BEACONS
            else:
                self.color = "purple"
                init_start = np.array(rospy.get_param("start_purple"))
                beacons = PURPLE_BEACONS
            self.world_beacons = beacons
            self.prev_side_status = side.data
            buf_pf = ParticleFilter(color=self.color, start_x=init_start[0], start_y=init_start[1], start_angle=init_start[2])
            angles, distances = buf_pf.get_landmarks(self.scan, 3000)
            x = distances * np.cos(angles)
            y = distances * np.sin(angles)
            landmarks = (np.array([x, y])).T
            start_coords = find_position_triangulation(beacons, landmarks, init_start)
            self.robot_pf_point = start_coords
            self.pf = ParticleFilter(color=self.color, start_x=start_coords[0], start_y=start_coords[1], start_angle=start_coords[2])
        


    def scan_callback(self, scan):
        self.scan = scan
        self.laser_time = scan.header.stamp

    def point_cloud_from_scan(self):
        distances = np.linalg.norm(self.robot_pf_point[:2] - self.world_beacons, axis=1)
        min_dist_to_beacon = min(distances)
        if min_dist_to_beacon < 0.4:
            intense = 2000
        else:
            intense = 3000
        rospy.loginfo(distances)
        angles, ranges = self.pf.get_landmarks(self.scan, intense)
        x = ranges * np.cos(angles)
        y = ranges * np.sin(angles)
        points = np.array([x, y]).T
	#self.pub_landmarks(points, self.scan.header)
        return points

    @staticmethod
    def filter_by_min_range(points, min_range):
        return points[np.linalg.norm(points, axis=1) >= min_range]

    def covariance(self, beacon, points):
        self.cost_function.append(((-2) * (np.linalg.norm(points - beacon) - BEAC_R) * (points - beacon)) \
                                  / np.linalg.norm(points - beacon))

    def beacons_detection(self, points):
        points_number = points.shape[0]
        marked_points = [False] * points_number
        beacons = []
        for i in range(points_number):
            if not marked_points[i]:
                nearest_points = np.linalg.norm(points - points[i], axis=1) < self.beacon_range
                marked_points = nearest_points | marked_points
                rospy.logdebug("num beacons points %d" % int(np.count_nonzero(nearest_points)))
                if np.count_nonzero(nearest_points) >= self.min_points_per_beacon:
                    beacons.append(self.find_beacon(points[nearest_points], self.beacon_radius))
                    rospy.logdebug("find beacon at point (%.3f, %.3f)" % (beacons[-1][0], beacons[-1][1]))
        beacons = np.array(beacons)
        color = np.array([1, 0, 0])
        return np.array(beacons), color

    def publish_landmarks(self, beacons, header):
        markers = []
        self.beacons = beacons
        # color = np.array([1, 0, 0])
        if (beacons.shape[0] == 2):
            dist_beacon = np.sqrt((beacons[0][0] - beacons[1][0])**2 + (beacons[0][1] - beacons[1][1])**2)
            if dist_beacon > 2:
                color = np.array([0, 1, 0])
            else:
                color = np.array([0, 0, 1])
        else:
            color = np.array([1, 1, 0])
        return np.array(beacons), color

    def pub_landmarks(self, beacons, header):
        markers = []
        for i, beacon in enumerate(beacons):
            marker = Marker()
            marker.header = header
            marker.ns = "beacons"
            marker.id = i
            marker.type = 3
            marker.pose.position.x = beacon[0]
            marker.pose.position.y = beacon[1]
            marker.pose.position.z = 0
            marker.pose.orientation.w = 1
            marker.scale.x = 0.01
            marker.scale.y = 0.01
            marker.scale.z = 0.005
            marker.color.a = 1
            marker.color.r = 1
            marker.lifetime = rospy.Duration(0.7)
            markers.append(marker)
        self.landmark_publisher.publish(markers)

    def publish_beacons(self, beacons, header, color):
        markers = []
        for i, beacon in enumerate(beacons):
            marker = Marker()
            marker.header = header
            marker.ns = "beacons"
            marker.id = i
            marker.type = 3
            marker.pose.position.x = beacon[0]
            marker.pose.position.y = beacon[1]
            marker.pose.position.z = 0
            marker.pose.orientation.w = 1
            marker.scale.x = 2*self.beacon_radius
            marker.scale.y = 2*self.beacon_radius
            marker.scale.z = 0.2
            marker.color.a = 1
            marker.color.g = color[1]
            marker.color.b = color[0]
            marker.color.r = color[2]
            marker.lifetime = rospy.Duration(0.1)
            markers.append(marker)
        self.beacons_publisher.publish(markers)


    @staticmethod
    def find_beacon(points, beacon_radius):
        def fun(x):
            scs = np.sum((points - x) * (-x), axis=1) / np.linalg.norm(x)
            return np.abs(np.linalg.norm(points - x, axis=1) - beacon_radius) - np.where(scs < 0, scs, 0)

        res = scipy.optimize.least_squares(fun, points[0])
        return np.array(res.x)


    def callback_frame(self, data):
        try:
            if data.transforms[0].header.frame_id == "%s_odom"%self.robot_name:
                self.prev_lidar_odom_time = self.lidar_odom_time
                self.lidar_odom_time = data.transforms[0].header.stamp
                odom = self.tf_buffer.lookup_transform("%s_odom"%self.robot_name, self.robot_name, self.lidar_odom_time)
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

    def point_extrapolation(self, point1, point2, stamp1, stamp2, stamp3):
        dt21 = stamp2.to_sec() - stamp1.to_sec()
        dt31 = stamp3.to_sec() - stamp1.to_sec()
        if np.abs(dt21) > 1E-6:
            dp = cvt_global2local(point2, point1) * dt31 / dt21
        else:
            dp = np.array([0, 0, 0])
        return cvt_local2global(dp, point1)

    # noinspection PyUnusedLocal
    def localization(self, event):
        header = self.scan.header
        points = self.point_cloud_from_scan()
        #self.publish_landmarks(points, header)
        beacons, color = self.beacons_detection(points)
        self.publish_beacons(beacons, header, color)
        f, robot_odom_point = self.get_odom()
        robot_odom_point = self.point_extrapolation(self.prev_lidar_odom_point_odom, self.lidar_odom_point_odom,
                                                    self.prev_lidar_odom_time, self.lidar_odom_time, self.laser_time)
        if f:
            lidar_odom_point = cvt_local2global(self.lidar_point, robot_odom_point)
            delta = cvt_global2local(lidar_odom_point, self.prev_lidar_odom_point)
            self.prev_lidar_odom_point = lidar_odom_point.copy()
            lidar_pf_point = self.pf.localization(delta, beacons)
            self.robot_pf_point = find_src(lidar_pf_point, self.lidar_point)
            self.publish_pf(self.get_odom_frame(self.robot_pf_point, robot_odom_point))
            self.publish_particles()

    def get_odom(self):
        try:
            t = self.buffer.lookup_transform('%s_odom' % self.robot_name, self.robot_name, rospy.Time(0))
            q = [t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w]
            yaw = tf_conversions.transformations.euler_from_quaternion(q)[2]
            return True, np.array([t.transform.translation.x, t.transform.translation.y, yaw])
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logwarn("Transform for PF with error")

        return False, np.array([0, 0, 0])

    def publish_pf(self, point):
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

    def particles_to_poses(self):
        return map(self.particle_to_pose, self.pf.particles)

    def publish_particles(self):
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
