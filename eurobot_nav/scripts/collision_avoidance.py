#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64MultiArray, String
from visualization_msgs.msg import MarkerArray, Marker
from core_functions import *
from sensor_msgs.msg import LaserScan
import tf2_ros
import tf_conversions
import tf2_geometry_msgs
from tf.transformations import euler_from_quaternion


LIDAR_DELTA_ANGLE = (np.pi / 180) / 4
LIDAR_START_ANGLE = -(np.pi / 2 + np.pi / 4)
class CollisionAvoidance(object):

    def __init__(self):
        rospy.init_node("collision_avoidance", anonymous=True)
        rospy.Subscriber("/secondary_robot/scan", LaserScan, self.scan_callback, queue_size=1)
        rospy.Subscriber("/sensors", Float64MultiArray, self.distances_callback, queue_size=10)
        rospy.Subscriber("/point", String, self.point_callback, queue_size=10)
        self.point_publisher = rospy.Publisher("/obstacle_point", MarkerArray, queue_size=1)
        self.min_dist2obstacle = 0.1
        self.sensors_num = 3
        self.min_dist_before_terminating = 0.05
        self.points = np.ones(self.sensors_num) * 100
        self.distances = None
        # self.sensors_coords = np.array([0., 0., 0.], [2, 2, 2], [3, 3, 3]])
        self.obstacle_points = np.zeros((3, 2))
        self.distance_sensors_centre = 0.2
        self.scan = None
        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer)
        self.coords = None
        #self.max_dist = 3.5
        self.min_sin = 0.3
        #self.robot_name = "secondary_robot"
        self.max_dist = 0.5

    def get_landmarks(self, scan):
        """Returns filtrated lidar data"""
        ranges = np.array(scan.ranges)
        ind = self.filter_scan(scan)
        final_ind = np.where((np.arange(ranges.shape[0]) * ind) > 0)[0]
        angles = (LIDAR_DELTA_ANGLE * final_ind + LIDAR_START_ANGLE) % (2 * np.pi)
        distances = ranges[final_ind]
        return angles, distances

    def scan_callback(self, scan):
        self.scan = scan
        self.update_coords()
        self.scan_stamp = scan.header.stamp
        angles, distances = self.get_landmarks(self.scan)
        x = distances * np.cos(angles)
        y = distances * np.sin(angles)
        landmarks = (np.array([x, y])).T
        landmarks = cvt_local2global(landmarks, self.coords)
        self.set_collision_point(landmarks)

    def filter_scan(self, scan):
        ranges = np.array(scan.ranges)
        intensities = np.array(scan.intensities)
        cloud = cvt_ros_scan2points(scan)
        index0 = ranges < self.max_dist
        index1 = self.alpha_filter(cloud, self.min_sin)
        index = index0 * index1
        return np.where(index, ranges, 0)

    @staticmethod
    def alpha_filter(cloud, min_sin_angle):
        x, y = cloud.T
        x0, y0 = 0, 0
        x1, y1 = np.roll(x, 1), np.roll(y, 1)
        cos_angle = ((x - x0) * (x - x1) + (y - y0) * (y - y1)) / (np.sqrt((x - x0) * (x - x0) + (y - y0) * (y - y0))
                                                                   * np.sqrt((x - x1) * (x - x1) + (y - y1) * (y - y1)))
        sin_angle = np.sqrt(1 - cos_angle * cos_angle)
        index = sin_angle >= min_sin_angle
        return index
    def set_collision_point(self, positions):
        marker = []
        for i, position in enumerate(positions):
            point = Marker()
            point.header.stamp = rospy.Time.now()
            point.header.frame_id = "map"
            point.id = i
            point.type = 3
            point.ns = "obstacle"
            point.pose.position.x = position[0]
            point.pose.position.y = position[1]
            point.pose.position.z = 0
            point.pose.orientation.w = 1
            point.scale.x = 0.2
            point.scale.y = 0.2
            point.scale.z = 0.35
            point.color.a = 1
            point.color.r = 1
            #point.lifetime = rospy.Duration(0.7)
            marker.append(point)
        self.point_publisher.publish(marker)

    def distances_callback(self, distances):
        for i in range(self.sensors_num):
            if distances[i].data > 0:
                self.distances[i] = float(distances[i].data)
            else:
                self.distances[i] = 100

    def point_callback(self, data):
        data_split = data.data.split()
        x = data_split[0]
        y = data_split[1]
        self.set_collision_point(np.array([[x, y]]).astype(float))

    def cvt_distances2points(self):
        return self.sensors_coords + (self.distances + self.distance_sensors_centre)/self.distances
    
    def update_coords(self):
        try:
            trans = self.tfBuffer.lookup_transform('map', self.robot_name, rospy.Time())
            q = [trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z,
                    trans.transform.rotation.w]
            angle = euler_from_quaternion(q)[2] % (2 * np.pi)
            self.coords = np.array([trans.transform.translation.x, trans.transform.translation.y, angle])
            rospy.loginfo("Robot coords:\t" + str(self.coords))
            return True
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as msg:
            rospy.logwarn(str(msg))
            return False

if __name__ == "__main__":
    collision_avoidance = CollisionAvoidance()
    rospy.spin()




