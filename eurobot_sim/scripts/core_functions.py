#!/usr/bin/env python

import numpy as np
import sys
if sys.version_info.major == 2:
    from geometry_msgs.msg import Pose, Transform
    import tf_conversions
    import tf2_ros
    import rospy


def cvt_local2global(local_point, src_point):
    """
    Convert points from local frame to global
    :param local_point: A local point or array of local points that must be converted 1-D np.array or 2-D np.array
    :param src_point: A
    :return:
    """
    size = local_point.shape[-1]
    x, y, a = 0, 0, 0
    if size == 3:
        x, y, a = local_point.T
    elif size == 2:
        x, y = local_point.T
    X, Y, A = src_point.T
    x1 = x * np.cos(A) - y * np.sin(A) + X
    y1 = x * np.sin(A) + y * np.cos(A) + Y
    a1 = (a + A + np.pi) % (2 * np.pi) - np.pi
    if size == 3:
        return np.array([x1, y1, a1]).T
    elif size == 2:
        return np.array([x1, y1]).T
    else:
        return


def cvt_global2local(global_point, src_point):
    size = global_point.shape[-1]
    x1, y1, a1 = 0, 0, 0
    if size == 3:
        x1, y1, a1 = global_point.T
    elif size == 2:
        x1, y1 = global_point.T
    X, Y, A = src_point.T
    x = x1 * np.cos(A) + y1 * np.sin(A) - X * np.cos(A) - Y * np.sin(A)
    y = -x1 * np.sin(A) + y1 * np.cos(A) + X * np.sin(A) - Y * np.cos(A)
    a = (a1 - A + np.pi) % (2 * np.pi) - np.pi
    if size == 3:
        return np.array([x, y, a]).T
    elif size == 2:
        return np.array([x, y]).T
    else:
        return


def find_src(global_point, local_point):
    x, y, a = local_point.T
    x1, y1, a1 = global_point.T
    A = (a1 - a) % (2 * np.pi)
    X = x1 - x * np.cos(A) + y * np.sin(A)
    Y = y1 - x * np.sin(A) - y * np.cos(A)
    return np.array([X, Y, A]).T


def cvt_point2ros_pose(point):
    pose = Pose()
    pose.position.x = point[0]
    pose.position.y = point[1]
    q = tf_conversions.transformations.quaternion_from_euler(0, 0, point[2])
    pose.orientation.x = q[0]
    pose.orientation.y = q[1]
    pose.orientation.z = q[2]
    pose.orientation.w = q[3]
    return pose


def cvt_ros_pose2point(pose):
    x = pose.position.x
    y = pose.position.y
    q = [pose.orientation.x,
         pose.orientation.y,
         pose.orientation.z,
         pose.orientation.w]
    _, _, a = tf_conversions.transformations.euler_from_quaternion(q)
    return np.array([x, y, a])


def cvt_point2ros_transform(point):
    transform = Transform()
    transform.translation.x = point[0]
    transform.translation.y = point[1]
    q = tf_conversions.transformations.quaternion_from_euler(0, 0, point[2])
    transform.rotation.x = q[0]
    transform.rotation.y = q[1]
    transform.rotation.z = q[2]
    transform.rotation.w = q[3]
    return transform


def cvt_ros_transform2point(transform):
    x = transform.translation.x
    y = transform.translation.y
    q = [transform.rotation.x,
         transform.rotation.y,
         transform.rotation.z,
         transform.rotation.w]
    _, _, a = tf_conversions.transformations.euler_from_quaternion(q)
    return np.array([x, y, a])


def cvt_ros_scan2points(scan):
    ranges = np.array(scan.ranges)
    n = ranges.shape[0]
    angles = np.arange(scan.angle_min, scan.angle_min + n * scan.angle_increment, scan.angle_increment)
    x = ranges * np.cos(angles)
    y = ranges * np.sin(angles)
    return np.array([x, y]).T


def get_transform(buffer_, child_frame, parent_frame, stamp):
    t = buffer_.lookup_transform(parent_frame, child_frame, stamp)
    return cvt_ros_transform2point(t.transform)
