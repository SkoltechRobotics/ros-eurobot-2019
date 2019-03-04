import numpy as np
import sys
if sys.version_info.major == 2:
    from geometry_msgs.msg import Pose, Transform
    import tf_conversions
    import tf2_ros
    import rospy


def wrap_angle(angle):

    """
    Wraps the given angle to the range [-pi, +pi].
    :param angle: The angle (in rad) to wrap (can be unbounded).
    :return: The wrapped angle (guaranteed to in [-pi, +pi]).
    """

    return (angle + np.pi) % (2 * np.pi) - np.pi


def wrap_back(angle):
    """

    :param angle: (-pi, pi)
    :return: (0, 2*pi)
    """
    return (angle + 2 * np.pi) % (2 * np.pi)


def calculate_distance(coords1, coords2):
    """
    Calculates x_diff, y_diff between two points
    Calculates x_diff, y_diff and theta_diff between two vectors and wrapes angle so it lies
    :param coords1:
    :param coords2:
    :return:
    """
    theta_diff = None
    distance_map_frame = coords2[:2] - coords1[:2]
    if len(coords1) == 3 and len(coords2) == 3:
        theta_diff = wrap_angle(coords2[2] - coords1[2])
    return distance_map_frame, theta_diff


def batch_calculate_distance(coords1, coords2):
    """
    Calculates x_diff, y_diff between two points
    Calculates x_diff, y_diff and theta_diff between two vectors and wrapes angle so it lies
    :param coords1:
    :param coords2:
    :return:
    """
    distance_map_frame = coords2[:, :2] - coords1[:, :2]
    return distance_map_frame


def cvt_local2global(local_point, src_point):
    """
    Convert points from local frame to global

    For example we have a robot located in one point (src_point param) and we
    want it to move half a meter back, while keeping the same orientation.

    The easiest way to do it is to imagine that robot stays at [x=0, y=0, theta=0] in his local frame and
    to substract 0.5 from corresponding axis
    Considering X axis is looking forward, we'll get [-0.5, 0, 0] - this is a local_point param

    :param local_point: A local point or array of local points that must be converted 1-D np.array or 2-D np.array
                        change in pos and orientation in local robot fame
    :param src_point: Point from which robot starts moving
    :return: coordinates [x, y, theta] where we want robot to be in global frame
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


def wrap_angle(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi


def euler_angles_to_rotation_matrix(theta):
    r_x = np.array([[1, 0, 0], [0, np.cos(theta[0]), -np.sin(theta[0])],
                    [0, np.sin(theta[0]),
                     np.cos(theta[0])]])
    r_y = np.array([[np.cos(theta[1]), 0,
                     np.sin(theta[1])], [0, 1, 0],
                    [-np.sin(theta[1]), 0,
                     np.cos(theta[1])]])
    r_z = np.array([[np.cos(theta[2]), -np.sin(theta[2]), 0],
                    [np.sin(theta[2]), np.cos(theta[2]), 0], [0, 0, 1]])
    r = np.dot(r_z, np.dot(r_y, r_x))
    return r


def make_pose3(rotation_matrix, translation_vector):
    return np.concatenate((rotation_matrix, translation_vector[:, None]), axis=1)


def get_rotation_matrix(pose):
    return pose[:, :3]


def get_translation_matrix(pose):
    return pose[:, :3]


def pose_multiplication3(pose1, pose2):
    """
    pose1 * pose2
    :param pose1: np.ndarray shape (3x4)
    :param pose2: np.ndarray shape (3x4)
    :return:np.ndarray shape (3x4)
    """
    assert type(pose1) is np.ndarray
    assert type(pose2) is np.ndarray
    assert pose1.shape == (3, 4)
    assert pose2.shape == (3, 4)
    r1, t1 = pose1[:, :3], pose1[:, 3]
    r2, t2 = pose2[:, :3], pose2[:, 3]
    r = r1.dot(r2)
    t = r1.dot(t2) + t1
    return make_pose3(r, t)


def inverse_pose3(pose):
    assert type(pose) is np.ndarray
    assert pose.shape == (3, 4)
    r, t = pose[:, :3], pose[:, 3]
    return make_pose3(np.linalg.inv(r), (-np.linalg.inv(r)).dot(t))


def cvt_local2global3(local_pose, src_pose):
    """
    Transform pose of a robot from local 3d frame to global 3d frame, global_pose
    pose - concatenation of rotation matrix and translation vector (coordinates of start of frame)
    :param local_pose: np.ndarray shape (3x4)
    :param src_pose: np.ndarray shape (3x4)
    :return:np.ndarray shape (3x4)
    """
    return pose_multiplication3(src_pose, local_pose)


def cvt_global2local3(global_pose, src_pose):
    return pose_multiplication3(inverse_pose3(src_pose), global_pose)


def find_src3(global_pose, local_pose):
    return pose_multiplication3(global_pose, inverse_pose3(local_pose))


def cvt_point2pose3(point):
    return make_pose3(euler_angles_to_rotation_matrix([0, 0, point[2]]), np.array([point[0], point[1], 0]))


# if __name__ == "__main__":
#     point1 = np.array([2, 3, 3.2])
#     point2 = np.array([3, 4, 5.2])
#     pose1 = cvt_point2pose3(point1)
#     pose2 = cvt_point2pose3(point2)
#
#     print(cvt_point2pose3(cvt_global2local2(point1, point2)))
#     print(cvt_global2local3(pose1, pose2))
#     print(cvt_point2pose3(cvt_global2local2(point1, point2)) - cvt_global2local3(pose1, pose2))
#
#     print(cvt_point2pose3(cvt_local2global2(point1, point2)))
#     print(cvt_local2global3(pose1, pose2))
#     print(cvt_point2pose3(cvt_local2global2(point1, point2)) - cvt_local2global3(pose1, pose2))
#
#     print(cvt_point2pose3(find_src2(point1, point2)))
#     print(find_src3(pose1, pose2))
#     print(cvt_point2pose3(find_src2(point1, point2)) - find_src3(pose1, pose2))
