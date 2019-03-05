#!/usr/bin/env python
import numpy as np
import rospy
import tf2_ros
from tf.transformations import euler_from_quaternion
import tf_conversions
from geometry_msgs.msg import TransformStamped, PoseArray, Pose, Quaternion


#
# coords = np.array([0, 0, 0])


def get_coords():
    try:
        t = tf_buffer.lookup_transform('map', 'secondary_robot',  rospy.Time(0))
        q = [t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w]
        yaw = tf_conversions.transformations.euler_from_quaternion(q)[2]
        return True, np.array([t.transform.translation.x, t.transform.translation.y, yaw])
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as msg:
        rospy.logwarn(str(msg))
        return False, np.array([0, 0, 0])




if __name__ == '__main__':
    rospy.init_node('Variance', anonymous=True)
    tf_buffer = tf2_ros.Buffer()
    listener_tf = tf2_ros.TransformListener(tf_buffer)
    coords = np.array([0, 0, 0])
    coords = coords[np.newaxis, :]
    start_time = rospy.Time.now()
    while (not rospy.is_shutdown()) and ((rospy.Time.now() - start_time).to_sec() < 3):
        rospy.sleep(0.1)
        f, r = get_coords()
        if f:
            coords = np.append(coords, r[np.newaxis, :], axis=0)
            print r
    print coords.shape
    #coords = np.delete(coords, 0)
    print coords.shape
    coords[0] = coords[1]
    x = coords[:, 0]
    y = coords[:, 1]
    w = coords[:, 2]
    print np.std(x)
    print np.std(y)
    print np.std(w)