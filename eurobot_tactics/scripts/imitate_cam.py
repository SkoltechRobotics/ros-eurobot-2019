import numpy as np

import rospy
from visualization_msgs.msg import MarkerArray, Marker


def publish_pucks(publisher_pucks, coordinates):
    markers = []
    for num, coord in enumerate(coordinates):
        print (num,"    coord", coord)
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = rospy.Time.now()
        marker.ns = "pucks"
        marker.id = num
        marker.type = 3
        marker.pose.position.x = coord[0]
        marker.pose.position.y = coord[1]
        marker.pose.position.z = 0.0125
        marker.pose.orientation.w = 1
        marker.scale.x = 0.075
        marker.scale.y = 0.075
        marker.scale.z = 0.0125
        marker.color.a = 1
        marker.color.r = 1
        marker.lifetime = rospy.Duration(3)
        markers.append(marker)

    print (markers)
    publisher_pucks.publish(markers)


if __name__ == '__main__':
    rospy.init_node('imitate_cam_node', anonymous=True)
    publisher_pucks = rospy.Publisher("/pucks", MarkerArray, queue_size=1)
    rospy.sleep(2)
    coordinates_list = [[0.9, 0.9],
                        [0.8, 1],
                        [0.9, 1.1],
                        [1, 1]]
    publish_pucks(publisher_pucks, coordinates_list)
