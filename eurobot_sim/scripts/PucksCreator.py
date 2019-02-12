#!/usr/bin/env python  
import rospy
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np

REDIUM_COLOR = (230./255., 0, 0)
GREENIUM_COLOR = (77./255., 255./255., 25./255.)
BLUEIUM_COLOR =(25./255., 153./255., 255./255.)

PUCK_DIAMETER = 0.076
PUCK_HEIGHT = 0.025


class PucksCreatorNode(object):
    def __init__(self):
        rospy.init_node("pucks_creator_node")
        self.publisher = rospy.Publisher("/visualisation/pucks", Marker, queue_size=10)
        
        self.id = 0
        
        self.pucks = self.create_pucks_starting_area()
        for puck in self.pucks:
            self.publisher.publish(puck)
        
    def create_pucks_starting_area(self):
        pucks = []
        pucks.append(self.create_puck(x_coord=0.5,
                                      y_coord=0.45,
                                      color=REDIUM_COLOR))
        pucks.append(self.create_puck(x_coord=0.5,
                                      y_coord=0.75,
                                      color=REDIUM_COLOR))
        pucks.append(self.create_puck(x_coord=0.5,
                                      y_coord=1.05,
                                      color=GREENIUM_COLOR))
        
        return pucks
    
    def create_puck(self, x_coord, y_coord, color):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time()
        marker.ns = "pucks_starting_area"
        marker.id = self.id
        marker.type = Marker.CYLINDER
        marker.action = 0
        marker.pose.position.x = x_coord
        marker.pose.position.y = y_coord
        marker.pose.position.z = 0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = PUCK_DIAMETER
        marker.scale.y = PUCK_DIAMETER
        marker.scale.z = PUCK_HEIGHT
        marker.color.a = 1.0 
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]

        self.id += 1
        return marker
    
if __name__=="__main__":
    node = PucksCreatorNode()
