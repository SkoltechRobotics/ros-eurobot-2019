#!/usr/bin/env python
import time
import re

import rospy
from std_msgs.msg import String



class Manipulator():
    def __init__(self):
        self.publisher = rospy.Publisher("/secondary_robot/stm_command", String, queue_size=10)
        rospy.Subscriber("/seondary_robot/stm_response", String, self.response_callback)
        self.las_response_id = None
        self.last_response_args = None
        time.sleep(2)

    def response_callback(self, data):
        response = data.split()
        if re.match(r"manipulator-*", response[0]):
            self.las_response_id = response[0]
            self.last_response_args = response[1]

    def send_command(self, id, cmd):
        self.publisher.publish(String("manipulator-" + str(id) + " " + str(cmd)))
        while(True):
            if self.last_response_id == id:
                return self.last_response_args




	def collect_puck(self):
		# Release grabber
		print("__1__")
        self.send_command(1, 22)
		# self.publisher.publish(String("manipulator-1 22"))
		# Set pump to the wall
        self.send_command(2, 20)
		# self.publisher.publish(String("manipulator-2 20"))
		# Set pump to the ground
        self.send_command(3, 19)
        #         self.publisher.publish("manipulator-3 19")
		# Start pump
        self.send_command(4, 17)
        #         self.publisher.publish("manipulator-4 17")
		# Set pump to the platform
        self.send_command(5, 21)
                # self.publisher.publish("manipulator-5 21")
		# Prop pack
        self.send_command(6, 23)
                # self.publisher.publish("manipulator-6 23")
		# Stop pump
        self.send_command(7, 18)
                # self.publisher.publish("manipulator-7 18")
		# Set pump to the wall
        self.send_command(8, 20)
                # self.publisher.publish("manipulator-8 20")
		# Grab pack
        self.send_command(9, 24)
		# self.publisher.publish("manipulator-9 24")
		# Release grabber
        self.send_command(10, 22)
		# self.publisher.publish("manipulator-10 22")

if __name__=="__main__":
	print("1")
	rospy.init_node("manuipulator_node", anonymous=True)	
	manipulator = Manipulator()
    manipulator.collect_puck()
	print("2")
