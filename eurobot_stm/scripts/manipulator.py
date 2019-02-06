#!/usr/bin/env python
import time
import rospy
from std_msgs.msg import String



class Manipulator():
	def __init__(self):
		self.publisher = rospy.Publisher("/secondary_robot/stm_command", String, queue_size=10)
		time.sleep(2)
	def collect_puck(self):
		# Release grabber
		print("__1__")
		self.publisher.publish(String("manipulator-1 22"))
		# Set pump to the wall
		self.publisher.publish(String("manipulator-2 20"))
		# Set pump to the ground
                self.publisher.publish("manipulator-3 19")
		# Start pump
                self.publisher.publish("manipulator-4 17")
		# Set pump to the platform
                self.publisher.publish("manipulator-5 21")
		# Prop pack
                self.publisher.publish("manipulator-6 23")
		# Stop pump
                self.publisher.publish("manipulator-7 18")
		# Set pump to the wall
                self.publisher.publish("manipulator-8 20")
		# Grab pack
		self.publisher.publish("manipulator-9 24")
		# Release grabber
		self.publisher.publish("manipulator-10 22")

if __name__=="__main__":
	print("1")
	rospy.init_node("manuipulator_node", anonymous=True)	
	manipulator = Manipulator()
    manipulator.collect_puck()
	print("2")
	#manipulator.collect_puck()
