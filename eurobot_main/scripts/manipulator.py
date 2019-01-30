#!/usr/bin/env python
import time
import rospy
from std_msgs.msg import String

class Manipulator():
	def __init__(self):
		self.publisher = rospy.Publisher("/secondary_robot/stm_command", String, queue_size=10)
		time.sleep(2)
		self.collect_puck()
	def collect_puck(self):
		# Release grabber
		# print("__1__")
		self.publisher.publish(String("22"))
		# Set pump to the wall
		self.publisher.publish(String("20"))
		# Set pump to the ground
                self.publisher.publish("19")
		# Start pump
                self.publisher.publish("17")
		# Set pump to the platform
                self.publisher.publish("21")
		# Prop pack
                self.publisher.publish("23")
		# Stop pump
                self.publisher.publish("18")
		# Set pump to the wall
                self.publisher.publish("20")
		# Grab pack
		self.publisher.publish("24")
		# Release grabber
		self.publisher.publish("22")
		return True

#if __name__=="__main__":
#	print("1")
#	rospy.init_node("manuipulator_node", anonymous=True)	
#	manipulator = Manipulator() 
#	manipulator.collect_puck()
	#manipulator.collect_puck()


