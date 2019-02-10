#!/usr/bin/env python
import time
import re

import rospy
from std_msgs.msg import String



class Manipulator():
    def __init__(self):
        rospy.init_node("manuipulator_node", anonymous=True)

        self.publisher = rospy.Publisher("/secondary_robot/stm_command", String, queue_size=10)
        rospy.Subscriber("/secondary_robot/stm_response", String, self.response_callback)
        self.last_response_id = None
        self.last_response_args = None
        rospy.sleep(2)

    def response_callback(self, data):
        response = data.data.split()
        if re.match(r"manipulator-\d", response[0]):
            print ("response[0]", response[0])
        self.last_response_id = response[0]
            self.last_response_args = response[1]

    def send_command(self, id, cmd):
        while (True):
            self.publisher.publish(String("manipulator-" + str(id) + " " + str(cmd)))
            print ("last_response_id=",self.last_response_id)
            print("laste_response_args", self.last_response_args)
            rospy.sleep(0.1)
            if self.last_response_id == ("manipulator-"+str(id)):
                print ("YYYYYEEEESSSS")
                if self.last_response_args == "OK":
                    print("AS<AKMSFLAMSFALFM")
                    return self.last_response_args


    def collect_puck(self):
        # Release grabber
        print("__1__")
        self.send_command(1, 22)
        rospy.sleep(0.5)
        # self.publisher.publish(String("manipulator-1 22"))
        # Set pump to the wall
        self.send_command(2, 20)
        rospy.sleep(0.5)

        # self.publisher.publish(String("manipulator-2 20"))
        # Set pump to the ground
        self.send_command(3, 19)
        rospy.sleep(0.5)

        #         self.publisher.publish("manipulator-3 19")
        # Start pump
        self.send_command(4, 17)
        rospy.sleep(0.5)

        #         self.publisher.publish("manipulator-4 17")
        # Set pump to the platform
        self.send_command(5, 21)
        rospy.sleep(0.5)

        # self.publisher.publish("manipulator-5 21")
        # Prop pack
        self.send_command(6, 23)
        rospy.sleep(0.5)

        # self.publisher.publish("manipulator-6 23")
        # Stop pump
        self.send_command(7, 18)
        rospy.sleep(0.5)

        # self.publisher.publish("manipulator-7 18")
        # Set pump to the wall
        self.send_command(8, 20)
        rospy.sleep(0.5)

        # self.publisher.publish("manipulator-8 20")
        # Grab pack
        self.send_command(9, 24)
        rospy.sleep(0.5)

        # self.publisher.publish("manipulator-9 24")
        # Release grabber
        self.send_command(10, 22)
        # self.publisher.publish("manipulator-10 22")