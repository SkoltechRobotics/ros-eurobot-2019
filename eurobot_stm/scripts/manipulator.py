#!/usr/bin/env python
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
        self.id_command = 0
        rospy.sleep(2)

    def response_callback(self, data):
        response = data.data.split()
        if re.match(r"manipulator-\d", response[0]):
            self.last_response_id = response[0]
            self.last_response_args = response[1]

    def send_command(self, cmd):
        while (True):
            self.publisher.publish(String("manipulator-"+self.id_command+" "+str(cmd)))
            self.id_command += 1
            rospy.sleep(0.1)
            if self.last_response_id == (str(id)):
                if self.last_response_args == "OK":

                    return self.last_response_args
                # if don't get response a lot of time



    def calibrate_step_motor(self):
        self.send_command("manipulator-calibrate_step_motor", 32)


    def collect_puck(self):
        # Release grabber
        self.send_command("manipulator-release_grabber", 22)
        rospy.sleep(0.5)
        # Set pump to the wall
        self.send_command("manipulator-set_pump_to_the_wall", 20)
        rospy.sleep(0.5)
        # Set pump to the ground
        self.send_command("manipulator-set_pump_to_the_ground", 19)
        rospy.sleep(0.5)
        # Start pump
        self.send_command("manipulator-start_pump", 17)
        rospy.sleep(0.5)
        # Set pump to the platform
        self.send_command("manipulator-set_pump_to_the_platform", 21)
        rospy.sleep(0.5)
        # Prop pack
        self.send_command("manipulator-prop_pack", 23)
        rospy.sleep(0.5)
        # Stop pump
        self.send_command("manipulator-stop_pump", 18)
        rospy.sleep(0.5)
        # Set pump to the wall
        self.send_command("manipulator-set_pump_to_the_wall", 20)
        rospy.sleep(0.5)
        # Grab pack
        self.send_command("manipulator-grab_pack", 24)
        rospy.sleep(0.5)
        # Release grabber
        self.send_command("manipulator-release_grabber", 22)
