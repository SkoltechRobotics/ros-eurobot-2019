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
        self.id_command = 1
        rospy.sleep(2)

    def response_callback(self, data):
        response = data.data.split()
        if re.match(r"manipulator-\d", response[0]):
            self.last_response_id = response[0]
            self.last_response_args = response[1]

    def send_command(self, cmd, args=None):
        message = ""
        if args == None:
            message = str(self.id_command) + str(cmd)
        else:
            message = str(self.id_command) + str(cmd) + str(args)
        while (True):
            self.publisher.publish(String("manipulator-"+self.id_command+" "+str(cmd)))
            self.id_command += 1
            rospy.sleep(0.1)
            if self.last_response_id == (str(self.id_command)):
                if self.last_response_args == "OK":
                    return self.last_response_args
                # if don't get response a lot of time


    def calibrate_small_robot(self):
        self.send_command("manipulator-calibrate_step_motor_1", 48)

    def calibrate_big_robot(self):
        # 1) collector move left
        # 2) start calibration right stepper
        # 3) make step down by right stepper
        # 4) collector move right
        # 5) start calibration left stepper
        # 6) make step down by left stepper
        # 7) make step down by left stepper
        # 9) collector move default
        self.send_command(33)
        self.send_command(48, 1)

        self.send_command(50, 1)
        self.send_command(32)

        self.send_command(48, 0)
        self.send_command(50, 0)


        self.send_command(50, 0)
        self.send_command(25)


    def collect_puck(self):
        # Release grabber
        self.send_command("manipulator-release_grabber", 22)
        # Set pump to the wall
        self.send_command("manipulator-set_pump_to_the_wall", 20)
        # Set pump to the ground
        self.send_command("manipulator-set_pump_to_the_ground", 19)
        # Start pump
        self.send_command("manipulator-start_pump", 17)
        # Set pump to the platform
        self.send_command("manipulator-set_pump_to_the_platform", 21)
        # Prop pack
        self.send_command("manipulator-prop_pack", 23)
        # Stop pump
        self.send_command("manipulator-stop_pump", 18)
        # Set pump to the wall
        self.send_command("manipulator-set_pump_to_the_wall", 20)
        # Grab pack
        self.send_command("manipulator-grab_pack", 24)
        # Release grabber
        self.send_command("manipulator-release_grabber", 22)
