#!/usr/bin/env python
import re

import rospy
from std_msgs.msg import String


class Manipulator(object):
    def __init__(self):
        rospy.init_node("manipulator_node", anonymous=True)

        self.robot_name = "secondary_robot"

        self.response_publisher = rospy.Publisher("manipulator/response", String, queue_size=10)
        rospy.Subscriber("manipulator/command", String, self.command_callback)

        self.publisher = rospy.Publisher("secondary_robot/stm_command", String, queue_size=10)
        rospy.Subscriber("secondary_robot/stm_response", String, self.response_callback)
        self.last_response_id = None
        self.last_response_args = None
        self.id_command = 1
        rospy.sleep(2)

    def command_callback(self, data):
        command = data.data.split()
        cmd_id = command[0]
        print ("CMD_ID=", cmd_id)
        cmd = command[1]
        print ("CMD=", cmd)
        if cmd == "calibrate":
            self.calibrate()
        elif cmd == "take_ground":
            self.take_ground()
        elif cmd == "complete_ground_collect":
            self.complete_ground_collect()
        elif cmd == "collect_wall":
            self.collect_wall()
        elif cmd == "collect_ground":
            self.collect_small()

        self.response_publisher.publish(cmd_id + " success")

    def response_callback(self, data):
        response = data.data.split()
        print ("RESPONSE=", response)
        if re.match(r"manipulator-\d", response[0]):
            self.last_response_id = response[0]
            self.last_response_args = response[1]

    def send_command(self, cmd, args=None):
        if args == None:
            message = "manipulator-" + str(self.id_command) + " " + str(cmd)
        else:
            message = "manipulator-" + str(self.id_command) + " " + str(cmd) + " " + str(args)
        while (True):
            self.publisher.publish(String(message))

            rospy.sleep(0.1)
            if self.last_response_id == ("manipulator-" + str(self.id_command)):
                if self.last_response_args == "OK":
                    self.id_command += 1
                    return self.last_response_args
                # if don't get response a lot of time

    def calibrate(self):
        if self.robot_name == "main_robot":
            self.send_command(33)
            self.send_command(48, 1)

            self.send_command(50, 1)
            self.send_command(52, 1)
            # rospy.sleep(3)
            self.send_command(32)

            self.send_command(48, 0)
            self.send_command(50, 0)

            self.send_command(50, 0)
            self.send_command(52, 0)
            self.send_command(25)
            return True
        elif self.robot_name == "secondary_robot":
            self.send_command(48)
            self.send_command(20)
            self.send_command(25)
            return True


    # def calibrate_small(self):
    #     self.send_command(48)
    #     self.send_command(20)
    #     self.send_command(25)
    #     return True
    #
    #
    # def calibrate_big(self):
    #     # 1) collector move left
    #     # 2) start calibration right stepper
    #     # 3) make step down by right stepper
    #     # 4) collector move right
    #     # 5) start calibration left stepper
    #     # 6) make step down by left stepper
    #     # 7) make step down by left stepper
    #     # 9) collector move default
    #     self.send_command(33)
    #     self.send_command(48, 1)
    #
    #     self.send_command(50, 1)
    #     self.send_command(52, 1)
    #     # rospy.sleep(3)
    #     self.send_command(32)
    #
    #     self.send_command(48, 0)
    #     self.send_command(50, 0)
    #
    #     self.send_command(50, 0)
    #     self.send_command(52, 0)
    #     self.send_command(25)
    #     return True

    def collect_big(self, num):
        # Release grabber
        self.send_command(22)
        # Collector move default
        self.send_command(25)
        # Set pump to the wall
        self.send_command(20)
        # Set pump to the ground
        self.send_command(19)
        # Start pump
        self.send_command(17)
        # Set pump to the platform
        self.send_command(21)
        # Prop pack
        self.send_command(23)
        # Stop pump
        self.send_command(18)
        # Set pump to the wall
        self.send_command(20)
        # Grab pack
        self.send_command(24)
        # Collector move right/left
        self.send_command(32)
        # Make step down left / right collector
        self.send_command(50, 1)
        # Release grabber
        self.send_command(22)
        return True

    # def collect_ground_secondary(self):
    #     # Release grabber
    #     self.send_command(22)
    #     # Set pump to the wall
    #     self.send_command(20)
    #     # Set pump to the ground
    #     self.send_command(19)
    #     # Start pump
    #     self.send_command(17)
    #     # Set pump to the platform
    #     self.send_command(21)
    #     # Prop pack
    #     self.send_command(23)
    #     # Stop pump
    #     self.send_command(18)
    #     # Set pump to the wall
    #     self.send_command(20)
    #     # Grab pack
    #     self.send_command(24)
    #     # Release grabber
    #     self.send_command(22)
    #     return True

    def take_ground(self):
        if self.robot_name == "main_robot":
            pass
        if self.robot_name == "secondary_robot":
            # Release grabber
            self.send_command(22)
            # Set pump to the wall
            self.send_command(20)
            # Set pump to the ground
            self.send_command(19)
            # Start pump
            self.send_command(17)
            return True

    def complete_ground_collect(self):
        if self.robot_name == "main_robot":
            pass
        if self.robot_name == "secondary_robot":
            # Set pump to the platform
            self.send_command(21)
            # Prop pack
            self.send_command(23)
            # Stop pump
            self.send_command(18)
            # Set pump to the wall
            self.send_command(20)
            # Grab pack
            self.send_command(24)
            # Release grabber
            self.send_command(22)
            self.send_command(50)
            
            return True

    def collect_wall(self):
        if self.robot_name == "main_robot":
            pass
        if self.robot_name == "secondary_robot":
            # Release grabber
            self.send_command(22)
            # Set pump to the wall
            self.send_command(20)
            # Start pump
            self.send_command(17)
            # Set pump to the platform
            self.send_command(21)
            # Prop pack
            self.send_command(23)
            # Stop pump
            self.send_command(18)
            # Set pump to the wall
            self.send_command(20)
            # Grab pack
            self.send_command(24)
            # Release grabber
            self.send_command(22)
	        self.send_command(50)
            
            return True


    def release_small(self):
        self.send_command(25)
        self.send_command(50)
        self.send_command(50)
        self.send_command(50)
        self.send_command(52)
        self.send_command(32)
        self.send_command(51)
        self.send_command(51)
        self.send_command(52)
        self.send_command(25)

        self.send_command(50)
        self.send_command(50)
        self.send_command(52)
        self.send_command(32)
        self.send_command(51)
        self.send_command(51)
        self.send_command(52)
        self.send_command(25)

        self.send_command(50)
        self.send_command(50)
        self.send_command(52)
        self.send_command(32)
        self.send_command(51)
        self.send_command(51)
        self.send_command(52)
        self.send_command(25)

        self.send_command(50)
        self.send_command(50)
        self.send_command(52)
        self.send_command(32)
        self.send_command(51)
        self.send_command(51)
        self.send_command(52)
        self.send_command(25)
        return True


    def release_big(self):
        self.send_command(33)
        self.send_command(34)
        self.send_command(51, 1)
        self.send_command(51, 1)
        self.send_command(52, 1)
        self.send_command(35)
        return True

if __name__ == '__main__':
    try:
        manipulator = Manipulator()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
