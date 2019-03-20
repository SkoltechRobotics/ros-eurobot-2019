#!/usr/bin/env python
import re

import rospy
from std_msgs.msg import String


class Manipulator(object):
    def __init__(self):
        rospy.init_node("manipulator_node", anonymous=True)

        self.robot_name = rospy.get_param("robot_name")

        self.response_publisher = rospy.Publisher("manipulator/response", String, queue_size=10)
        rospy.Subscriber("manipulator/command", String, self.command_callback)

        self.publisher = rospy.Publisher("stm/command", String, queue_size=10)
        rospy.Subscriber("stm/response", String, self.response_callback)

        self.last_response_id = None
        self.last_response_args = None
        self.id_command = 1
        rospy.sleep(2)

    def command_callback(self, data):
        command = data.data.split()
        cmd_id = command[0]
        print("CMD_ID=", cmd_id)
        cmd = command[1]
        print("CMD=", cmd)
        if cmd == "default":
            self.calibrate()
        elif cmd == "manipulator_wall":
            self.set_manipulator_wall()
        elif cmd == "manipulator_up":
            self.set_manipulator_up()
        elif cmd == "take_ground":
            self.take_ground()
        elif cmd == "complete_ground_collect":
            self.complete_ground_collect()
        elif cmd == "start_collect_wall":
            self.start_collect_wall()
        elif cmd == "complete_collect_wall":
            self.complete_collect_wall()
        elif cmd == "release_5":
            self.release(5)
        elif cmd == "collect_ground":
            self.collect_small()
        elif cmd == "release_accelerator":
            self.release_accelerator()
        elif cmd == "manipulator_up_and_keep_holding":
            self.manipulator_up_and_keep_holding()
        elif cmd == "set_angle_to_push_blunium":
            self.set_angle_to_push_blunium()
        elif cmd == "grab_goldenium_and_hold_up":
            self.goldenium_up_and_hold()
        elif cmd == "release_goldenium_on_scales":
            self.release_goldenium_on_scales()

        self.response_publisher.publish(cmd_id + " success")

    def response_callback(self, data):
        response = data.data.split()
        print("RESPONSE=", response)
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
            self.send_command(48)
            self.send_command(21)
            self.send_command(25)  # FIXME
            return True
        elif self.robot_name == "secondary_robot":
            self.send_command(48)
            self.send_command(50)
            self.send_command(23)
            self.send_command(21)
            self.send_command(25)
            return True

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

    def set_manipulator_wall(self):
        self.send_command(20)

    def set_manipulator_up(self):
        self.send_command(24)
        self.send_command(21)

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

    def start_collect_wall(self):
        if self.robot_name == "main_robot":
            pass
        if self.robot_name == "secondary_robot":
            # Release grabber
            self.send_command(22)
            # Set pump to the wall
            self.send_command(20)
            # rospy.sleep(0.5)
            # Start pump
            self.send_command(17)
            return True

    def complete_collect_wall(self):
        if self.robot_name == "main_robot":
            pass
        if self.robot_name == "secondary_robot":
            rospy.sleep(0.3)

            # Set pump to the platform
            self.send_command(21)
            # Prop pack
            self.send_command(23)
            # Stop pump
            self.send_command(18)
            # Set pump to the wall
            self.send_command(20)
            # grab pack
            self.send_command(24)

            # Release grabber
            self.send_command(22)
            # Set pump to the platform
            # self.send_command(21)

            self.send_command(50)
            return True

    def release(self, pucks_number):
        self.send_command(25)

        self.send_command(51)
        self.send_command(52)
        self.send_command(51)
        self.send_command(52)
        self.send_command(32)
        self.send_command(25)
        
        self.send_command(51)
        self.send_command(52)
        self.send_command(32)
        self.send_command(25)

        self.send_command(51)
        self.send_command(52)
        self.send_command(32)
        self.send_command(25)

        self.send_command(51)
        self.send_command(52)
        self.send_command(32)
        self.send_command(25)

        self.send_command(51)
        self.send_command(52)
        self.send_command(32)
        self.send_command(25)


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

    def release_accelerator(self):
        # assume that we need to move pucks 1 level up to start throwing them

        # release grabber
        self.send_command(22)
        # make step up
        self.send_command(51)
        # grabber throw (up)
        self.send_command(19)
        return True

    def manipulator_up_and_keep_holding(self):
        # release grabber
        self.send_command(22)
        # Set pump to the platform
        self.send_command(21)
        return True

    def set_angle_to_push_blunium(self):
        # push blunium
        self.send_command(34)
        return True

    def goldenium_up_and_hold(self):
        # release grabber
        self.send_command(22)
        # set angle to grab goldenium
        self.send_command(35)
        # start pump
        self.send_command(17)
        # lift up goldenium
        self.send_command(36)
        return True

    def release_goldenium_on_scales(self):
        # set pump to the wall
        self.send_command(20)
        # stop pump
        self.send_command(20)
        # set pump to the platform
        self.send_command(21)
        return True


if __name__ == '__main__':
    try:
        manipulator = Manipulator()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
