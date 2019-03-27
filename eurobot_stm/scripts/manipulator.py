#!/usr/bin/env python
import re
import enum

import rospy
from std_msgs.msg import String


class ResponseStatus(enum.Enum):
    OK = "OK"
    ERROR = "ER"


class Protocol(enum.Enum):
    SET_ANGLE = 0x10
    START_PUMP = 0x11
    STOP_PUMP = 0x12
    SET_GROUND = 0x13
    SET_WALL = 0x14
    SET_PLATFORM = 0x15
    OPEN_GRABBER = 0x16
    PROP_PUCK_GRABBER = 0x17
    GRAB_PUCK_GRABBER = 0x18
    # Secondary
    RELEASER_DEFAULT_SECONDARY = 0x19
    RELEASER_THROW_SECONDARY = 0x20
    # Main
    RELEASE_PUCK_TOP_MAIN = 0x21
    RELEASER_DEFAULT_MAIN = 0x22
    RELEASE_PUCK_BOT_MAIN = 0x23
    SET_BLUNIUM_ANGLE_MAIN = 0x24
    SET_GRAB_GOLDENIUM_ANGLE_MAIN = 0x25
    SET_LIFT_GOLDENIUM_ANGLE_MAIN = 0x26
    # ---
    START_CALIBRATION = 0x30
    MAKE_STEP = 0x31
    MAKE_STEP_DOWN = 0x32
    MAKE_STEP_UP = 0x33
    GET_STEP_MOTOR_STATUS = 0x34
    GET_PROXIMITY_STATUS = 0x40


class Manipulator(object):
    def __init__(self):
        rospy.init_node("manipulator_node", anonymous=True)

        self.robot_name = rospy.get_param("robot_name")

        self.response_publisher = rospy.Publisher("manipulator/response", String, queue_size=10)
        rospy.Subscriber("manipulator/command", String, self.command_callback)

        self.stm_publisher = rospy.Publisher("stm/command", String, queue_size=10)
        rospy.Subscriber("stm/response", String, self.response_callback)

        self.responses = {}

        self.last_response_id = None
        self.last_response_args = None
        self.id_command = 1
        # rospy.sleep(2)

    def parse_data(self, data):
        data = data.data.split()
        cmd_id = data[0]
        cmd = data[1]
        return cmd_id, cmd

    def make_command(self, cmd):
        if cmd == "default":
            self.calibrate()

        elif cmd == "manipulator_wall":
            self.set_manipulator_wall()
        elif cmd == "manipulator_up":
            self.set_manipulator_up()
        elif cmd == "start_collect_ground":
            self.start_collect_ground()
        elif cmd == "complete_collect_ground":
            self.complete_collect_ground()
        elif cmd == "start_collect_wall":
            self.start_collect_wall()
        elif cmd == "complete_collect_wall":
            self.complete_collect_wall()
        elif cmd == "release_5":
            self.release(5)
        elif cmd == "release_accelerator":
            self.release_accelerator()
        elif cmd == "start_collect_blunium":
            self.start_collect_blunium()
        elif cmd == "grab_goldenium_and_hold_up":
            self.goldenium_up_and_hold()
        elif cmd == "release_goldenium_on_scales":
            self.release_goldenium_on_scales()
        elif cmd == "only_pump_up":
            self.only_pump_up()
        elif cmd == "set_angle_to_grab_goldenium":
            self.set_angle_to_grab_goldenium()
        elif cmd == "stepper_step_up":
            self.stepper_step_up()
        return True

    def command_callback(self, data):
        cmd_id, cmd = self.parse_data(data)
        result = self.make_command(cmd)
        if result:
            self.response_publisher.publish(cmd_id + " success")
        else:
            self.response_publisher.publish(cmd_id + " failed")

    def response_callback(self, data):
        response = data.data.split()
        print("RESPONSE=", response)
        if re.match(r"manipulator-\d", response[0]):
            self.responses[response[0]] = response[1]

    def send_command(self, cmd, args=None):
        if args is None:
            message = "manipulator-" + str(self.id_command) + " " + str(cmd)
        else:
            message = "manipulator-" + str(self.id_command) + " " + str(cmd) + " " + str(args)
        while True:
            self.stm_publisher.publish(String(message))
            # Wait answer
            rospy.sleep(0.1)
            if ("manipulator-" + str(self.id_command)) in self.responses.keys():
                if self.responses[("manipulator-" + str(self.id_command))] == ResponseStatus.OK:
                    self.id_command += 1
                    return
                elif self.responses[("manipulator-" + str(self.id_command))] == ResponseStatus.ERROR:
                    self.id_command += 1
                else:
                    rospy.loginfo("Error in send_command()->manipulator.py")





    def calibrate(self):
        if self.robot_name == "main_robot":
            # start calibration
            self.send_command(48)
            # set pump to the platform
            self.send_command(21)
            # grabber throw (up)
            self.send_command(25)  # FIXME
            # make step down
            self.send_command(50)
            return True
        elif self.robot_name == "secondary_robot":
            self.send_command(Protocol.START_CALIBRATION)
            self.send_command(Protocol.MAKE_STEP_DOWN)
            self.send_command(Protocol.PROP_PUCK_GRABBER)
            self.send_command(Protocol.SET_PLATFORM)
            self.send_command(Protocol.RELEASER_DEFAULT_SECONDARY)
            return True

    def start_collect_ground(self):
        # Release grabber
        self.send_command(22)
        # Set pump to the ground
        self.send_command(19)
        # Start pump
        self.send_command(17)
        # Set pump to the wall
        self.send_command(20)
        return True

    def set_manipulator_wall(self):
        self.send_command(20)

    # FIXME  why 24 cmd?

    def set_manipulator_up(self):
        self.send_command(24)
        self.send_command(21)

    def complete_collect_ground(self):
        # Set pump to the platform
        self.send_command(21)
        # Prop pack
        self.send_command(23)
        # Stop pump
        self.send_command(18)
        # Grab pack
        self.send_command(24)
        # Release grabber
        self.send_command(22)

        # Set pump to the wall
        # self.send_command(20)

        # make step down
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
        # get step motor status
        self.send_command(52)
        # grabber throw (up)
        self.send_command(25)
        # release grabber
        self.send_command(22)
        return True

    def start_collect_blunium(self):
        # Release grabber
        self.send_command(22)
        # push blunium angle
        self.send_command(34)
        # Start pump
        self.send_command(17)
        return True

    def goldenium_up_and_hold(self):
        # release grabber
        self.send_command(22)
        # set angle to grab goldenium
        self.send_command(35)
        # start pump
        self.send_command(17)
        rospy.sleep(1)
        # lift up goldenium
        self.send_command(36)
        return True

    def set_angle_to_grab_goldenium(self):
        # set angle to grab goldenium
        self.send_command(35)

    def release_goldenium_on_scales(self):
        # set pump to the wall
        self.send_command(20)
        # stop pump
        self.send_command(18)
        # set pump to the platform
        self.send_command(21)
        return True

    def stepper_step_up(self):
        # make step up
        self.send_command(51)

    def only_pump_up(self):
        # set pump to the platform
        self.send_command(21)


if __name__ == '__main__':
    try:
        manipulator = Manipulator()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
