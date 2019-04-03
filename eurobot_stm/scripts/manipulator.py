#!/usr/bin/env python
import re
import enum

import rospy
from std_msgs.msg import String


class ResponseAnswer(enum.Enum):
    OK = "OK"
    ERROR = "ER"


class ResponseStatus(enum.Enum):
    FAIL = "0"
    SUCCESS = "1"


class Manipulator(object):
    def __init__(self):
        rospy.init_node("manipulator_node", anonymous=True)

        self.robot_name = rospy.get_param("robot_name")

        self.response_publisher = rospy.Publisher("manipulator/response", String, queue_size=10)
        rospy.Subscriber("manipulator/command", String, self.command_callback)

        self.stm_publisher = rospy.Publisher("stm/command", String, queue_size=10)
        rospy.Subscriber("stm/response", String, self.response_callback)

        self.responses = {}
        self.id_command = 1
        self.status_command = 1

        self.protocol = {
            "SET_ANGLE" : 0x10,
            "START_PUMP" : 0x11,
            "STOP_PUMP" : 0x12,
            "SET_GROUND" : 0x13,
            "SET_WALL" : 0x14,
            "SET_PLATFORM" : 0x15,
            "OPEN_GRABBER" : 0x16,
            "PROP_PUCK_GRABBER" : 0x17,
            "GRAB_PUCK_GRABBER" : 0x18,
    
            # only for Secondary
            "RELEASER_DEFAULT_SECONDARY" : 0x19,
            "RELEASER_THROW_SECONDARY" : 0x20,
    
            # only for Main
            "UNLOAD_PUCK_TOP_MAIN" : 0x21,
            "UNLOAD_DEFAULT_MAIN" : 0x22,  # FIXME
            "UNLOAD_PUCK_BOTTOM_MAIN" : 0x23,
            "SET_BLUNIUM_ANGLE_MAIN" : 0x24,
            "SET_GRAB_GOLDENIUM_ANGLE_MAIN" : 0x25,
            "SET_LIFT_GOLDENIUM_ANGLE_MAIN" : 0x26,

            # for both Main and Secondary robots
            "GET_PACK_PUMPED_STATUS" : 0x27,
            "START_CALIBRATION" : 0x30,
            "MAKE_STEP" : 0x31,
            "MAKE_STEP_DOWN" : 0x32,
            "MAKE_STEP_UP" : 0x33,
            "GET_STEP_MOTOR_STATUS" : 0x34,
            "GET_PROXIMITY_STATUS" : 0x40
        }

    def parse_data(self, data):
        data = data.data.split()
        cmd_id = data[0]
        cmd = data[1]
        return cmd_id, cmd

    def do_command(self, cmd):
        if cmd == "default":
            return self.calibrate()
        elif cmd == "manipulator_wall":
            return self.set_manipulator_wall()
        elif cmd == "manipulator_up":
            return self.set_manipulator_platform()
        elif cmd == "manipulator_ground":
            return self.set_manipulator_ground()
        elif cmd == "manipulator_ground_delay":
            return self.set_manipulator_ground_delay()
        elif cmd == "stop_pump":
            return self.stop_pump()
        elif cmd == "start_collect_ground":
            return self.start_collect_ground()
        elif cmd == "release_from_manipulator":
            return self.release_from_manipulator()
        elif cmd == "complete_collect_ground":
            return self.complete_collect_ground()
        elif cmd == "start_collect_wall":
            return self.start_collect_wall()
        elif cmd == "start_collect_wall_without_grabber":
            return self.start_collect_wall_without_grabber()
        elif cmd == "complete_collect_wall":
            return self.complete_collect_wall()
        elif cmd == "complete_collect_last_wall":
            return self.complete_collect_last_wall()
        elif cmd == "release_5":
            return self.release(5)
        elif cmd == "release_accelerator":
            return self.release_accelerator()
        elif cmd == "start_collect_blunium":
            return self.start_collect_blunium()
        elif cmd == "grab_goldenium_and_hold_up":
            return self.goldenium_up_and_hold()
        elif cmd == "release_goldenium_on_scales":
            return self.release_goldenium_on_scales()
        elif cmd == "set_angle_to_grab_goldenium":
            return self.set_angle_to_grab_goldenium()
        elif cmd == "stepper_step_up":
            return self.stepper_step_up()

    def command_callback(self, data):
        cmd_id, cmd = self.parse_data(data)
        result = self.do_command(cmd)
        if result:
            self.response_publisher.publish(cmd_id + " success")
        else:
            self.response_publisher.publish(cmd_id + " failed")

    def response_callback(self, data):
        response = data.data.split()
        if re.match(r"manipulator", response[0]):
            print ("RESPONSE", response)
            self.responses[response[0]] = response[1]

    def is_okay_answer(self, id_command):
        while True:
            if ("manipulator-" + str(id_command)) in self.responses.keys():
                if self.responses[("manipulator-" + str(id_command))] == ResponseAnswer.OK.value:
                    self.id_command += 1
                    return True
                elif self.responses[("manipulator-" + str(id_command))] == ResponseAnswer.ERROR.value:
                    self.id_command += 1
                    # rospy.sleep(0.1)
                    return False
                else:
                    rospy.loginfo("Error in send_command()->manipulator.py->is_okay_answer")

    def send_command(self, cmd, args=None):
        while True:
            if args is None:
                message = "manipulator-" + str(self.id_command) + " " + str(cmd)
            else:
                message = "manipulator-" + str(self.id_command) + " " + str(cmd) + " " + str(args)
            self.stm_publisher.publish(String(message))
            if self.is_okay_answer(self.id_command):
                return True
                

    def is_success_status(self, id_command):
        rospy.loginfo("is_success_status")
        while True:
            if ("manipulator_status-" + str(id_command)) in self.responses.keys():
                if self.responses[("manipulator_status-" + str(id_command))] == ResponseStatus.SUCCESS.value:
                    rospy.loginfo("SUCCESS")
                    self.status_command += 1
                    return True
                elif self.responses[("manipulator_status-" + str(id_command))] == ResponseStatus.FAIL.value:
                    self.status_command += 1
                    rospy.loginfo("FAIL")
                    # rospy.sleep(0.1)
                    return False
                else:
                    rospy.loginfo("Error in send_command()->manipulator.py->is_success_status")

    def check_status(self, cmd):
        rospy.sleep(0.5)
        rospy.loginfo("check_status")
        self.stm_publisher.publish(String("manipulator_status-" + str(self.status_command) + " " + str(cmd)))
        rospy.loginfo("check_status")
        return self.is_success_status(self.status_command)

    def calibrate(self):
        if self.robot_name == "main_robot":  # FIXME
            self.send_command(self.protocol["START_CALIBRATION"])
            self.send_command(self.protocol["SET_PLATFORM"])
            self.send_command(self.protocol["UNLOAD_PUCK_TOP_MAIN"])
            self.send_command(self.protocol["MAKE_STEP_DOWN"])
            return True
        elif self.robot_name == "secondary_robot":
            self.send_command(self.protocol["START_CALIBRATION"])
            self.send_command(self.protocol["MAKE_STEP_DOWN"])
            self.send_command(self.protocol["PROP_PUCK_GRABBER"])
            self.send_command(self.protocol["SET_PLATFORM"])
            self.send_command(self.protocol["RELEASER_DEFAULT_SECONDARY"])
            return True

    def start_collect_ground(self):
        self.send_command(self.protocol["OPEN_GRABBER"])
        self.send_command(self.protocol["SET_GROUND"])
        self.send_command(self.protocol["START_PUMP"])
        self.send_command(self.protocol["SET_WALL"])  # FIXME remove this step?
        return True

    def complete_collect_ground(self):
        self.send_command(self.protocol["SET_PLATFORM"])
        self.send_command(self.protocol["PROP_PUCK_GRABBER"])
        self.send_command(self.protocol["STOP_PUMP"])
        self.send_command(self.protocol["GRAB_PUCK_GRABBER"])
        self.send_command(self.protocol["OPEN_GRABBER"])
        self.send_command(self.protocol["MAKE_STEP_DOWN"])
        return True

    def set_manipulator_ground(self):
        self.send_command(self.protocol["SET_GROUND"])
        return True

    def set_manipulator_ground_delay(self):
        rospy.sleep(0.5)
        self.send_command(self.protocol["SET_GROUND"])
        return True

    def set_manipulator_wall(self):
        self.send_command(self.protocol["SET_WALL"])
        return True

    def set_manipulator_platform(self):
        self.send_command(self.protocol["SET_PLATFORM"])
        return True
    def stop_pump(self):
        self.send_command(self.protocol["STOP_PUMP"])
        return True

    def start_collect_wall(self):
        if self.robot_name == "main_robot":
            pass
        if self.robot_name == "secondary_robot":
            self.send_command(self.protocol["OPEN_GRABBER"])
            self.send_command(self.protocol["SET_WALL"])
            self.send_command(self.protocol["START_PUMP"])
            result = self.check_status(self.protocol["GET_PACK_PUMPED_STATUS"])
            if result:
                return True
            else:
                return False

    def start_collect_wall_without_grabber(self):
        if self.robot_name == "main_robot":
            pass
        if self.robot_name == "secondary_robot":
            self.send_command(self.protocol["SET_WALL"])
            self.send_command(self.protocol["START_PUMP"])
            return True

    def complete_collect_wall(self):
        if self.robot_name == "main_robot":
            pass
        if self.robot_name == "secondary_robot":
            self.send_command(self.protocol["SET_PLATFORM"])
            self.send_command(self.protocol["PROP_PUCK_GRABBER"])
            self.send_command(self.protocol["STOP_PUMP"])
            self.send_command(self.protocol["SET_WALL"])
            self.send_command(self.protocol["GRAB_PUCK_GRABBER"])
            self.send_command(self.protocol["OPEN_GRABBER"])
            self.send_command(self.protocol["MAKE_STEP_DOWN"])
            return True

    def complete_collect_last_wall(self):
        if self.robot_name == "main_robot":
            pass
        if self.robot_name == "secondary_robot":
            self.send_command(self.protocol["SET_PLATFORM"])
            self.send_command(self.protocol["PROP_PUCK_GRABBER"])
            self.send_command(self.protocol["STOP_PUMP"])
            self.send_command(self.protocol["GRAB_PUCK_GRABBER"])
            self.send_command(self.protocol["PROP_PUCK_GRABBER"])
            self.send_command(self.protocol["MAKE_STEP_DOWN"])
            return True


    def release_from_manipulator(self):
        # self.send_command(self.protocol["SET_GROUND"])
        self.send_command(self.protocol["STOP_PUMP"])
        rospy.sleep(0.3)
        self.send_command(self.protocol["SET_PLATFORM"])

    def release(self, pucks_number):

        self.send_command(self.protocol["RELEASER_DEFAULT_SECONDARY"])

        self.send_command(self.protocol["MAKE_STEP_UP"])
        self.send_command(self.protocol["GET_STEP_MOTOR_STATUS"])
        self.send_command(self.protocol["MAKE_STEP_UP"])
        self.send_command(self.protocol["GET_STEP_MOTOR_STATUS"])
        self.send_command(self.protocol["RELEASER_THROW_SECONDARY"])
        self.send_command(self.protocol["RELEASER_DEFAULT_SECONDARY"])
        
        self.send_command(self.protocol["MAKE_STEP_UP"])
        self.send_command(self.protocol["GET_STEP_MOTOR_STATUS"])
        self.send_command(self.protocol["RELEASER_THROW_SECONDARY"])
        self.send_command(self.protocol["RELEASER_DEFAULT_SECONDARY"])

        self.send_command(self.protocol["MAKE_STEP_UP"])
        self.send_command(self.protocol["GET_STEP_MOTOR_STATUS"])
        self.send_command(self.protocol["RELEASER_THROW_SECONDARY"])
        self.send_command(self.protocol["RELEASER_DEFAULT_SECONDARY"])

        self.send_command(self.protocol["MAKE_STEP_UP"])
        self.send_command(self.protocol["GET_STEP_MOTOR_STATUS"])
        self.send_command(self.protocol["RELEASER_THROW_SECONDARY"])
        self.send_command(self.protocol["RELEASER_DEFAULT_SECONDARY"])

        self.send_command(self.protocol["MAKE_STEP_UP"])
        self.send_command(self.protocol["GET_STEP_MOTOR_STATUS"])
        self.send_command(self.protocol["RELEASER_THROW_SECONDARY"])
        self.send_command(self.protocol["RELEASER_DEFAULT_SECONDARY"])
        return True

    

    def release_accelerator(self):
        # assume that we need to move pucks 1 level up to start throwing them
        self.send_command(self.protocol["OPEN_GRABBER"])
        self.send_command(self.protocol["MAKE_STEP_UP"])
        self.send_command(self.protocol["GET_STEP_MOTOR_STATUS"])
        self.send_command(self.protocol["UNLOAD_PUCK_TOP_MAIN"])
        self.send_command(self.protocol["OPEN_GRABBER"])
        return True

    def start_collect_blunium(self):
        self.send_command(self.protocol["OPEN_GRABBER"])
        self.send_command(self.protocol["SET_BLUNIUM_ANGLE_MAIN"])
        self.send_command(self.protocol["START_PUMP"])
        return True

    def goldenium_up_and_hold(self):
        self.send_command(self.protocol["OPEN_GRABBER"])
        self.send_command(self.protocol["SET_GRAB_GOLDENIUM_ANGLE_MAIN"])
        self.send_command(self.protocol["START_PUMP"])
        rospy.sleep(1)
        self.send_command(self.protocol["SET_LIFT_GOLDENIUM_ANGLE_MAIN"])
        return True

    def set_angle_to_grab_goldenium(self):
        self.send_command(self.protocol["SET_GRAB_GOLDENIUM_ANGLE_MAIN"])
        return True

    def release_goldenium_on_scales(self):
        self.send_command(self.protocol["SET_WALL"])
        self.send_command(self.protocol["STOP_PUMP"])
        self.send_command(self.protocol["SET_PLATFORM"])
        return True

    def stepper_step_up(self):
        self.send_command(self.protocol["MAKE_STEP_UP"])
        return True


if __name__ == '__main__':
    try:
        manipulator = Manipulator()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
