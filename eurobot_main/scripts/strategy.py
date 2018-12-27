#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from visualization_msgs.msg import MarkerArray
import numpy as np

flag = False


def callback_response(data):
    if data.data == 'finished':
        global flag
        flag = True
        #print flag


def generate_command(x, y, theta):
    command = 'abc move_arc '
    command += str(x)
    command += ' '
    command += str(y)
    command += ' '
    command += str(theta)
    rospy.loginfo('--------NEW COMMAND--------')
    rospy.loginfo(command)
    return command


def perform_command(command):
    global flag
    flag = False
    move_command.publish(command)
    while not flag and not rospy.is_shutdown():
        rate.sleep()
        print flag


def perform_stm_command(command, wait_time):
    if rospy.is_shutdown():
        return
    stm_command_publisher.publish(command)
    rospy.sleep(wait_time)


def move_to_vertical_puck(puck):
    # perform_command(generate_command(0.5 + puck * 0.1, 1, 1.57))    
    perform_command(generate_command(0.49 + puck * 0.1, 1.298, 1.57))
    perform_stm_command("8 0.3 0 0.5", 0.3)
    perform_stm_command("8 0 0 0", 0.01)
    perform_stm_command("8 -0.3 0 -0.5", 0.3)
    perform_stm_command("8 0 0 0", 0.01)   
    # perform_command(generate_command(0.49 + puck * 0.1, 1.35, 1.57))
    # perform_command(generate_command(0.49 + puck * 0.1, 1.298, 1.57))


def move_to_hill():
    perform_command(generate_command(0.23, 1.298, 1.57))
    perform_command(generate_command(0.23, 1.75, 1.57))
    perform_command(generate_command(0.23, 1.83, 0))

    perform_stm_command("8 0.15 0 0", 5)
    perform_stm_command("8 0 0 0", 0.01)

    perform_stm_command("8 -0.15 0 0", 5)
    perform_stm_command("8 0 0 0", 0.01)

    perform_command(generate_command(0.23, 1.83, 0))
    perform_command(generate_command(0.23, 1.75, 1.57))
    perform_command(generate_command(0.23, 1.298, 1.57))


def perform_move_cmd(x, y, angle):
    perform_command(generate_command(x, y, angle))


def move_to_puck(puck):
    perform_move_cmd(1, 0.5, 3.14)
    perform_move_cmd(1, 0.5, 1.57)
    perform_move_cmd(puck[0], puck[1] - 0.11, 1.57)
    perform_move_cmd(puck[0] + 0.11, puck[1], 3.14)
    perform_move_cmd(0.4, 1.05, 3.14)


pucks = []


def pucks_marker_callback(data):
    global pucks
    pucks = [(x_.pose.position.x, x_.pose.position.y) for x_ in data]
    pucks = np.array(pucks)


if __name__ == '__main__':
    rospy.init_node('Demonstration', anonymous=True)
    move_command = rospy.Publisher('move_command', String, queue_size=10)
    stm_command_publisher = rospy.Publisher("secondary_robot/stm_command", String, queue_size=10)
    rospy.Subscriber("pucks_position", MarkerArray, pucks_marker_callback)
    rospy.sleep(2)
    response = rospy.Subscriber('response', String, callback_response, queue_size=10)
    rate = rospy.Rate(20)
    # perform_stm_command("14 0 0 0", 0.05)
    # perform_stm_command("8 0.6 0 0", 1)
    # perform_stm_command("8 0 0 0", 0.01)
   
    # for i in range(6):
    #     move_to_vertical_puck(i)
    # move_to_hill()

    while not rospy.is_shutdown():
        if len(pucks) > 0:
            move_to_puck(pucks[0])
        rospy.sleep(0.1)
    # perform_command(generate_command(1.5, 0.45, 3.14))
    # perform_command(generate_command(0.61, 0.45, 3.14))
    # perform_command(generate_command(0.5, 0.34, 1.57))
    # perform_command(generate_command(0.5, 0.94, 1.57))
    # perform_command(generate_command(0.4, 1.05, 3.14))

