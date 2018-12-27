#!/usr/bin/env python
import rospy
from std_msgs.msg import String


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


if __name__ == '__main__':
    rospy.init_node('Demonstration', anonymous=True)
    move_command = rospy.Publisher('move_command', String, queue_size=10)
    stm_command_publisher = rospy.Publisher("secondary_robot/stm_command", String, queue_size=10)
    rospy.sleep(2)
    response = rospy.Subscriber('response', String, callback_response, queue_size=10)
    rate = rospy.Rate(20)
    # perform_stm_command("14 0 0 0", 0.05)
    # perform_stm_command("8 0.6 0 0", 1)
    # perform_stm_command("8 0 0 0", 0.01)
   
    # for i in range(6):
    #     move_to_vertical_puck(i)
    move_to_hill()

    # perform_command(generate_command(1.5, 0.45, 3.14))
    # perform_command(generate_command(0.61, 0.45, 3.14))
    # perform_command(generate_command(0.5, 0.34, 1.57))
    # perform_command(generate_command(0.5, 0.94, 1.57))
    # perform_command(generate_command(0.4, 1.05, 3.14))
