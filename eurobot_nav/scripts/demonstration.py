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
    command = 'abc arc_move '
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

if __name__ == '__main__':
    rospy.init_node('Demonstration', anonymous=True)
    move_command = rospy.Publisher('move_command', String, queue_size=10)
    response = rospy.Subscriber('response', String, callback_response, queue_size=10)
    rate = rospy.Rate(20)
    perform_command(generate_command(1.5, 0.45, 3.14))
    perform_command(generate_command(0.61, 0.45, 3.14))
    perform_command(generate_command(0.5, 0.34, 1.57))
    perform_command(generate_command(0.5, 0.94, 3.14))
    perform_command(generate_command(0.4, 1.05, 3.14))