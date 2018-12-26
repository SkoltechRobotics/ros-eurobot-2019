#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from core_functions import cvt_global2local
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

if __name__ == '__main__':
    rospy.init_node('Demonstration', anonymous=True)
    move_command = rospy.Publisher('move_command', String, queue_size=10)
    rospy.sleep(3)
    response = rospy.Subscriber('response', String, callback_response, queue_size=10)
    rate = rospy.Rate(20)
    point0 = np.array([0.5, 0.45, 0])
    
    def get_rot_point(angle):
        if angle == 0:
            return np.array([0.39, 0.45, 0])
        elif angle == 1:
            return np.array([0.5, 0.34, 1.57])
        elif angle == 2:
            return np.array([0.61, 0.45, 3.14])
        elif angle == 3:
            return np.array([0.5, 0.56, 4.71])

    for i in range(3, 13):
        i %= 4
        perform_command(generate_command(*get_rot_point(i)))
#     perform_command(generate_command(1.5, 0.45, 3.14))
#     perform_command(generate_command(0.61, 0.45, 3.14))
#     perform_command(generate_command(0.5, 0.34, 1.57))
#     perform_command(generate_command(0.5, 0.94, 1.57))
#     perform_command(generate_command(0.4, 1.05, 3.14))
