#!/usr/bin/env python
# coding: utf-8

# simple go to goal in different methods: odom movement

import rospy
import numpy as np
import tf2_ros
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from threading import Lock
from std_msgs.msg import Int32MultiArray
from visualization_msgs.msg import MarkerArray


class ManipulatorNode:
    def __init__(self):

        # FIXME
        # rospy.init_node("motion_planner", anonymous=True)
        rospy.init_node('ManipulatorNode')

        self.mutex = Lock()

        self.number_of_atoms = 17
        self.atoms_coordinates = np.zeros([17, 3])
        self.atom_grasped = False
        self.arm_ready = False
        self.atoms_placed = 0
        self.flag = False

        # FIXME
        # queue_size=1 ???????
        self.move_command = rospy.Publisher('move_command', String, queue_size=10)
        self.pub_cmd = rospy.Publisher('/secondary_robot/stm_command', String)

        rospy.Subscriber("pucks_position", MarkerArray, self.pucks_marker_callback)
        rospy.sleep(2)
        self.response = rospy.Subscriber('response', String, self.callback_response, queue_size=10)
        rate = rospy.Rate(20)

    def callback_response(data):
        if data.data == 'finished':
            global flag
            flag = True
            #print flag

    def pucks_marker_callback(self):
        """
        С камеры приходят координаты атомов
        4 из зоны хаоса
        3 periodic table
        6 на стене
        3 на стене
        1 на подъезде
        Need to create a dictionary
        For ex chaos zone:
        r1 -- []
        r2 -- []
        g3 -- []
        g4 -- []

        :return:
        """
        self.mutex.acquire()

        # if cmd_type == "grab_atom" or cmd_type == "release atom":

        self.mutex.release()

        pass

    def grab_from_floor(self):
        pass

    def grab_from_wall(self):
        pass

    def place_atom(self):
        pass

    def execute(self, cmd):

        """
        Проверить, что робот остановился.
        Проверить, что цель достигнута.
        Подписаться на топик, в который будут публиковаться координаты атомов с камеры

        Проверить, что манипулятор в исходной позиции
        Опустить манипулятор
        Проверить, что манипулятор опустился
        Включить насос (на сколько?)
        Поднять манипулятор до конца
        Подпереть атом граблей
        Выключить насос


        :return:
        """

        m_cmd = str(cmd)

        rospy.loginfo("m_cmd:\t" + str(m_cmd))
        rospy.loginfo("Sending cmd: " + m_cmd)
        self.pub_cmd.publish(m_cmd)

        pass
