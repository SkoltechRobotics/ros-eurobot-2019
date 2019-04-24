import enum
import rospy
from std_msgs.msg import String

from termcolor import colored, cprint

import requests


class SideStatus(enum.Enum):
    YELLOW = 1
    PURPLE = 0


class BTController(object):
    def __init__(self, behavior_tree):
        self.side_status_subscriber = rospy.Subscriber("stm/side_status", String, self.side_status_callback)
        self.start_status_subscriber = rospy.Subscriber("stm/start_status", String, self.start_status_callback)

        self.start_counter = 0
        self.behavior_tree = behavior_tree

        self.experiment_init = False
        self.experiment_start = False

    def side_status_callback(self, data):
            if self.behavior_tree.side_status is None:
                if data.data == "1":
                    cprint ("UPDATE SIDE TO " + colored("YELLOW", "yellow", attrs=['bold', 'blink']))
                    self.behavior_tree.change_side(SideStatus.YELLOW)
                elif data.data == "0":
                    cprint ("UPDATE SIDE TO " + colored("PURPLE", "magenta", attrs=['bold', 'blink']))
                    self.behavior_tree.change_side(SideStatus.PURPLE)

            if data.data == "1" and self.behavior_tree.side_status == SideStatus.PURPLE:
                cprint ("UPDATE SIDE TO " + colored("YELLOW", "yellow", attrs=['bold', 'blink']))
                self.behavior_tree.change_side(SideStatus.YELLOW)
            if data.data == "0" and self.behavior_tree.side_status == SideStatus.YELLOW:
                cprint ("UPDATE SIDE TO " + colored("PURPLE", "magenta", attrs=['bold', 'blink']))
                self.behavior_tree.change_side(SideStatus.PURPLE)

    def start_status_callback(self, data):
        if not self.experiment_init:
            try:
                r = requests.get('http://192.168.88.220/init')
                if r.status_code == 200:
                    self.experiment_init = True
            except requests.exceptions.RequestException as e:
                pass

        if data.data == "1":
            self.behavior_tree.start()
            if self.experiment_init:
                for i in range(50):
                    r = requests.get('http://192.168.88.220/start')
                    if r.status_code == 200:
                        self.experiment_start = True

            self.start_status_subscriber.unregister()
            self.side_status_subscriber.unregister()
