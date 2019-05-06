#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import requests

class ExperimentController(object):
    def __init__(self):
        self.start_status_subscriber = rospy.Subscriber("stm/start_status", String, self.start_status_callback)

        self.experiment_init = False
        self.experiment_start = False

    def start_status_callback(self, data):
        if not self.experiment_init:
            self.init_experiment()

        if data.data == "1":
            if self.experiment_init:
                self.start_experiment()

    def init_experiment(self):
        try:
            r = requests.get('http://192.168.88.220/init')
            if r.status_code == 200:
                self.experiment_init = True
        except requests.exceptions.RequestException as e:
            rospy.loginfo("Can't connect to the experiment!!!")

    def start_experiment(self):
        try:
            for i in range(50):
                r = requests.get('http://192.168.88.220/start')
                if r.status_code == 200:
                    self.experiment_start = True
        except requests.exceptions.RequestException as e:
            rospy.loginfo("Can't connect to the experiment!!!")

if __name__ == '__main__':
    try:
        rospy.init_node("experiment_controller")
        experiment_controller = ExperimentController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
