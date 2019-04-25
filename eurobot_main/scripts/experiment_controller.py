#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import requests

class ExperimentController(object):
    def __init__(self):
        self.start_status_subscriber = rospy.Subscriber("stm/start_status", String, self.start_status_callback)

        self.experiment_init = False
        self.experiment_start = False

        self.init_timer = rospy.Timer(rospy.Duration(1), self.timer_callback)

    def start_status_callback(self, data):
        if data.data == "1":
            if self.experiment_init:
                self.start_experiment()

    def timer_callback(self, event):
        if not self.experiment_init:
            self.init_experiment()

    def init_experiment(self):
        try:
            r = requests.get('http://192.168.88.220/init')
            if r.status_code == 200:
                self.experiment_init = True
        except requests.exceptions.RequestException as e:
            rospy.loginfo("Can't connect to the experiment!!!")

    def start_experiment(self):
        for i in range(50):
            try:
                r = requests.get('http://192.168.88.220/start')
                if r.status_code == 200:
                    self.experiment_start = True
                    return True
            except requests.exceptions.RequestException as e:
                rospy.loginfo("Can't connect to the experiment!!!")

if __name__ == '__main__':
    try:
        rospy.init_node("experiment_controller")
        experiment_controller = ExperimentController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
