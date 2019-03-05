#!/usr/bin/env python
import rospy
import behavior_tree as bt
import bt_ros
import numpy as np
from std_msgs.msg import String


class FirstIterBT():
    def __init__(self):
        rospy.init_node("test_first_iter")

        self.robot_name = rospy.get_param("robot_name")

        self.move_publisher = rospy.Publisher("navigation/command", String, queue_size=100)
        self.move_client = bt_ros.ActionClient(self.move_publisher)
        rospy.Subscriber("navigation/response", String, self.move_client.response_callback)


        self.manipulator_publisher = rospy.Publisher("manipulator/command", String, queue_size=100)
        self.manipulator_client = bt_ros.ActionClient(self.manipulator_publisher)
        rospy.Subscriber("manipulator/response", String, self.manipulator_client.response_callback)

        rospy.sleep(2)

        move = bt.Latch(bt_ros.MoveToPoint([1.0, 1.0, 0], "move_client") )
        take_puck = bt.Latch( bt_ros.TakeWallPuck("manipulator_client") )

        self.bt = bt.Root(bt.SequenceNode([move, take_puck]),
                          action_clients={"move_client": self.move_client,
                                          "manipulator_client": self.manipulator_client})
        self.bt_timer = rospy.Timer(rospy.Duration(0.1), self.timer_callback)

    def timer_callback(self, event):
        status = self.bt.tick()
        if status != bt.Status.RUNNING:
            self.bt_timer.shutdown()
        print("============== BT LOG ================")
        self.bt.log(0)

    def is_1_puck_close(self):
        pass

    def go_to_1st_puck(self):
        pass


if __name__ == '__main__':
    try:
        tree = FirstIterBT()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass