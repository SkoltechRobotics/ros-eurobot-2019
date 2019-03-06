#!/usr/bin/env python
import rospy
import behavior_tree as bt
import bt_ros
import numpy as np
from std_msgs.msg import String

first_puck_zone = rospy.get_param("first_puck")
second_puck_zone = rospy.get_param("second_puck")
third_puck_zone = rospy.get_param("third_puck")


class SecondaryRobotBT():
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


        default_state = bt.Latch(bt_ros.DefaultState("manipulator_client"))
        # first
        move_1_puck = bt.Latch(bt_ros.MoveToPoint([0.6, 1.34, 1.57], "move_client"))
        take_1_puck = bt.Latch(bt_ros.TakeWallPuck("manipulator_client"))
        move_1_back = bt.Latch(bt_ros.MoveToPoint([0.7, 1.3, 1.57], "move_client"))
        # second
        move_2_puck = bt.Latch(bt_ros.MoveToPoint([0.8, 1.34, 1.57], "move_client"))
        take_2_puck = bt.Latch(bt_ros.TakeWallPuck("manipulator_client"))
        move_2_back = bt.Latch(bt_ros.MoveToPoint([0.9, 1.3, 1.57], "move_client"))
        # third
        move_3_puck = bt.Latch(bt_ros.MoveToPoint([1, 1.34, 1.57], "move_client"))
        take_3_puck = bt.Latch(bt_ros.TakeWallPuck("manipulator_client"))

        self.bt = bt.Root(bt.FallbackNode([default_state,bt.SequenceNode([move_1_puck, take_1_puck, move_1_back],
                        [move_2_puck, take_2_puck, move_2_back],
                        [move_3_puck, take_3_puck])]),
                          action_clients={"move_client": self.move_client,
                                                             "manipulator_client": self.manipulator_client})




        # self.collect_1_puck = bt.SequenceNode([])


        # self.bt = bt.Root(bt.SequenceNode([default_state,
        #                                    move_1_puck, take_1_puck, move_1_back,
        #                                    move_2_puck, take_2_puck, move_2_back,
        #                                    move_3_puck, take_3_puck]),
        #                   action_clients={"move_client": self.move_client,
        #                                   "manipulator_client": self.manipulator_client})



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
        tree = SecondaryRobotBT()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass