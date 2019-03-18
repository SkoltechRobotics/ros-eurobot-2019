#!/usr/bin/env python
import rospy
import behavior_tree as bt
import bt_ros
import numpy as np
from std_msgs.msg import String

first_puck_zone = rospy.get_param("first_puck_zone")
second_puck_zone = rospy.get_param("second_puck_zone")
third_puck_zone = rospy.get_param("third_puck_zone")


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

        self.stm_publisher = rospy.Publisher("stm/command", String, queue_size=100)
        self.stm_client = bt_ros.ActionClient(self.stm_publisher)
        rospy.Subscriber("stm/response", String, self.manipulator_client.response_callback)

        rospy.sleep(2)


        start_status = bt_ros.isStartStatus("stm_client")


        default_state = bt.Latch(bt_ros.SetToDefaultState("manipulator_client"))

        # first
        move_1_puck = bt.Latch(bt_ros.MoveLineToPoint([0.12, 1.82, 1.57], "move_client"))
        set_man_wall = bt.Latch(bt_ros.SetManipulatortoWall("manipulator_client"))
        parallel0 = bt.ParallelNode([move_1_puck, set_man_wall], threshold=2)
        
        start_take_1_puck = bt.Latch(bt_ros.StartTakeWallPuck("manipulator_client"))
        complete_take_1_puck = bt.Latch(bt_ros.CompleteTakeWallPuck("manipulator_client"))
        move_1_back = bt.Latch(bt_ros.MoveLineToPoint([0.12, 1.75, 1.57], "move_client"))
        move_1_back1 = bt.Latch(bt_ros.MoveLineToPoint([0.22, 1.75, 1.57], "move_client"))
        
        parallel1 = bt.ParallelNode([bt.SequenceNode([move_1_back, move_1_back1]), complete_take_1_puck], threshold=2)
        

        # second
        move_2_puck = bt.Latch(bt_ros.MoveLineToPoint([0.21, 1.82, 1.57], "move_client"))
        start_take_2_puck = bt.Latch(bt_ros.StartTakeWallPuck("manipulator_client"))
        complete_take_2_puck = bt.Latch(bt_ros.CompleteTakeWallPuck("manipulator_client"))
        move_2_back = bt.Latch(bt_ros.MoveLineToPoint([0.21, 1.3, 1.57], "move_client"))
        move_2_back2 = bt.Latch(bt_ros.MoveLineToPoint([0.6, 1.3, 1.57], "move_client"))
        parallel2 = bt.ParallelNode([bt.SequenceNode([move_2_back,move_2_back2]), complete_take_2_puck], threshold=2)
        # third
        move_3_puck = bt.Latch(bt_ros.MoveLineToPoint([0.59, 1.34, 1.57], "move_client"))
        start_take_3_puck = bt.Latch(bt_ros.StartTakeWallPuck("manipulator_client"))
        complete_take_3_puck = bt.Latch(bt_ros.CompleteTakeWallPuck("manipulator_client"))
        move_3_back = bt.Latch(bt_ros.MoveLineToPoint([0.59, 1.3, 1.57], "move_client"))
        move_3_back3 = bt.Latch(bt_ros.MoveLineToPoint([0.79, 1.3, 1.57], "move_client"))
        parallel3 = bt.ParallelNode([bt.SequenceNode([move_3_back,move_3_back3]), complete_take_3_puck], threshold=2)
        # forth
        move_4_puck = bt.Latch(bt_ros.MoveLineToPoint([0.79, 1.34, 1.57], "move_client"))
        start_take_4_puck = bt.Latch(bt_ros.StartTakeWallPuck("manipulator_client"))
        complete_take_4_puck = bt.Latch(bt_ros.CompleteTakeWallPuck("manipulator_client"))
        move_4_back = bt.Latch(bt_ros.MoveLineToPoint([0.79, 1.3, 1.57], "move_client"))
        move_4_back4 = bt.Latch(bt_ros.MoveLineToPoint([1, 1.3, 1.57], "move_client"))
        parallel4 = bt.ParallelNode([bt.SequenceNode([move_4_back,move_4_back4]),complete_take_4_puck], threshold=2)
        # fivth
        move_5_puck = bt.Latch(bt_ros.MoveLineToPoint([0.99, 1.34, 1.57], "move_client"))
        start_take_5_puck = bt.Latch(bt_ros.StartTakeWallPuck("manipulator_client"))
        complete_take_5_puck = bt.Latch(bt_ros.CompleteTakeWallPuck("manipulator_client"))
        move_5_back = bt.Latch(bt_ros.MoveLineToPoint([0.23, 0.44, 0], "move_client"))
        set_man_up = bt.Latch(bt_ros.SetManipulatortoUp("manipulator_client"))
        # parallel5 = bt.ParallelNode([move_5_back, set_man_up], threshold=2)
        # Scales
        move_scales1 = bt.Latch(bt_ros.MoveLineToPoint([1.34, 1.34, 1.57], "move_client"))
        move_scales2 = bt.Latch(bt_ros.MoveLineToPoint([1.34, 1.44, 1.57], "move_client"))
        move_back = bt.Latch(bt_ros.MoveLineToPoint([0.23, 0.44, 0], "move_client"))  

        first_puck = bt.SequenceNode([parallel0, start_take_1_puck, parallel1])

        second_puck = bt.SequenceNode([move_2_puck, start_take_2_puck, parallel2])

        third_puck = bt.SequenceNode([move_3_puck, start_take_3_puck, parallel3])

        forth_puck = bt.SequenceNode([move_4_puck, start_take_4_puck, parallel4])

        fivth_puck = bt.SequenceNode([move_5_puck, start_take_5_puck, complete_take_5_puck, set_man_up])

        scales = bt.SequenceNode([move_scales1, move_scales2, move_back])

        pucks = bt.SequenceNode([first_puck, second_puck, third_puck, forth_puck, fivth_puck, scales])

        leaf = bt.SequenceNode([start_status, default_state, pucks])

        self.bt = bt.Root(leaf, action_clients={"move_client": self.move_client, "manipulator_client": self.manipulator_client})

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