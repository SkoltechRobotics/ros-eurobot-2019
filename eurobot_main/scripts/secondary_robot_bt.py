#!/usr/bin/env python
import enum
import rospy
import behavior_tree as bt
import bt_ros
import numpy as np
from std_msgs.msg import String

side_statuses = {"yellow", "purple"}

class SideStatus(enum.Enum):
    RIGHT = 0
    LEFT = 1

class BTController():
    def __init__(self):
        rospy.Subscriber("stm/side_status", String, self.side_status_callback)

        self.bt = None
        self.side_status = None

    def side_status_callback(self, data):
        if self.side_status is None and data.data == "0":
            self.side_status = SideStatus.RIGHT
        if self.side_status is None and data.data == "1":
            self.side_status = SideStatus.LEFT

        if data.data == "0" and self.side_status == SideStatus.LEFT:
            self.side_status = SideStatus.RIGHT
            bt = SecondaryRobotBT(self.side_status)
        if data.data == "1" and self.side_status == SideStatus.RIGHT:
            self.side_status = SideStatus.LEFT
            bt = SecondaryRobotBT(self.side_status)


class SecondaryRobotBT():
    def __init__(self, side_status):
        self.robot_name = rospy.get_param("robot_name")

        self.move_publisher = rospy.Publisher("navigation/command", String, queue_size=100)
        self.move_client = bt_ros.ActionClient(self.move_publisher)
        rospy.Subscriber("navigation/response", String, self.move_client.response_callback)

        self.manipulator_publisher = rospy.Publisher("manipulator/command", String, queue_size=100)
        self.manipulator_client = bt_ros.ActionClient(self.manipulator_publisher)
        rospy.Subscriber("manipulator/response", String, self.manipulator_client.response_callback)

        rospy.Subscriber("stm/start_status", String, self.start_status_callback)


        self.start_status = False
        self.start_counter = 0
        self.side_status = side_status

        if self.side_status == SideStatus.RIGHT:
            first_puck = rospy.get_param("first_puck_zone_right")
            second_puck = rospy.get_param("second_puck_zone_right")
            third_puck = rospy.get_param("third_puck_zone_right")
            forth_puck = rospy.get_param("forth_puck_zone_right")
            fifth_puck = rospy.get_param("fifth_puck_zone_right")
            scales_zone = rospy.get_param("scales_zone_right")
            start_zone = rospy.get_param("start_zone_right")
        if self.side_status == SideStatus.LEFT:
            first_puck = rospy.get_param("first_puck_zone_left")
            second_puck = rospy.get_param("second_puck_zone_left")
            third_puck = rospy.get_param("third_puck_zone_left")
            forth_puck = rospy.get_param("forth_puck_zone_left")
            fifth_puck = rospy.get_param("fifth_puck_zone_left")
            scales_zone = rospy.get_param("scales_zone_left")
            start_zone = rospy.get_param("start_zone_left")

        print ("SIDE=", side_status)

        rospy.sleep(2)

        start_node = bt.ConditionNode(self.is_start)
        default_state = bt.Latch(bt_ros.SetToDefaultState("manipulator_client"))

        # first
        move_1_puck = bt.Latch(bt_ros.MoveLineToPoint(first_puck, "move_client"))
        set_man_wall = bt.Latch(bt_ros.SetManipulatortoWall("manipulator_client"))
        parallel0 = bt.ParallelNode([move_1_puck, set_man_wall], threshold=2)
        


        start_take_1_puck = bt.Latch(bt_ros.StartTakeWallPuck("manipulator_client"))
        complete_take_1_puck = bt.Latch(bt_ros.CompleteTakeWallPuck("manipulator_client"))
        x = first_puck[1] -0.07
        first_puck[1] = x
        move_1_back = bt.Latch(bt_ros.MoveLineToPoint(first_puck, "move_client"))
        if self.side_status == SideStatus.LEFT:
            x = first_puck[0] + 0.1
            first_puck[0] = x
        elif self.side_status == SideStatus.RIGHT:
            x = first_puck[0] - 0.1
            first_puck[0] = x
        move_1_back1 = bt.Latch(bt_ros.MoveLineToPoint(first_puck, "move_client"))
    
        asf = bt.SequenceNode([move_1_back, move_1_back1])

        parallel1 = bt.ParallelNode([ asf , complete_take_1_puck ], threshold=2)
    
        # second
        move_2_puck = bt.Latch(bt_ros.MoveLineToPoint(second_puck, "move_client"))
        start_take_2_puck = bt.Latch(bt_ros.StartTakeWallPuck("manipulator_client"))
        complete_take_2_puck = bt.Latch(bt_ros.CompleteTakeWallPuck("manipulator_client"))
        x = second_puck[1] -0.5
        second_puck[1] = x
        move_2_back = bt.Latch(bt_ros.MoveLineToPoint(second_puck, "move_client"))
        if self.side_status == SideStatus.LEFT:
            x = second_puck[0] + 0.39
            second_puck[0] = x
        elif self.side_status == SideStatus.RIGHT:  
            x = second_puck[0] - 0.39
            second_puck[0] = x
        move_2_back2 = bt.Latch(bt_ros.MoveLineToPoint(second_puck, "move_client"))
        parallel2 = bt.ParallelNode([bt.SequenceNode([move_2_back,move_2_back2]), complete_take_2_puck], threshold=2)
        # third
        move_3_puck = bt.Latch(bt_ros.MoveLineToPoint(third_puck, "move_client"))
        start_take_3_puck = bt.Latch(bt_ros.StartTakeWallPuck("manipulator_client"))
        complete_take_3_puck = bt.Latch(bt_ros.CompleteTakeWallPuck("manipulator_client"))
        x = third_puck[1] - 0.04
        third_puck[1] = x
        move_3_back = bt.Latch(bt_ros.MoveLineToPoint(third_puck, "move_client"))
        if self.side_status == SideStatus.LEFT:
            x = third_puck[0] + 0.2
            third_puck[0] = x
        elif self.side_status == SideStatus.RIGHT:
            x = third_puck[0] - 0.2
            third_puck[0] = x
        move_3_back3 = bt.Latch(bt_ros.MoveLineToPoint(third_puck, "move_client"))
        parallel3 = bt.ParallelNode([bt.SequenceNode([move_3_back,move_3_back3]), complete_take_3_puck], threshold=2)
        # forth
        move_4_puck = bt.Latch(bt_ros.MoveLineToPoint(forth_puck, "move_client"))
        start_take_4_puck = bt.Latch(bt_ros.StartTakeWallPuck("manipulator_client"))
        complete_take_4_puck = bt.Latch(bt_ros.CompleteTakeWallPuck("manipulator_client"))
        x = forth_puck[1] - 0.04
        forth_puck[1] = x
        move_4_back = bt.Latch(bt_ros.MoveLineToPoint(forth_puck, "move_client"))
        if self.side_status == SideStatus.LEFT:
            x = forth_puck[0] + 0.21
            forth_puck[0] = x
        elif self.side_status == SideStatus.RIGHT:
            x = forth_puck[0] - 0.2
            forth_puck[0] = x
        move_4_back4 = bt.Latch(bt_ros.MoveLineToPoint(forth_puck, "move_client"))
        parallel4 = bt.ParallelNode([bt.SequenceNode([move_4_back,move_4_back4]),complete_take_4_puck], threshold=2)
        # fivth
        move_5_puck = bt.Latch(bt_ros.MoveLineToPoint(fifth_puck, "move_client"))
        start_take_5_puck = bt.Latch(bt_ros.StartTakeWallPuck("manipulator_client"))
        complete_take_5_puck = bt.Latch(bt_ros.CompleteTakeWallPuck("manipulator_client"))
        set_man_up = bt.Latch(bt_ros.SetManipulatortoUp("manipulator_client"))
        # parallel5 = bt.ParallelNode([move_5_back, set_man_up], threshold=2)
        # Scales
        move_scales1 = bt.Latch(bt_ros.MoveLineToPoint(scales_zone, "move_client"))
        x = scales_zone[1] + 0.14
        scales_zone[1] = x
        move_scales2 = bt.Latch(bt_ros.MoveLineToPoint(scales_zone, "move_client"))
        move_back = bt.Latch(bt_ros.MoveLineToPoint(start_zone, "move_client"))  

        release = bt.Latch(bt_ros.ReleaseFivePucks("manipulator_client"))


        first_puck = bt.SequenceNode([parallel0, start_take_1_puck, parallel1])

        second_puck = bt.SequenceNode([move_2_puck, start_take_2_puck, parallel2])

        third_puck = bt.SequenceNode([move_3_puck, start_take_3_puck, parallel3])

        forth_puck = bt.SequenceNode([move_4_puck, start_take_4_puck, parallel4])

        fivth_puck = bt.SequenceNode([move_5_puck, start_take_5_puck, complete_take_5_puck, set_man_up])

        scales = bt.SequenceNode([move_scales1, move_scales2, release, move_back])

        pucks = bt.SequenceNode([first_puck, second_puck, third_puck, forth_puck, fivth_puck, scales])

        leaf = bt.SequenceNode([default_state, pucks])

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

    # FIXME::HOW TO TURN OFF THE CALLBACK?        
    def start_status_callback(self, data):
        if data.data == "1":
            self.start_counter += 1
        if self.start_counter == 5:
            self.start_status = True

    def is_start(self):
        if self.start_status==True:
            return bt.Status.SUCCESS
        elif self.start_status==False:
            return bt.Status.RUNNING



if __name__ == '__main__':
    try:
        rospy.init_node("secondary_robot_BT_controller")
        bt_controller = BTController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass