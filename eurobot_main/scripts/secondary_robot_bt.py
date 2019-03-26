#!/usr/bin/env python
import numpy as np

import rospy
import tf2_ros
from tf.transformations import euler_from_quaternion

import bt_ros
from std_msgs.msg import String

import behavior_tree as bt
from bt_controller import SideStatus, BTController

class YellowTactics(object):
    def __init__(self):
        self.first_puck = rospy.get_param("yellow_side/first_puck_zone")
        self.second_puck = rospy.get_param("yellow_side/second_puck_zone")
        self.third_puck = rospy.get_param("yellow_side/third_puck_zone")
        self.forth_puck = rospy.get_param("yellow_side/forth_puck_zone")
        self.fifth_puck = rospy.get_param("yellow_side/fifth_puck_zone")
        self.scales_zone = rospy.get_param("yellow_side/scales_zone")
        self.start_zone = rospy.get_param("yellow_side/start_zone")

        default_state = bt.Latch(bt_ros.SetToDefaultState("manipulator_client"))

        # first
        move_1_puck = bt.Latch(bt_ros.MoveLineToPoint(self.first_puck, "move_client"))
        set_man_wall = bt.Latch(bt_ros.SetManipulatortoWall("manipulator_client"))
        parallel0 = bt.ParallelNode([move_1_puck, set_man_wall], threshold=2)

        start_take_1_puck = bt.Latch(bt_ros.StartTakeWallPuck("manipulator_client"))
        complete_take_1_puck = bt.Latch(bt_ros.CompleteTakeWallPuck("manipulator_client"))
        x = self.first_puck[1] - 0.07
        self.first_puck[1] = x
        move_1_back = bt.Latch(bt_ros.MoveLineToPoint(self.first_puck, "move_client"))
        if self.side_status == SideStatus.PURPLE:
            x = self.first_puck[0] - 0.1
            self.first_puck[0] = x
        elif self.side_status == SideStatus.YELLOW:
            x = self.first_puck[0] + 0.1
            self.first_puck[0] = x
        move_1_back1 = bt.Latch(bt_ros.MoveLineToPoint(self.first_puck, "move_client"))

        asf = bt.SequenceNode([move_1_back, move_1_back1])

        parallel1 = bt.ParallelNode([asf, complete_take_1_puck], threshold=2)

        # second
        move_2_puck = bt.Latch(bt_ros.MoveLineToPoint(self.second_puck, "move_client"))

        start_take_2_puck = bt.Latch(bt_ros.StartTakeWallPuck("manipulator_client"))
        complete_take_2_puck = bt.Latch(bt_ros.CompleteTakeWallPuck("manipulator_client"))
        x = self.second_puck[1] - 0.5
        self.second_puck[1] = x
        move_2_back = bt.Latch(bt_ros.MoveLineToPoint(self.second_puck, "move_client"))
        if self.side_status == SideStatus.PURPLE:
            x = self.second_puck[0] + 0.49
            self.second_puck[0] = x
        elif self.side_status == SideStatus.YELLOW:
            x = self.second_puck[0] - 0.49
            self.second_puck[0] = x
        move_2_back2 = bt.Latch(bt_ros.MoveLineToPoint(self.second_puck, "move_client"))
        parallel2 = bt.ParallelNode([bt.SequenceNode([move_2_back, move_2_back2]), complete_take_2_puck], threshold=2)
        # third
        move_3_puck = bt.Latch(bt_ros.MoveLineToPoint(self.third_puck, "move_client"))
        start_take_3_puck = bt.Latch(bt_ros.StartTakeWallPuck("manipulator_client"))
        complete_take_3_puck = bt.Latch(bt_ros.CompleteTakeWallPuck("manipulator_client"))
        x = self.third_puck[1] - 0.04
        self.third_puck[1] = x
        move_3_back = bt.Latch(bt_ros.MoveLineToPoint(self.third_puck, "move_client"))
        if self.side_status == SideStatus.PURPLE:
            x = self.third_puck[0] + 0.2
            self.third_puck[0] = x
        elif self.side_status == SideStatus.YELLOW:
            x = self.third_puck[0] - 0.2
            self.third_puck[0] = x
        move_3_back3 = bt.Latch(bt_ros.MoveLineToPoint(self.third_puck, "move_client"))
        parallel3 = bt.ParallelNode([bt.SequenceNode([move_3_back, move_3_back3]), complete_take_3_puck], threshold=2)
        # forth
        move_4_puck = bt.Latch(bt_ros.MoveLineToPoint(self.forth_puck, "move_client"))
        start_take_4_puck = bt.Latch(bt_ros.StartTakeWallPuck("manipulator_client"))
        complete_take_4_puck = bt.Latch(bt_ros.CompleteTakeWallPuck("manipulator_client"))
        x = self.forth_puck[1] - 0.04
        self.forth_puck[1] = x
        move_4_back = bt.Latch(bt_ros.MoveLineToPoint(self.forth_puck, "move_client"))
        if self.side_status == SideStatus.PURPLE:
            x = self.forth_puck[0] + 0.21
            self.forth_puck[0] = x
        elif self.side_status == SideStatus.YELLOW:
            x = self.forth_puck[0] - 0.2
            self.forth_puck[0] = x
        move_4_back4 = bt.Latch(bt_ros.MoveLineToPoint(self.forth_puck, "move_client"))
        parallel4 = bt.ParallelNode([bt.SequenceNode([move_4_back, move_4_back4]), complete_take_4_puck], threshold=2)
        # fivth
        move_5_puck = bt.Latch(bt_ros.MoveLineToPoint(self.fifth_puck, "move_client"))
        start_take_5_puck = bt.Latch(bt_ros.StartTakeWallPuck("manipulator_client"))
        complete_take_5_puck = bt.Latch(bt_ros.CompleteTakeWallPuck("manipulator_client"))
        set_man_up = bt.Latch(bt_ros.SetManipulatortoUp("manipulator_client"))

        # Scales
        move_scales1 = bt.Latch(bt_ros.MoveLineToPoint(self.scales_zone, "move_client"))
        parallel5 = bt.ParallelNode([move_scales1, complete_take_5_puck], threshold=2)

        x = self.scales_zone[1] + 0.14
        self.scales_zone[1] = x
        move_scales2 = bt.Latch(bt_ros.MoveLineToPoint(self.scales_zone, "move_client"))
        move_back = bt.Latch(bt_ros.MoveLineToPoint(self.start_zone, "move_client"))

        parallel6 = bt.ParallelNode([set_man_up, move_scales2], threshold=2)

        release = bt.Latch(bt_ros.ReleaseFivePucks("manipulator_client"))

        first_puck = bt.SequenceNode([parallel0, start_take_1_puck, parallel1])

        second_puck = bt.SequenceNode([move_2_puck, start_take_2_puck, parallel2])

        third_puck = bt.SequenceNode([move_3_puck, start_take_3_puck, parallel3])

        forth_puck = bt.SequenceNode([move_4_puck, start_take_4_puck, parallel4])

        fivth_puck = bt.SequenceNode([move_5_puck, start_take_5_puck, parallel5, parallel6])

        scales = bt.SequenceNode([release, move_back])

        pucks = bt.SequenceNode([first_puck, second_puck, third_puck, forth_puck, fivth_puck, scales])

        self.leaf = bt.SequenceNode([default_state, pucks])




class Tactics(object):
    def update_pucks_zone(self, side):
        if side == SideStatus.YELLOW:
            self.first_puck = rospy.get_param("yellow_side/first_puck_zone")
            self.second_puck = rospy.get_param("yellow_side/second_puck_zone")
            self.third_puck = rospy.get_param("yellow_side/third_puck_zone")
            self.forth_puck = rospy.get_param("yellow_side/forth_puck_zone")
            self.fifth_puck = rospy.get_param("yellow_side/fifth_puck_zone")
            self.scales_zone = rospy.get_param("yellow_side/scales_zone")
            self.start_zone = rospy.get_param("yellow_side/start_zone")
        if side == SideStatus.PURPLE:
            self.first_puck = rospy.get_param("purple_side/first_puck_zone")
            self.second_puck = rospy.get_param("purple_side/second_puck_zone")
            self.third_puck = rospy.get_param("purple_side/third_puck_zone")
            self.forth_puck = rospy.get_param("purple_side/forth_puck_zone")
            self.fifth_puck = rospy.get_param("purple_side/fifth_puck_zone")
            self.scales_zone = rospy.get_param("purple_side/scales_zone")
            self.start_zone = rospy.get_param("purple_side/start_zone")

    def __init__(self, side_status):
        self.side_status = side_status

        self.update_pucks_zone(self.side_status)
        default_state = bt.Latch(bt_ros.SetToDefaultState("manipulator_client"))

        # first
        move_1_puck = bt.Latch(bt_ros.MoveLineToPoint(self.first_puck, "move_client"))
        set_man_wall = bt.Latch(bt_ros.SetManipulatortoWall("manipulator_client"))
        parallel0 = bt.ParallelNode([move_1_puck, set_man_wall], threshold=2)
       

        start_take_1_puck = bt.Latch(bt_ros.StartTakeWallPuck("manipulator_client"))
        complete_take_1_puck = bt.Latch(bt_ros.CompleteTakeWallPuck("manipulator_client"))
        x = self.first_puck[1] -0.07
        self.first_puck[1] = x
        move_1_back = bt.Latch(bt_ros.MoveLineToPoint(self.first_puck, "move_client"))
        if self.side_status == SideStatus.PURPLE:
            x = self.first_puck[0] - 0.1
            self.first_puck[0] = x
        elif self.side_status == SideStatus.YELLOW:
            x = self.first_puck[0] + 0.1
            self.first_puck[0] = x
        move_1_back1 = bt.Latch(bt_ros.MoveLineToPoint(self.first_puck, "move_client"))
    
        asf = bt.SequenceNode([move_1_back, move_1_back1])

        parallel1 = bt.ParallelNode([ asf , complete_take_1_puck ], threshold=2)
    
        # second
        move_2_puck = bt.Latch(bt_ros.MoveLineToPoint(self.second_puck, "move_client"))


        start_take_2_puck = bt.Latch(bt_ros.StartTakeWallPuck("manipulator_client"))
        complete_take_2_puck = bt.Latch(bt_ros.CompleteTakeWallPuck("manipulator_client"))
        x = self.second_puck[1] -0.5
        self.second_puck[1] = x
        move_2_back = bt.Latch(bt_ros.MoveLineToPoint(self.second_puck, "move_client"))
        if self.side_status == SideStatus.PURPLE:
            x = self.second_puck[0] + 0.49
            self.second_puck[0] = x
        elif self.side_status == SideStatus.YELLOW:  
            x = self.second_puck[0] - 0.49
            self.second_puck[0] = x
        move_2_back2 = bt.Latch(bt_ros.MoveLineToPoint(self.second_puck, "move_client"))
        parallel2 = bt.ParallelNode([bt.SequenceNode([move_2_back,move_2_back2]), complete_take_2_puck], threshold=2)
        # third
        move_3_puck = bt.Latch(bt_ros.MoveLineToPoint(self.third_puck, "move_client"))
        start_take_3_puck = bt.Latch(bt_ros.StartTakeWallPuck("manipulator_client"))
        complete_take_3_puck = bt.Latch(bt_ros.CompleteTakeWallPuck("manipulator_client"))
        x = self.third_puck[1] - 0.04
        self.third_puck[1] = x
        move_3_back = bt.Latch(bt_ros.MoveLineToPoint(self.third_puck, "move_client"))
        if self.side_status == SideStatus.PURPLE:
            x = self.third_puck[0] + 0.2
            self.third_puck[0] = x
        elif self.side_status == SideStatus.YELLOW:
            x = self.third_puck[0] - 0.2
            self.third_puck[0] = x
        move_3_back3 = bt.Latch(bt_ros.MoveLineToPoint(self.third_puck, "move_client"))
        parallel3 = bt.ParallelNode([bt.SequenceNode([move_3_back,move_3_back3]), complete_take_3_puck], threshold=2)
        # forth
        move_4_puck = bt.Latch(bt_ros.MoveLineToPoint(self.forth_puck, "move_client"))
        start_take_4_puck = bt.Latch(bt_ros.StartTakeWallPuck("manipulator_client"))
        complete_take_4_puck = bt.Latch(bt_ros.CompleteTakeWallPuck("manipulator_client"))
        x = self.forth_puck[1] - 0.04
        self.forth_puck[1] = x
        move_4_back = bt.Latch(bt_ros.MoveLineToPoint(self.forth_puck, "move_client"))
        if self.side_status == SideStatus.PURPLE:
            x = self.forth_puck[0] + 0.21
            self.forth_puck[0] = x
        elif self.side_status == SideStatus.YELLOW:
            x = self.forth_puck[0] - 0.2
            self.forth_puck[0] = x
        move_4_back4 = bt.Latch(bt_ros.MoveLineToPoint(self.forth_puck, "move_client"))
        parallel4 = bt.ParallelNode([bt.SequenceNode([move_4_back,move_4_back4]),complete_take_4_puck], threshold=2)
        # fivth
        move_5_puck = bt.Latch(bt_ros.MoveLineToPoint(self.fifth_puck, "move_client"))
        start_take_5_puck = bt.Latch(bt_ros.StartTakeWallPuck("manipulator_client"))
        complete_take_5_puck = bt.Latch(bt_ros.CompleteTakeWallPuck("manipulator_client"))
        set_man_up = bt.Latch(bt_ros.SetManipulatortoUp("manipulator_client"))
        
        # Scales
        move_scales1 = bt.Latch(bt_ros.MoveLineToPoint(self.scales_zone, "move_client"))
        parallel5 = bt.ParallelNode([move_scales1, complete_take_5_puck], threshold=2)

        x = self.scales_zone[1] + 0.14
        self.scales_zone[1] = x
        move_scales2 = bt.Latch(bt_ros.MoveLineToPoint(self.scales_zone, "move_client"))
        move_back = bt.Latch(bt_ros.MoveLineToPoint(self.start_zone, "move_client"))  
        
        parallel6 = bt.ParallelNode([set_man_up, move_scales2], threshold=2)

        release = bt.Latch(bt_ros.ReleaseFivePucks("manipulator_client"))

        first_puck = bt.SequenceNode([parallel0, start_take_1_puck, parallel1])

        second_puck = bt.SequenceNode([move_2_puck, start_take_2_puck, parallel2])


        third_puck = bt.SequenceNode([move_3_puck, start_take_3_puck, parallel3])

        forth_puck = bt.SequenceNode([move_4_puck, start_take_4_puck, parallel4])

        fivth_puck = bt.SequenceNode([move_5_puck, start_take_5_puck, parallel5, parallel6])

        scales = bt.SequenceNode([release, move_back])

        pucks = bt.SequenceNode([first_puck, second_puck, third_puck, forth_puck, fivth_puck, scales])

        self.leaf = bt.SequenceNode([default_state, pucks])

class SecondaryRobotBT():
    def __init__(self, side_status=SideStatus.PURPLE):
        self.robot_name = rospy.get_param("robot_name")

        self.move_publisher = rospy.Publisher("navigation/command", String, queue_size=100)
        self.move_client = bt_ros.ActionClient(self.move_publisher)
        rospy.Subscriber("navigation/response", String, self.move_client.response_callback)

        self.manipulator_publisher = rospy.Publisher("manipulator/command", String, queue_size=100)
        self.manipulator_client = bt_ros.ActionClient(self.manipulator_publisher)
        rospy.Subscriber("manipulator/response", String, self.manipulator_client.response_callback)

        self.side_status = side_status

        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener()
        self.robot_coordinates = None

        self.tactics = Tactics(self.side_status)

    def start(self):
        self.bt_timer = rospy.Timer(rospy.Duration(0.1), self.timer_callback)
        self.bt = bt.Root(self.tactics.leaf,
                          action_clients={"move_client": self.move_client, "manipulator_client": self.manipulator_client})

    def change_side(self, side):
        self.tactics = Tactics(side)

    def update_coords(self):
        try:
            trans = self.tfBuffer.lookup_transform('map', self.robot_name, rospy.Time(0))
            q = [trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z,
                 trans.transform.rotation.w]
            angle = euler_from_quaternion(q)[2] % (2 * np.pi)
            self.robot_coordinates = np.array([trans.transform.translation.x, trans.transform.translation.y, angle])
            rospy.loginfo("Robot coords:\t" + str(self.coords))
            return True
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as msg:
            rospy.logwarn(str(msg))
            rospy.logwarn("SMT WROND QQ")
            return False

    def timer_callback(self, event):
        status = self.bt.tick()
        if status != bt.Status.RUNNING:
            self.bt_timer.shutdown()
        print("============== BT LOG ================")
        self.bt.log(0)


if __name__ == '__main__':
    try:
        rospy.init_node("secondary_robot_BT")
        secondary_robot_bt = SecondaryRobotBT()
        bt_controller = BTController(secondary_robot_bt)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass