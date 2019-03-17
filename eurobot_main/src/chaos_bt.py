#!/usr/bin/env python
import rospy
import behavior_tree as bt
import bt_ros
import numpy as np
from std_msgs.msg import String

# FIXME change to if zone == "orange" then
red_cell_puck = rospy.get_param("red_cell_puck")
green_cell_puck = rospy.get_param("green_cell_puck")
blue_cell_puck = rospy.get_param("blue_cell_puck")

move_publisher = rospy.Publisher("navigation/command", String, queue_size=100)
manipulator_publisher = rospy.Publisher("manipulator/command", String, queue_size=100)

move_client = bt_ros.ActionClient(move_publisher)
manipulator_client = bt_ros.ActionClient(manipulator_publisher)


class CollectChaosPucks(bt.FallbackNode):

    # get observation from camera
    # is_chaos_collected_completely
        # calculate_pucks_configuration
        # calculate_landings
        # move to first landing
        # suck puck
        # move back for safety
        # finish suck + seq(calculate_pucks_configuration + calculate_landings) + move_to_next_prelanding


    def __init__(self, move_client, manipulator_client):

        super(CollectChaosPucks, self).__init__([
            bt.ConditionNode(self.is_chaos_collected_completely),
            bt.SequenceNode([
                bt.SequenceWithMemoryNode([
                    bt.FallbackNode([
                        bt.ConditionNode(self.is_chaos_observed),
                        self.get_chaos_observation_from_camera()
                    ]),
                    bt.SequenceNode([
                        self.calculate_pucks_configuration(),
                        self.calculate_landings()
                    ]),
                    bt.FallbackNode([
                        bt.ConditionNode(self.is_puck_sucked),
                        bt.SequenceNode([
                            bt.FallbackNode([
                                bt.ConditionNode(self.is_landing_approached),
                                bt.SequenceNode([
                                    bt.ConditionNode(self.is_trajectory_collision_free),
                                    self.approach_nearest_prelanding(),
                                    self.approach_nearest_landing(),
                                ])
                            ]),
                            self.start_suck()
                        ])
                    ]),
                    bt.FallbackNode([
                        bt.ConditionNode(self.is_safe_away_from_other_pucks),
                        bt.SequenceNode([
                            bt.ConditionNode(self.is_trajectory_collision_free),
                            self.drive_back()
                        ])
                    ]),
                    bt.ParallelNode([
                        self.finish_suck(),
                        bt.SequenceNode([
                            self.calculate_pucks_configuration(),
                            self.calculate_landings()
                        ]),
                        bt.SequenceNode([
                            bt.ConditionNode(self.is_trajectory_collision_free),
                            self.approach_nearest_prelanding(),
                            self.approach_nearest_landing(),
                        ])
                    ], threshold=3),  # FIXME
                ]),
                bt.ConditionNode(lambda: bt.Status.RUNNING),
            ])
        ])

#

# CollectChaos = CollectChaosPucks(move_client, manipulator_client)



class MainRobotBT:
    def __init__(self):
        rospy.init_node("chaos_bt")

        self.robot_name = rospy.get_param("robot_name")


        rospy.Subscriber("navigation/response", String, self.move_client.response_callback)


        rospy.Subscriber("manipulator/response", String, self.manipulator_client.response_callback)

        rospy.sleep(2)

        # Calibrate manipulator
        default_state = bt.Latch(bt_ros.SetToDefaultState("manipulator_client"))


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

        leaf = bt.SequenceNode([default_state, pucks])

        self.bt = bt.Root(leaf, action_clients={"move_client": self.move_client, "manipulator_client": self.manipulator_client})

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
        tree = MainRobotBT()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
