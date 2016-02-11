#!/usr/bin/env python

import rospy
import numpy as np
import behavior_tree as bt
import bt_ros
import tf2_ros
from tf.transformations import euler_from_quaternion
from core_functions import *
from std_msgs.msg import String
from tactics_math import *
from score_controller import ScoreController
from visualization_msgs.msg import MarkerArray


class CollectChaos(bt.SequenceWithMemoryNode):
    def __init__(self, known_chaos_pucks, action_client_id):
        # Init parameters
        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer)
        self.main_coords = None
        self.known_chaos_pucks = bt.BTVariable(np.array([]))  # (x, y, id, r, g, b)
        self.known_chaos_pucks.set(known_chaos_pucks)

        self.collected_pucks = bt.BTVariable(np.array([]))
        self.score_master = ScoreController(self.collected_pucks, "main_robot")

        self.waypoints = bt.BTVariable(np.array([]))
        self.incoming_puck_color = bt.BTVariable(None)
        self.approach_dist = rospy.get_param("approach_dist")  # meters, distance from robot to puck where robot will try to grab it
        self.approach_dist = np.array(self.approach_dist)
        self.critical_angle = np.pi * 2/3

        self.drive_back_dist = rospy.get_param("drive_back_dist")
        self.drive_back_dist = np.array(self.drive_back_dist)

        self.approach_vec = np.array([-1*self.approach_dist, 0, 0])
        self.drive_back_vec = np.array([-1*self.drive_back_dist, 0, 0])
        self.scale_factor = rospy.get_param("scale_factor")  # used in calculating outer bissectrisa for hull's angles
        self.scale_factor = np.array(self.scale_factor)

        self.starting_pos = np.array([0.3, 0.45, 0])
        self.starting_pos_var = bt.BTVariable(self.starting_pos)

        self.guard_chaos_point = np.array([0.6, 0.6, 0.78])  # FIXME move to get_param
        self.guard_chaos_point = bt.BTVariable(self.guard_chaos_point)

        self.closest_landing = bt.BTVariable()
        self.nearest_PRElanding = bt.BTVariable()

        super(CollectChaos, self).__init__([
                # 1st
                bt.ActionNode(self.calculate_pucks_configuration),
                bt.ActionNode(self.calculate_closest_landing),
                bt.ActionNode(self.calculate_prelanding),

                # bt.ParallelWithMemoryNode([
                #     bt_ros.SetManipulatortoUp("manipulator_client"),
                #     bt_ros.MoveToVariable(self.guard_chaos_point, "move_client")
                # ], threshold=2),

                bt_ros.MoveToVariable(self.guard_chaos_point, "move_client"),
                bt_ros.MoveToVariable(self.nearest_PRElanding, "move_client"),
                bt_ros.ArcMoveToVariable(self.closest_landing, "move_client"),
                bt_ros.BlindStartCollectGround("manipulator_client"),
                bt.ActionNode(self.update_chaos_pucks),
                bt.ActionNode(lambda: self.score_master.add(self.incoming_puck_color.get())),
                bt_ros.MoveToVariable(self.nearest_PRElanding, "move_client"),

                # 2nd
                bt.ActionNode(self.calculate_pucks_configuration),
                bt.ActionNode(self.calculate_closest_landing),
                bt.ActionNode(self.calculate_prelanding),

                bt.ParallelWithMemoryNode([
                    bt_ros.CompleteCollectGround("manipulator_client"),
                    bt_ros.ArcMoveToVariable(self.nearest_PRElanding, "move_client"),
                ], threshold=2),

                bt_ros.MoveToVariable(self.closest_landing, "move_client"),
                bt_ros.BlindStartCollectGround("manipulator_client"),
                bt.ActionNode(self.update_chaos_pucks),
                bt.ActionNode(lambda: self.score_master.add(self.incoming_puck_color.get())),
                bt_ros.MoveToVariable(self.nearest_PRElanding, "move_client"),

                # 3rd
                bt.ActionNode(self.calculate_pucks_configuration),
                bt.ActionNode(self.calculate_closest_landing),
                bt.ActionNode(self.calculate_prelanding),

                bt.ParallelWithMemoryNode([
                    bt_ros.CompleteCollectGround("manipulator_client"),
                    bt_ros.ArcMoveToVariable(self.nearest_PRElanding, "move_client"),
                ], threshold=2),

                bt_ros.MoveToVariable(self.closest_landing, "move_client"),
                bt_ros.BlindStartCollectGround("manipulator_client"),
                bt.ActionNode(self.update_chaos_pucks),
                bt.ActionNode(lambda: self.score_master.add(self.incoming_puck_color.get())),
                bt_ros.MoveToVariable(self.nearest_PRElanding, "move_client"),

                # 4th
                bt.ActionNode(self.calculate_pucks_configuration),
                bt.ActionNode(self.calculate_closest_landing),

                bt.ParallelWithMemoryNode([
                    bt_ros.CompleteCollectGround("manipulator_client"),
                    bt_ros.MoveToVariable(self.closest_landing, "move_client"),
                ], threshold=2),

                bt_ros.BlindStartCollectGround("manipulator_client"),
                bt.ActionNode(self.update_chaos_pucks),
                bt.ActionNode(lambda: self.score_master.add(self.incoming_puck_color.get())),

                # back_to_start
                bt.ParallelWithMemoryNode([
                    bt.SequenceWithMemoryNode([
                        bt_ros.CompleteCollectGround("manipulator_client"),
                        bt_ros.StepperUp("manipulator_client"),
                        bt_ros.MainSetManipulatortoGround("manipulator_client")
                    ]),
                    bt_ros.MoveToVariable(self.starting_pos_var, "move_client"),
                ], threshold=2),

                bt_ros.UnloadAccelerator("manipulator_client"),
                bt.ActionNode(lambda: self.score_master.unload("ACC")),
                bt_ros.UnloadAccelerator("manipulator_client"),
                bt.ActionNode(lambda: self.score_master.unload("ACC")),
                bt_ros.UnloadAccelerator("manipulator_client"),
                bt.ActionNode(lambda: self.score_master.unload("ACC")),
                bt_ros.UnloadAccelerator("manipulator_client"),
                bt.ActionNode(lambda: self.score_master.unload("ACC")),  # COMAAAA
                bt_ros.SetManipulatortoUp("manipulator_client")

            ])

    @staticmethod
    def get_color(puck):
        """
        red (1, 0, 0)
        green (0, 1, 0)
        blue (0, 0, 1)
        :param puck: (x, y, id, 0, 0, 1)
        :return:
        """
        pucks_colors = {
            (1, 0, 0): "REDIUM",
            (0, 1, 0): "GREENIUM",
            (0, 0, 1): "BLUNIUM"
        }
        color_val = None
        color_key = puck[3:]
        print color_key
        if all(color_key == np.array([1, 0, 0])):
            color_val = "REDIUM"
        elif all(color_key == np.array([0, 1, 0])):
            color_val = "GREENIUM"
        elif all(color_key == np.array([0, 0, 1])):
            color_val = "BLUNIUM"
        # color_val = pucks_colors.get(color_key)
        return color_val

    def update_chaos_pucks(self):
        """
        delete taken puck from known on the field
        get color of last taken puck
        :return: None
        """
        incoming_puck_color = self.get_color(self.known_chaos_pucks.get()[0])
        self.incoming_puck_color.set(incoming_puck_color)
        rospy.loginfo("incoming_puck_color: " + str(self.incoming_puck_color.get()))
        self.known_chaos_pucks.set(np.delete(self.known_chaos_pucks.get(), 0, axis=0))
        rospy.loginfo("Known pucks after removing: " + str(self.known_chaos_pucks.get()))

    def calculate_pucks_configuration(self):
        """

        :return: # [(0.95, 1.1, 3, 0, 0, 1), ...]
        """
        while not self.update_main_coords():
            print "no coords available"
            rospy.sleep(0.5)

        known_chaos_pucks = sort_wrt_robot(self.main_coords, self.known_chaos_pucks.get())
        print "sorted"
        self.known_chaos_pucks.set(known_chaos_pucks)
        if len(self.known_chaos_pucks.get()) >= 3:
            is_hull_safe_to_approach, coords_sorted_by_angle = sort_by_inner_angle_and_check_if_safe(self.main_coords,
                                                                                                     self.known_chaos_pucks.get(),
                                                                                                     self.critical_angle)

            if not is_hull_safe_to_approach:
                self.known_chaos_pucks.set(coords_sorted_by_angle)  # calc vert-angle, sort by angle, return vertices (sorted)
                rospy.loginfo("hull is not safe to approach, sorted by angle")
            else:  # only sharp angles
                rospy.loginfo("hull is SAFE to approach, keep already sorted wrt robot")
        rospy.loginfo("Known pucks sorted: " + str(self.known_chaos_pucks.get()))

    #     when we finally sorted them, chec if one of them is blue. If so, roll it so blue becomes last one to collect
    #     if self.known_chaos_pucks.get().size > 1 and all(self.known_chaos_pucks.get()[0][3:6] == [0, 0, 1]):
    #         # self.known_chaos_pucks.set(np.roll(self.known_chaos_pucks.get(), -1, axis=0))
    #         rospy.loginfo("blue rolled")

    def calculate_closest_landing(self):
        """

        :return: [(x, y, theta), ...]
        """
        if len(self.known_chaos_pucks.get()) == 1:
            landings = calculate_closest_landing_to_point(self.main_coords,
                                                          self.known_chaos_pucks.get()[:, :2],
                                                          self.approach_vec)
        else:
            landings = unleash_power_of_geometry(self.known_chaos_pucks.get()[:, :2],
                                                 self.scale_factor,
                                                 self.approach_dist)

        self.closest_landing.set(landings[0])
        # self.waypoints.set(landings[0])
        rospy.loginfo("Inside calculate_closest_landing, waypoints are : ")
        print(self.closest_landing.get())
        # print self.waypoints.get()
        print " "

    def calculate_prelanding(self):
        nearest_PRElanding = cvt_local2global(self.drive_back_vec, self.closest_landing.get())
        self.nearest_PRElanding.set(nearest_PRElanding)
        rospy.loginfo("Nearest PRElanding calculated: " + str(self.nearest_PRElanding.get()))
        print " "

    def update_main_coords(self):
        try:
            trans_main = self.tfBuffer.lookup_transform('map', "main_robot", rospy.Time(0))  # 0 means last measurment
            q_main = [trans_main.transform.rotation.x,
                      trans_main.transform.rotation.y,
                      trans_main.transform.rotation.z,
                      trans_main.transform.rotation.w]
            angle_main = euler_from_quaternion(q_main)[2] % (2 * np.pi)
            self.main_coords = np.array([trans_main.transform.translation.x,
                                         trans_main.transform.translation.y,
                                         angle_main])
            rospy.loginfo("main coords: " + str(self.main_coords))
            return True
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as msg:
            rospy.logwarn(str(msg))
            return False


class MainRobotBT(object):
    def __init__(self):
        # rospy.init_node("test_bt_node")
        self.move_publisher = rospy.Publisher("navigation/command", String, queue_size=100)
        self.move_client = bt_ros.ActionClient(self.move_publisher)
        self.manipulator_publisher = rospy.Publisher("manipulator/command", String, queue_size=100)
        self.manipulator_client = bt_ros.ActionClient(self.manipulator_publisher)
        self.is_observed_flag = bt.BTVariable(False)
        self.known_chaos_pucks = bt.BTVariable(np.array([]))  # (x, y, id, r, g, b)

        self.pucks_subscriber = rospy.Subscriber("/pucks", MarkerArray, self.pucks_callback, queue_size=1)
        rospy.Subscriber("navigation/response", String, self.move_client.response_callback)
        rospy.Subscriber("manipulator/response", String, self.manipulator_client.response_callback)

        # yellow
        # self.known_chaos_pucks = np.array([[1.95, 1.05, 1, 1, 0, 0],
        #                                     [2, 1.1, 2, 0, 1, 0],
        #                                     [2, 1, 3, 0, 0, 1],
        #                                     [2.05, 1.05, 4, 1, 0, 0]])

        # purple
        # self.known_chaos_pucks = np.array([[0.95, 1.05, 1, 1, 0, 0],
        #                                     [1, 1.1, 2, 0, 1, 0],
        #                                     [1, 1, 3, 0, 0, 1],
        #                                     [1.05, 1.05, 4, 1, 0, 0]])

        rospy.sleep(15)
        self.bt = bt.Root(
                    bt.FallbackWithMemoryNode([
                        bt.SequenceNode([
                            bt.ConditionNode(self.is_observed),
                            CollectChaos(self.known_chaos_pucks.get(), "move_client")
                        ]),
                        bt.ConditionNode(lambda: bt.Status.RUNNING)
                    ]),
                    action_clients={"move_client": self.move_client,
                                    "manipulator_client": self.manipulator_client})
        self.bt_timer = rospy.Timer(rospy.Duration(0.1), self.timer_callback)


    ## WORKS
    def pucks_callback(self, data):
        # [(0.95, 1.1, 3, 0, 0, 1), ...] - blue, id=3  IDs are not guaranteed to be the same from frame to frame
        # red (1, 0, 0)
        # green (0, 1, 0)
        # blue (0, 0, 1)
        rospy.loginfo(data)

        if len(self.known_chaos_pucks.get()) == 0:
            try:
                new_observation_pucks = [[marker.pose.position.x,
                                          marker.pose.position.y,
                                          marker.id,
                                          marker.color.r,
                                          marker.color.g,
                                          marker.color.b] for marker in data.markers]

                if len(new_observation_pucks) == 4:
                    new_observation_pucks = np.array(new_observation_pucks)

                    self.known_chaos_pucks.set(new_observation_pucks)
                    self.is_observed_flag.set(True)
                    rospy.loginfo("Got pucks observation:")
                    rospy.loginfo(self.known_chaos_pucks.get())
                    self.pucks_subscriber.unregister()

            except Exception:  # FIXME
                rospy.loginfo("list index out of range - no visible pucks on the field ")

    # def pucks_callback(self, data):
    #     rospy.loginfo(data)

    #     new_observation_pucks = [[marker.pose.position.x,
    #                                 marker.pose.position.y,
    #                                 marker.id,
    #                                 marker.color.r,
    #                                 marker.color.g,
    #                                 marker.color.b] for marker in data.markers]

    #     new_observation_pucks = np.array(new_observation_pucks)

    #     self.known_chaos_pucks.set(new_observation_pucks)
    #     self.is_observed_flag.set(True)
    #     rospy.loginfo("Got pucks observation:")
    #     rospy.loginfo(self.known_chaos_pucks.get())
    #     rospy.loginfo(self.is_observed_flag.get())


    def is_observed(self):
        # rospy.loginfo("is observed?")
        if self.is_observed_flag.get():
            # rospy.loginfo('YES! Got all pucks coords')
            return bt.Status.SUCCESS
        else:
            rospy.loginfo('Still waiting for the cam, known: ' + str(len(self.known_chaos_pucks.get())))
            return bt.Status.FAILED

    def timer_callback(self, event):
        status = self.bt.tick()
        if status != bt.Status.RUNNING:
            self.bt_timer.shutdown()
        # print("============== BT LOG ================")
        self.bt.log(0)


if __name__ == '__main__':
    try:
        rospy.init_node("main_robot_BT")
        rospy.sleep(1)
        main_robot_bt = MainRobotBT()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
