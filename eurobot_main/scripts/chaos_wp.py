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


class CollectChaos(bt.FallbackNode):
    def __init__(self, known_chaos_pucks, action_client_id):
        # Init parameters
        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer)
        self.main_coords = None
        self.known_chaos_pucks = bt.BTVariable(np.array([]))
        self.known_chaos_pucks.set(known_chaos_pucks)

        self.is_observed = bt.BTVariable(True)  # FIXME to FALSE!!!!!!!!!!!!!!!!!!

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

        # Init useful child nodes
        self.move_to_waypoint_node = bt_ros.ActionClientNode("move 0 0 0", action_client_id, name="move_to_waypoint")
        self.choose_new_waypoint_latch = bt.Latch(bt.ActionNode(self.choose_new_waypoint))

        # Make BT
        # super(CollectChaos, self).__init__([
        #     bt.ConditionNode(self.is_chaos_empty),
        #     bt.SequenceNode([
        #         bt.SequenceWithMemoryNode([
        #             bt.ActionNode(self.calculate_pucks_configuration),
        #             bt.ActionNode(self.calculate_closest_landing),
        #             bt.ActionNode(self.calculate_prelanding),
        #
        #             bt.FallbackNode([
        #                 bt.ConditionNode(lambda: bt.Status.FAILED if len(self.waypoints.get()) > 0 else bt.Status.SUCCESS),
        #                 bt.SequenceNode([
        #                     self.choose_new_waypoint_latch,
        #                     self.move_to_waypoint_node,
        #                     bt.ActionNode(self.remove_waypoint),
        #                     bt.ActionNode(self.choose_new_waypoint_latch.reset),
        #                     bt.ConditionNode(lambda: bt.Status.RUNNING)
        #                 ])
        #             ]),
        #             bt_ros.BlindStartCollectGround("manipulator_client"),
        #             bt.ActionNode(self.update_chaos_pucks),
        #             bt.ActionNode(lambda: self.score_master.add(self.incoming_puck_color.get())),
        #             bt_ros.CompleteCollectGround("manipulator_client"),
        #
        #
        #         ]),
        #         # bt_ros.SetManipulatortoWall("manipulator_client"),
        #         # bt_ros.SetManipulatortoUp("manipulator_client"),
        #         # # bt.ActionNode(self.calculate_drive_back_point),
        #
        #         # bt.ActionNode(self.update_chaos_pucks),
        #
        #         bt.ConditionNode(lambda: bt.Status.RUNNING)
        #     ])
        # ])

        super(CollectChaos, self).__init__([
            bt.ConditionNode(self.is_chaos_empty),
            bt.SequenceNode([
                bt.SequenceWithMemoryNode([
                    bt.ActionNode(self.calculate_pucks_configuration),
                    bt.ActionNode(self.calculate_closest_landing),
                    bt.ActionNode(self.calculate_prelanding),
                    # bt.ActionNode(self.calculate_drive_back_point),

                    bt.FallbackNode([
                        bt.ConditionNode(lambda: bt.Status.FAILED if len(self.waypoints.get()) > 0 else bt.Status.SUCCESS),
                        bt.SequenceNode([
                            self.choose_new_waypoint_latch,
                            self.move_to_waypoint_node,
                            bt.ActionNode(self.remove_waypoint),
                            bt.ActionNode(self.choose_new_waypoint_latch.reset),
                            bt.ConditionNode(lambda: bt.Status.RUNNING)
                        ])
                    ]),
                    bt_ros.BlindStartCollectGround("manipulator_client"),
                    bt.ActionNode(self.update_chaos_pucks),
                    bt.ActionNode(lambda: self.score_master.add(self.incoming_puck_color.get())),
                    # self.choose_new_waypoint_latch,
                    # self.move_to_waypoint_node,
                    # bt.ActionNode(self.clear_waypoints),
                    # bt.ActionNode(self.choose_new_waypoint_latch.reset),
                    bt_ros.CompleteCollectGround("manipulator_client"),
                ]),

                bt.ConditionNode(lambda: bt.Status.RUNNING)
            ])
        ])


        # when list of known already updated
        # bt.SequenceWithMemoryNode([
        #     bt.ConditionNode(self.is_not_first),
        #     bt.ActionNode(self.calculate_drive_back_point),
        #     bt.ActionNode(self.calculate_closest_landing),
        #     bt.ActionNode(self.calculate_prelanding),
        #
        #
        # ])
        #
        # bt.ParallelWithMemoryNode([
        #     self.move_to_waypoint_node,
        #
        # ])

        # TODO take into account, that when colecting 7 pucks, don't make step down
        # collect_chaos = bt.SequenceNode([
        #                     bt.FallbackNode([
        #                         # completely?
        #                         bt.ConditionNode(self.is_chaos_collected_completely),
        #
        #                         # if last puck, just collect it, return success and get fuck out of here
        #                         bt.SequenceWithMemoryNode([
        #                             bt.ConditionNode(self.is_last_puck),
        #                             bt.SequenceWithMemoryNode([
        #                                 bt_ros.StartCollectGround("manipulator_client"),
        #                                 bt_ros.CompleteCollectGround("manipulator_client"),
        #                                 bt.ActionNode(self.update_chaos_pucks),
        #                                 bt.ActionNode(lambda: self.score_master.add(self.incoming_puck_color))
        #                             ])
        #                         ]),
        #
        #                         # calc config and start collect
        #                         bt.SequenceWithMemoryNode([
        #                             bt_ros.StartCollectGround("manipulator_client"),
        #                             self.calculate_drive_back_point_latch,
        #                             self.move_to_waypoint_node, # FIXME make it closer
        #
        #                             # calc new landing
        #                             bt.ActionNode(self.calculate_pucks_configuration),
        #                             bt.ActionNode(self.calculate_landings),
        #
        #                             # drive back, collect and move to new prelanding
        #                             bt.ParallelWithMemoryNode([
        #                                 bt_ros.CompleteCollectGround("manipulator_client"),
        #                                 bt.ActionNode(self.update_chaos_pucks),
        #                                 bt.ActionNode(lambda: self.score_master.add(self.incoming_puck_color.get())),
        #                                 self.calculate_prelanding_latch,
        #                                 self.move_to_waypoint_node,
        #                             ], threshold=5),
        #
        #                             bt_ros.MoveLineToPoint(self.nearest_landing.get(), "move_client"),
        #                         ]),
        #                     ]),
        #                     bt.ConditionNode(lambda: self.is_chaos_collected_completely1())
        #                 ])


    # def pucks_callback(self, data):
    #     self.is_observed.set(True)
    #     # [(0.95, 1.1, 3, 0, 0, 1), ...] - blue, id=3  IDs are not guaranteed to be the same from frame to frame
    #     # red (1, 0, 0)
    #     # green (0, 1, 0)
    #     # blue (0, 0, 1)
    #
    #     if len(self.known_chaos_pucks.get()) == 0:
    #         try:
    #             new_observation_pucks = [[marker.pose.position.x,
    #                                       marker.pose.position.y,
    #                                       marker.id,
    #                                       marker.color.r,
    #                                       marker.color.g,
    #                                       marker.color.b] for marker in data.markers]
    #
    #             self.known_chaos_pucks.set(np.append(self.known_chaos_pucks.get(), new_observation_pucks))  # Action
    #             rospy.loginfo("Got pucks observation:")
    #             rospy.loginfo(self.known_chaos_pucks.get())
    #             self.pucks_subscriber.unregister()
    #
    #         except Exception:  # FIXME
    #             rospy.loginfo("list index out of range - no visible pucks on the field ")

    # def is_chaos_collected_completely(self):
    #     if len(self.known_chaos_pucks.get()) == 0:
    #         rospy.loginfo("Chaos collected completely")
    #         return bt.Status.SUCCESS
    #     else:
    #         return bt.Status.FAILED
    #
    # def is_chaos_collected_completely1(self):
    #     if len(self.known_chaos_pucks.get()) == 0:
    #         rospy.loginfo("Chaos collected completely")
    #         return bt.Status.SUCCESS
    #     else:
    #         return bt.Status.RUNNING

    def is_last_puck(self):
        if len(self.known_chaos_pucks.get()) == 1:
            return bt.Status.SUCCESS
        else:
            return bt.Status.RUNNING

    def is_chaos_observed(self):
        if self.is_observed.get():
            return bt.Status.SUCCESS
        else:
            return bt.Status.RUNNING

    def is_chaos_empty(self):
        # print self.known_chaos_pucks.get()
        if len(self.known_chaos_pucks.get()) > 0:
            return bt.Status.FAILED
        else:
            return bt.Status.SUCCESS

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

        self.waypoints.set(landings[0])
        rospy.loginfo("Inside calculate_closest_landing, waypoints are : ")
        print self.waypoints.get()
        print " "

    def calculate_prelanding(self):
        nearest_landing = self.waypoints.get()
        nearest_PRElanding = cvt_local2global(self.drive_back_vec, nearest_landing)
        self.waypoints.set(np.stack((nearest_PRElanding, self.waypoints.get())))
        # rospy.loginfo("Nearest PRElanding calculated: " + str(self.waypoints.get()[0]))
        rospy.loginfo("Waypoints after concatenation: ")
        print self.waypoints.get()
        print " "

    def calculate_drive_back_point(self):
        # nearest_landing = self.waypoints.get()[-1]
        self.update_main_coords()
        drive_back_point = cvt_local2global(self.drive_back_vec, self.main_coords)
        # self.waypoints.set(drive_back_point)
        self.waypoints.set(np.concatenate((self.waypoints.get(), drive_back_point[np.newaxis, :]), axis=0))
        rospy.loginfo("Inside calculate_drive_back_point, drive_back_point is : ")
        print self.waypoints.get()

    def choose_new_waypoint(self):
        current_waypoint = self.waypoints.get()[0]
        rospy.loginfo("current_waypoint: " + str(current_waypoint))
        self.move_to_waypoint_node.cmd.set("move_arc %f %f %f" % tuple(current_waypoint))  # FIXME arc

    def remove_waypoint(self):
        self.waypoints.set(self.waypoints.get()[1:])

    def clear_waypoints(self):
        self.waypoints.set(np.array([]))

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

        self.is_observed = bt.BTVariable(False)  # FIXME to FALSE!!!!!!!!!!!!!!!!!!
        # self.known_chaos_pucks = bt.BTVariable(np.array([]))  # (x, y, id, r, g, b)  # FIXME

        self.pucks_subscriber = rospy.Subscriber("/pucks", MarkerArray, self.pucks_callback, queue_size=1)
        rospy.Subscriber("navigation/response", String, self.move_client.response_callback)
        rospy.Subscriber("manipulator/response", String, self.manipulator_client.response_callback)

        # self.known_chaos_pucks = np.array([[1.7, 0.8, 1, 1, 0, 0],
        #                                     [1.9, 0.9, 2, 0, 1, 0],
        #                                     [2.1, 0.85, 3, 0, 0, 1],
        #                                     [1.9, 1.1, 4, 1, 0, 0]])

        # yellow
        # self.known_chaos_pucks = np.array([[1.95, 1.05, 1, 1, 0, 0],
        #                                     [2, 1.1, 2, 0, 1, 0],
        #                                     [2, 1, 3, 0, 0, 1],
        #                                     [2.05, 1.05, 4, 1, 0, 0]])

        # purple
        self.known_chaos_pucks = np.array([[0.95, 1.05, 1, 1, 0, 0],
                                            [1, 1.1, 2, 0, 1, 0],
                                            [1, 1, 3, 0, 0, 1],
                                            [1.05, 1.05, 4, 1, 0, 0]])

        # self.known_chaos_pucks = bt.BTVariable(self.known_chaos_pucks)

        rospy.sleep(1)
        self.bt = bt.Root(
                    CollectChaos(self.known_chaos_pucks,
                        "move_client"), action_clients={"move_client": self.move_client,
                                                        "manipulator_client": self.manipulator_client})
        self.bt_timer = rospy.Timer(rospy.Duration(0.1), self.timer_callback)

    def pucks_callback(self, data):
        self.is_observed.set(True)
        # [(0.95, 1.1, 3, 0, 0, 1), ...] - blue, id=3  IDs are not guaranteed to be the same from frame to frame
        # red (1, 0, 0)
        # green (0, 1, 0)
        # blue (0, 0, 1)

        if len(self.known_chaos_pucks.get()) == 0:
            try:
                new_observation_pucks = [[marker.pose.position.x,
                                          marker.pose.position.y,
                                          marker.id,
                                          marker.color.r,
                                          marker.color.g,
                                          marker.color.b] for marker in data.markers]

                new_observation_pucks = np.array(new_observation_pucks)

                self.known_chaos_pucks.set(new_observation_pucks)
                rospy.loginfo("Got pucks observation:")
                rospy.loginfo(self.known_chaos_pucks.get())
                self.pucks_subscriber.unregister()

            except Exception:  # FIXME
                rospy.loginfo("list index out of range - no visible pucks on the field ")

    def timer_callback(self, event):
        status = self.bt.tick()
        if status != bt.Status.RUNNING:
            self.bt_timer.shutdown()
        # print("============== BT LOG ================")
        self.bt.log(0)


if __name__ == '__main__':
    try:
        rospy.init_node("main_robot_BT")
        main_robot_bt = MainRobotBT()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
