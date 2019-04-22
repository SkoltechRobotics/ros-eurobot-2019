import rospy
import numpy as np
import behavior_tree as bt
import bt_ros
import tf2_ros
from tf.transformations import euler_from_quaternion
from core_functions import *
from std_msgs.msg import String
from tactics_math import *


class CollectChaos(bt.FallbackNode):
    def __init__(self, known_chaos_pucks, action_client_id):
        # Init parameters
        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer)

        self.known_chaos_pucks = bt.BTVariable(known_chaos_pucks)
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
        super(CollectChaos, self).__init__([
            bt.ConditionNode(self.is_chaos_empty),
            bt.SequenceNode([
                bt.SequenceWithMemoryNode([
                    bt.ActionNode(self.calculate_pucks_configuration),
                    bt.ActionNode(self.calculate_closest_landing),
                    bt.ActionNode(self.calculate_prelanding),
                    bt.ActionNode(self.calculate_drive_back_point),

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
                ]),
                bt.ActionNode(self.update_chaos_pucks),
                bt.ConditionNode(lambda: bt.Status.RUNNING)
            ])
        ])

    def is_chaos_empty(self):
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
        color_key = puck[3:]
        color_val = pucks_colors.get(color_key)
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

    def calculate_pucks_configuration(self):
        """

        :return: # [(0.95, 1.1, 3, 0, 0, 1), ...]
        """
        self.update_main_coords()
        known_chaos_pucks = sort_wrt_robot(self.main_coords, self.known_chaos_pucks.get())

        self.known_chaos_pucks.set(known_chaos_pucks)

        if self.known_chaos_pucks.get().size >= 3:
            is_hull_safe_to_approach, coords_sorted_by_angle = sort_by_inner_angle_and_check_if_safe(self.main_coords,
                                                                                                     self.known_chaos_pucks.get(),
                                                                                                     self.critical_angle)

            if not is_hull_safe_to_approach:
                self.known_chaos_pucks.set(coords_sorted_by_angle)  # calc vert-angle, sort by angle, return vertices (sorted)
                rospy.loginfo("hull is not safe to approach, sorted by angle")

            else:  # only sharp angles
                rospy.loginfo("hull is SAFE to approach, keep already sorted wrt robot")

    def calculate_closest_landing(self):
        """

        :return: [(x, y, theta), ...]
        """
        if self.known_chaos_pucks.get().size == 1:
            landings = calculate_closest_landing_to_point(self.main_coords,
                                                          self.known_chaos_pucks.get()[:, :2],
                                                          self.approach_vec)
        else:
            landings = unleash_power_of_geometry(self.known_chaos_pucks.get()[:, :2],
                                                 self.scale_factor,
                                                 self.approach_dist)

        self.waypoints.set(landings[0])
        rospy.loginfo("Inside calculate_closest_landing, waypoints are : " + str(self.waypoints.get()))

    def calculate_prelanding(self):
        nearest_landing = self.waypoints.get()[0]
        nearest_PRElanding = cvt_local2global(self.drive_back_vec, nearest_landing)
        self.waypoints.set(np.concatenate(nearest_PRElanding, self.waypoints.get()))
        rospy.loginfo("Nearest PRElanding calculated: " + str(self.waypoints.get()[0]))

    def calculate_drive_back_point(self):
        nearest_landing = self.waypoints.get()[-1]
        drive_back_point = cvt_local2global(self.drive_back_vec, nearest_landing)
        self.waypoints.set(np.append(self.waypoints.get(), drive_back_point))
        rospy.loginfo("Nearest drive_back_point calculated: " + str(self.waypoints.get()[-1]))

    def choose_new_waypoint(self):
        current_waypoint = self.waypoints.get()[0]
        rospy.loginfo("current_waypoint: " + str(current_waypoint))
        self.move_to_waypoint_node.cmd.set("move_line %f %f %f" % tuple(current_waypoint))

    def remove_waypoint(self):
        self.waypoints.set(self.waypoints.get()[1:])

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
            # return True
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as msg:
            rospy.logwarn(str(msg))
            # return False


class MainRobotBT(object):
    def __init__(self):
        rospy.init_node("test_bt_node")
        self.move_publisher = rospy.Publisher("/navigation/move_command", String, queue_size=100)
        self.move_client = bt_ros.ActionClient(self.move_publisher)
        rospy.Subscriber("/response", String, self.move_client.response_callback)

        self.known_chaos_pucks = bt.BTVariable(np.array([[1.7, 0.8, 1, 1, 0, 0],
                                                            [1.9, 0.9, 2, 0, 1, 0],
                                                            [2.1, 0.85, 3, 0, 0, 1],
                                                            [1.9, 1.1, 4, 1, 0, 0]]))

        rospy.sleep(1)
        self.bt = bt.Root(
                    CollectChaos(self.known_chaos_pucks,
                        "move_client"), action_clients={"move_client": self.move_client})
        self.bt_timer = rospy.Timer(rospy.Duration(0.1), self.timer_callback)

    def timer_callback(self, event):
        status = self.bt.tick()
        if status != bt.Status.RUNNING:
            self.bt_timer.shutdown()
        print("============== BT LOG ================")
        self.bt.log(0)


if __name__ == '__main__':
    try:
        rospy.init_node("main_robot_BT")
        main_robot_bt = MainRobotBT()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
