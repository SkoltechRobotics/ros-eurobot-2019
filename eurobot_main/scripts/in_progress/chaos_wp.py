import rospy
import behavior_tree as bt
import bt_ros
import numpy as np
from std_msgs.msg import String


class CollectChaos(bt.FallbackNode):
    def __init__(self, waypoints, action_client_id):
        # Init parameters
        self.waypoints = bt.BTVariable(waypoints)

        # Init useful child nodes
        self.move_to_waypoint_node = bt_ros.ActionClientNode("move 0 0 0", action_client_id, name="move_to_waypoint")
        self.choose_new_waypoint_latch = bt.Latch(bt.ActionNode(self.choose_new_waypoint))

        # Make BT
        super(CollectChaos, self).__init__([
            bt.ConditionNode(self.is_chaos_empty),
            bt.SequenceNode([
                bt.ActionNode(self.calculate_pucks_configuration),
                bt.ActionNode(self.calculate_landings),
                self.choose_new_waypoint_latch,
                self.move_to_waypoint_node,
                bt.ActionNode(self.remove_waypoint),
                bt.ActionNode(self.choose_new_waypoint_latch.reset),
                bt.ConditionNode(lambda: bt.Status.RUNNING)
            ]),
        ])

    def is_chaos_empty(self):
        if len(self.waypoints.get()) > 0:
            return bt.Status.FAILED
        else:
            return bt.Status.SUCCESS

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
            if is_hull_safe_to_approach:  # only sharp angles
                rospy.loginfo("hull is SAFE to approach, sorted wrt robot")

            if not is_hull_safe_to_approach:
                self.known_chaos_pucks.set(coords_sorted_by_angle)  # calc vert-angle, sort by angle, return vertices (sorted)
                # known_chaos_pucks = coords_sorted_by_angle
                rospy.loginfo("hull is not safe to approach, sorted by angle")

    def calculate_landings(self):
        """

        :return: [(x, y, theta), ...]
        """
        print "calculating landings"
        coords = self.known_chaos_pucks.get()[:, :2]
        if coords.size == 1:
            landings = calculate_closest_landing_to_point(self.main_coords, coords, self.approach_vec)
            self.sorted_chaos_landings.set(landings)
        else:
            landings = unleash_power_of_geometry(coords, self.scale_factor, self.approach_dist)
            self.sorted_chaos_landings.set(landings)
        print("sorted landings")
        print(self.sorted_chaos_landings.get())

    def choose_new_waypoint(self):
        current_waypoint = self.waypoints.get()[0]
        print(self.waypoints.get())
        self.move_to_waypoint_node.cmd.set("move_line %f %f %f" % tuple(current_waypoint))

    def remove_waypoint(self):
        self.waypoints.set(self.waypoints.get()[1:])


class TestBT(object):
    def __init__(self):
        rospy.init_node("test_bt_node")
        self.move_publisher = rospy.Publisher("/navigation/move_command", String, queue_size=100)
        self.move_client = bt_ros.ActionClient(self.move_publisher)
        rospy.Subscriber("/response", String, self.move_client.response_callback)

        rospy.sleep(1)
        self.bt = bt.Root(
                    CollectChaos(
                        np.array([[1.7, 0.8, 1, 1, 0, 0],
                                    [1.9, 0.9, 2, 0, 1, 0],
                                    [2.1, 0.85, 3, 0, 0, 1],
                                    [1.9, 1.1, 4, 1, 0, 0]]),
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
        node = TestBT()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
