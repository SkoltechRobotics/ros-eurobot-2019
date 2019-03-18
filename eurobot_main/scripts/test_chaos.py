import rospy
import behavior_tree as bt
import bt_ros
import numpy as np
from std_msgs.msg import String


class MoveWaypoints(bt.FallbackNode):
    def __init__(self, waypoints, action_client_id):
        # Init parameters
        self.waypoints = bt.BTVariable(waypoints)

        # Init useful child nodes
        self.move_to_waypoint_node = ActionClientNode("move 0 0 0", action_client_id, name="move_to_waypoint")
        self.choose_new_waypoint_latch = bt.Latch(bt.ActionNode(self.choose_new_waypoint))

        # Make BT
        super(MoveWaypoints, self).__init__([
            bt.ConditionNode(self.is_waypoints_empty),
            bt.SequenceNode([
                self.choose_new_waypoint_latch,
                self.move_to_waypoint_node,
                bt.ActionNode(self.remove_waypoint),
                bt.ActionNode(self.choose_new_waypoint_latch.reset),
                bt.ConditionNode(lambda: bt.Status.RUNNING)
            ]),
        ])


class TestBT(object):
    def __init__(self):
        rospy.init_node("test_bt_node")
        self.move_publisher = rospy.Publisher("/navigation/move_command", String, queue_size=100)
        self.move_client = bt_ros.ActionClient(self.move_publisher)
        rospy.Subscriber("/response", String, self.move_client.response_callback)

        rospy.sleep(1)
        self.bt = bt.Root(
            bt_ros.MoveWaypoints([
                np.array([1, 1, 1]),
                np.array([2, 1, 1]),
                np.array([2, 2, 1]),
            ], "move_client"), action_clients={"move_client": self.move_client})
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
