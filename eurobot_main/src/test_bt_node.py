#! #!/usr/bin/env python
import rospy
import behavior_tree as bt
import bt_ros
from std_msgs.msg import String

class TestBTNode(bt.SequenceNode):
    def __init__(self):


class TestBT(object):
    def __init__(self):
        self.move_publisher = rospy.Publisher("move_cmd", String, queue_size=100)
        self.move_client = bt_ros.ActionClient(self.move_publisher)
        rospy.Subscriber("response", String, self.move_client.response_callback)

        self.bt = bt.Root(TestBTNode(), )


if __name__ == '__main__':
    try:
        node = TestBT()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass