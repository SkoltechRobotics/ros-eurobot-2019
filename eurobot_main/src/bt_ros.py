import rospy
import threading
import behavior_tree as bt
from bt_parameters import BTParameter


class ActionClient(object):
    cmd_id = 0

    def __init__(self, cmd_publisher):
        self.cmd_publisher = cmd_publisher
        self.cmd_statuses = {}
        self.mutex = threading.Lock()

    def response_callback(self, data):
        self.mutex.acquire()
        data_splitted = data.data.split()
        cmd_id = data_splitted[0]
        status = data_splitted[1]
        if cmd_id in self.cmd_statuses:
            self.cmd_statuses[cmd_id] = status
        self.mutex.release()

    def set_cmd(self, cmd, cmd_id=None):
        self.mutex.acquire()
        if cmd_id is None:
            cmd_id = str(self.cmd_id)
        self.cmd_id += 1
        self.cmd_statuses = "running"
        self.cmd_publisher.publish(cmd_id + " " + cmd)
        self.mutex.release()
        return cmd_id

    def get_status(self, cmd_id):
        with self.mutex:
            status = self.cmd_statuses[cmd_id]
        return status


class ActionClientNode(bt.SequenceNode):
    def __init__(self, cmd, action_client_id):
        self.action_client_id = action_client_id
        self.cmd = BTParameter(cmd)
        self.cmd_id = None

        self.start_move_node = bt.Latch(bt.ActionNode(self.start_action))
        bt.SequenceNode.__init__(self, [self.start_move_node, bt.ConditionNode(self.is_action_running),
                                        bt.ConditionNode(self.is_action_success)])

    def start_action(self):
        self.cmd_id = self.root.action_clients[self.action_client_id].set_cmd(self.cmd.get())

    def is_action_running(self):
        status = self.root.action_clients[self.action_client_id].get_status(self.cmd_id)
        if status == "running":
            return bt.Status.RUNNING
        else:
            return bt.Status.SUCCESS

    def is_action_success(self):
        status = self.root.action_clients[self.action_client_id].get_status(self.cmd_id)
        if status == "success":
            return bt.Status.SUCCESS
        else:
            return bt.Status.FAILED

    def reset(self):
        self.start_move_node.reset()


