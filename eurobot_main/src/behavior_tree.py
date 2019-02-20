import rospy
import enum
from bt_parameters import BTParameter


class Status(enum.Enum):
    SUCCESS = 0
    FAILED = 1
    RUNNING = 2


class BTNode(object):
    """
        Base class for all possible nodes in tree.
    """
    def __init__(self):
        self.status = Status.SUCCESS
        self.parent = None
        self.root = None
        self.is_parent_set = False

    def set_parent(self, parent):
        assert not self.is_parent_set
        self.parent = parent
        self.root = parent.root
        self.is_parent_set = True

    def tick(self):
        return self.status


class ControlNode(BTNode):
    def __init__(self, children):
        BTNode.__init__(self)
        self.children = children
        for child in self.children:
            child.set_parent(self)


class SequenceNode(ControlNode):
    def __init__(self, children):
        ControlNode.__init__(self, children)

    def tick(self):
        self.status = Status.SUCCESS
        for child in self.children:
            status = child.tick()
            if status == Status.FAILED:
                self.status = Status.FAILED
                break
            if status == Status.RUNNING:
                self.status = Status.RUNNING
                break
        if self.status == Status.FAILED or self.status == Status.SUCCESS:
            self.reset()
        return self.status

    def reset(self):
        pass


class FallbackNode(ControlNode):
    def __init__(self, children):
        ControlNode.__init__(self, children)

    def tick(self):
        self.status = Status.FAILED
        for child in self.children:
            status = child.tick()
            if status == Status.SUCCESS:
                self.status = Status.SUCCESS
                break
            if status == Status.RUNNING:
                self.status = Status.RUNNING
                break
        if self.status == Status.FAILED or self.status == Status.SUCCESS:
            self.reset()
        return self.status

    def reset(self):
        pass


class Latch(ControlNode):
    def __init__(self, child):
        ControlNode.__init__(self, [child])
        self.is_init = BTParameter(False)

    def tick(self):
        if self.is_init.get():
            self.status = self.children[0].tick()
        if self.status == Status.FAILED or self.status == Status.SUCCESS:
            self.is_init.set(True)
        return self.status

    def reset(self):
        self.is_init.set(False)


class ActionNode(BTNode):
    def __init__(self, function):
        self.function = function
        BTNode.__init__(self)

    def tick(self):
        self.function()
        return Status.SUCCESS


class ConditionNode(BTNode):
    def __init__(self, function):
        self.function = function
        BTNode.__init__(self)

    def tick(self):
        self.status = self.function()
        return self.status


class Root(BTNode):
    def __init__(self, child, action_clients=None):
        BTNode.__init__(self)
        self.root = self
        self.children = [child]
        if action_clients is not None:
            self.action_clients = action_clients
        else:
            self.action_clients = {}

    def tick(self):
        self.status = self.children[0].tick()
        return self.status


def log_node(node, level):
    print(level * "    " + node.__class__.__name__ + " ---> " + str(node.status))
    if isinstance(node, ControlNode):
        for child in node.children:
            log_node(child, level + 1)


def log_bt(bt):
    print("============== BT LOG ================")
    log_node(bt, 0)
