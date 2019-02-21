import enum
import threading
from termcolor import colored


class Status(enum.Enum):
    SUCCESS = 0
    FAILED = 1
    RUNNING = 2


class BTVariable(object):
    def __init__(self, default_data=None):
        self.data = default_data
        self.mutex = threading.Lock()

    def set(self, data):
        with self.mutex:
            self.data = data

    def get(self):
        with self.mutex:
            data = self.data
        return data


class BTNode(object):
    """
        Base class for all possible nodes in tree.
    """
    def __init__(self, name=None):
        self.status = Status.SUCCESS
        self.parent = None
        self.root = None
        self.is_parent_set = False
        self.name = name

    def set_parent(self, parent):
        assert not self.is_parent_set
        self.parent = parent
        self.root = parent.root
        self.is_parent_set = True

    def tick(self):
        return self.status

    def log(self, level):
        colors = {Status.SUCCESS: "green",
                  Status.FAILED: "red",
                  Status.RUNNING: "blue",
                  }
        if self.name is None:
            name = self.__class__.__name__
        else:
            name = self.name
        print level * "    " + name + " ---> " + colored(str(self.status), colors[self.status])


class ControlNode(BTNode):
    def __init__(self, children, **kwargs):
        BTNode.__init__(self, **kwargs)
        self.children = children
        for child in self.children:
            assert isinstance(child, BTNode)

    def set_parent(self, parent):
        BTNode.set_parent(self, parent)
        for child in self.children:
            child.set_parent(self)

    def log(self, level):
        BTNode.log(self, level)
        for child in self.children:
            child.log(level + 1)


class SequenceNode(ControlNode):
    def __init__(self, children, **kwargs):
        ControlNode.__init__(self, children, **kwargs)

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
    def __init__(self, children, **kwargs):
        ControlNode.__init__(self, children, **kwargs)

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
    def __init__(self, child, **kwargs):
        ControlNode.__init__(self, [child], **kwargs)
        self.is_init = BTVariable(False)

    def tick(self):
        if not self.is_init.get():
            self.status = self.children[0].tick()
        if self.status == Status.FAILED or self.status == Status.SUCCESS:
            self.is_init.set(True)
        return self.status

    def reset(self):
        self.is_init.set(False)


class ActionNode(BTNode):
    def __init__(self, function, **kwargs):
        assert callable(function)
        self.function = function
        BTNode.__init__(self, **kwargs)

    def tick(self):
        self.function()
        return Status.SUCCESS


class ConditionNode(BTNode):
    def __init__(self, function, **kwargs):
        assert callable(function)
        self.function = function
        BTNode.__init__(self, **kwargs)

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
        child.set_parent(self)

    def tick(self):
        self.status = self.children[0].tick()
        return self.status

    def log(self, level):
        self.children[0].log(0)
