import rospy
import enum


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

    def tick(self):
        return self.status


class InternalNode(BTNode):
    def __init__(self, children):
        BTNode.__init__(self)
        self.children = children


class SequenceNode(InternalNode):
    def __init__(self, children):
        InternalNode.__init__(self, children)

    def tick(self):
        for child in self.children:
            status = child.tick()
            if status == Status.FAILED:
                return Status.FAILED
            if status == Status.RUNNING:
                return Status.RUNNING
        return Status.SUCCESS


class SequenceNodeWithMemory(InternalNode):
    def __init__(self, children):
        InternalNode.__init__(self, children)
        self.current_child_ind = 0

    def reset(self):
        self.current_child_ind = 0

    def tick(self):
        while self.current_child_ind < len(self.children):
            status = self.children[self.current_child_ind].tick()
            if status == Status.RUNNING:
                return Status.RUNNING
            elif status == Status.FAILED:
                self.reset()
                return Status.FAILED
            elif status == Status.SUCCESS:
                self.current_child_ind += 1
        self.reset()
        return Status.SUCCESS
