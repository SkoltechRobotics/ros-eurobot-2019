import rospy
import threading
import behavior_tree as bt


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
        if cmd_id in self.cmd_statuses.keys():
            self.cmd_statuses[cmd_id] = status
        self.mutex.release()

    def set_cmd(self, cmd, cmd_id=None):
        self.mutex.acquire()
        if cmd_id is None:
            cmd_id = str(self.cmd_id)
        self.cmd_id += 1
        self.cmd_statuses[cmd_id] = "running"
        self.cmd_publisher.publish(cmd_id + " " + cmd)
        self.mutex.release()
        return cmd_id

    def get_status(self, cmd_id):
        with self.mutex:
            status = self.cmd_statuses[cmd_id]
        return status


class ActionClientNode(bt.SequenceNode):
    def __init__(self, cmd, action_client_id, **kwargs):
        self.action_client_id = action_client_id
        self.cmd = bt.BTVariable(cmd)
        self.cmd_id = bt.BTVariable()

        self.start_node = bt.Latch(bt.ActionNode(self.start_action))
        bt.SequenceNode.__init__(self, [self.start_node, bt.ConditionNode(self.action_status)], **kwargs)

    def start_action(self): 
        """
        action_clients: {}

        :return:
        """
        print("Start BT Action: " + self.cmd.get())
        self.cmd_id.set(self.root.action_clients[self.action_client_id].set_cmd(self.cmd.get()))

    def action_status(self):
        status = self.root.action_clients[self.action_client_id].get_status(self.cmd_id.get())
        if status == "running":
            return bt.Status.RUNNING
        elif status == "success":
            return bt.Status.SUCCESS
        else:
            return bt.Status.FAILED

    def reset(self):
        self.start_node.reset()

    def log(self, level, prefix=""):
        bt.BTNode.log(self, level, prefix)

class STMClientNode(bt.SequenceNode):
    def __init__(self, action_client_id):
        self.action_client_id = action_client_id
        self.cmd = bt.BTVariable(cmd)
        self.cmd_id = bt.BTVariable()


        self.start_node = bt.ActionNode(self.send_command)
        bt.FallbackNode.__init__(self, [bt.ConditionNode(self.action_status), self.start_node], **kwargs)

    def send_command(self): 
        self.cmd_id.set(self.root.action_clients[self.action_client_id].set_cmd(self.cmd.get()))

    def action_status(self):
        status = self.root.action_clients[self.action_client_id].get_status(self.cmd_id.get())
        if status == "0":
            return bt.Status.RUNNING
        elif status == "1":
            return bt.Status.SUCCESS
        else:
            return bt.Status.FAILED

    def reset(self):
        self.start_node.reset()

    def log(self, level, prefix=""):
        bt.BTNode.log(self, level, prefix)


class SetToDefaultState(ActionClientNode):
    def __init__(self, action_client_id):
        cmd = "default"
        super(SetToDefaultState, self).__init__(cmd, action_client_id)


class SetManipulatortoWall(ActionClientNode):
    def __init__(self, action_client_id):
        cmd = "manipulator_wall"
        super(SetManipulatortoWall, self).__init__(cmd, action_client_id)


class SetManipulatortoUp(ActionClientNode):
    def __init__(self, action_client_id):
        cmd = "manipulator_up"
        super(SetManipulatortoUp, self).__init__(cmd, action_client_id)


class StartTakeWallPuck(ActionClientNode):
    def __init__(self, action_client_id):
        cmd = "start_collect_wall"
        super(StartTakeWallPuck, self).__init__(cmd, action_client_id)


class CompleteTakeWallPuck(ActionClientNode):
    def __init__(self, action_client_id):
        cmd = "complete_collect_wall"
        super(CompleteTakeWallPuck, self).__init__(cmd, action_client_id)


class MoveLineToPoint(ActionClientNode):
    def __init__(self, point, action_client_id):
        cmd = "move_line " + str(point[0]) + " " + str(point[1]) + " " + str(point[2])
        super(MoveLineToPoint, self).__init__(cmd, action_client_id)


class MoveArcToPoint(ActionClientNode):
    def __init__(self, point, action_client_id):
        cmd = "move_arc " + str(point[0]) + " " + str(point[1]) + " " + str(point[2])
        super(MoveArcToPoint, self).__init__(cmd, action_client_id)


# ===========================================================


# Commands for collecting ground pucks
class StartCollectGround(ActionClientNode):
    def __init__(self, action_client_id):
        cmd = "take_ground"  # FIXME
        super(StartCollectGround, self).__init__(cmd, action_client_id)


class CompleteCollectGround(ActionClientNode):
    def __init__(self, action_client_id):
        cmd = "complete_ground_collect"  # FIXME
        super(CompleteCollectGround, self).__init__(cmd, action_client_id)


# ===========================================================

# TODO
# Command to push Blunium in Accelerator
class PushBlunium(ActionClientNode):
    def __init__(self, action_client_id):
        cmd = "push_blunium"
        super(PushBlunium, self).__init__(cmd, action_client_id)


# Command to unload in Accelerator
class UnloadAccelerator(ActionClientNode):
    def __init__(self, action_client_id):
        cmd = "release_accelerator"
        super(UnloadAccelerator, self).__init__(cmd, action_client_id)


# Command to grab Goldenium
class GrabGoldenium(ActionClientNode):
    def __init__(self, action_client_id):
        cmd = "grab_goldenium"
        super(GrabGoldenium, self).__init__(cmd, action_client_id)


# Command to unload Goldenium on Scales
class UnloadGoldenium(ActionClientNode):
    def __init__(self, action_client_id):
        cmd = "release_goldenium_on_scales"
        super(UnloadGoldenium, self).__init__(cmd, action_client_id)


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

    def is_waypoints_empty(self):
        if len(self.waypoints.get()) > 0:
            return bt.Status.FAILED
        else:
            return bt.Status.SUCCESS

    def choose_new_waypoint(self):
        current_waypoint = self.waypoints.get()[0]
        print(self.waypoints.get())
        self.move_to_waypoint_node.cmd.set("move_line %f %f %f" % tuple(current_waypoint))

    def remove_waypoint(self):
        self.waypoints.set(self.waypoints.get()[1:])
