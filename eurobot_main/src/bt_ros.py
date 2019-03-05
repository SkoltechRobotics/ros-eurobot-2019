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

        self.start_move_node = bt.Latch(bt.ActionNode(self.start_action))
        bt.SequenceNode.__init__(self, [self.start_move_node, bt.ConditionNode(self.action_status)], **kwargs)

    def start_action(self):
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
        self.start_move_node.reset()

    def log(self, level):
        bt.BTNode.log(self, level)


class MoveToPoint(ActionClientNode):
    def __init__(self, point, action_client_id):
        cmd = "move_line " + str(point[0]) + " " + str(point[1]) + " " + str(point[2])
        super(MoveToPoint, self).__init__(cmd, action_client_id)

# class MoveToPoint(bt.FallbackNode):
#     def __init__(self, point, action_client_id):
#         self.point = bt.BTVariable(point)
#
#         self.move_to_point_node = bt.Latch(ActionClientNode("move " + point[0] + " " + point[1] + " " + point[2],
#                                                             action_client_id, name="move_to_point"))
#         super(MoveToPoint, self).__init__([
#             bt.ConditionNode(self.is_reached),
#             bt.ActionNode(self.move_to_point)
#         ])
#
#     def is_reached(self):
#         if self.move_to_point_node.action_status() == bt.Status.SUCCESS:
#             return True
#         else:
#             return False
#
#     def move_to_point(self):
#         self.move_to_point_node.children[0].start_action()


class TakeWallPuck(ActionClientNode):
    def __init__(self, action_client_id):
        cmd = "collect_wall"
        super(TakeWallPuck, self).__init__(cmd, action_client_id)


# class TakeWallPuck(bt.FallbackNode):
#     def __init__(self, action_client_id):
#         self.take_wall_puck_node = ActionClientNode("take wall puck", action_client_id,
#                                                     name="take_wall_puck")
#         super(TakeWallPuck, self).__init__([
#             bt.ConditionNode(self.is_taken),
#             bt.ActionNode(self.take_puck)
#         ])
#
#     def is_taken(self):
#         if self.take_wall_puck_node.action_status() == bt.Status.SUCCESS:
#             return True
#         else:
#             return False
#
#     def take_puck(self):
#         self.take_wall_puck_node.start_action()


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