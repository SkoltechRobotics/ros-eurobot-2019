import rospy
import threading
import behavior_tree as bt
from core_functions import *
import numpy as np

import tf2_ros
from tf.transformations import euler_from_quaternion

class ActionClient(object):

    def __init__(self, cmd_publisher):
        self.cmd_id = 0
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


class SetToDefaultState(ActionClientNode):
    def __init__(self, action_client_id):
        cmd = "default"
        super(SetToDefaultState, self).__init__(cmd, action_client_id)


class SetManipulatortoWall(ActionClientNode):
    def __init__(self, action_client_id):
        cmd = "manipulator_wall"
        super(SetManipulatortoWall, self).__init__(cmd, action_client_id)


class SetManipulatortoUp(ActionClientNode):  # FIXME SetManipulatorUp
    def __init__(self, action_client_id):
        cmd = "manipulator_up"
        super(SetManipulatortoUp, self).__init__(cmd, action_client_id)


class StartTakeWallPuck(ActionClientNode):
    def __init__(self, action_client_id):
        cmd = "start_collect_wall"
        super(StartTakeWallPuck, self).__init__(cmd, action_client_id)

class StartTakeWallPuckPlatform(ActionClientNode):
    def __init__(self, action_client_id):
        cmd = "StartTakeWallPuckPlatform"
        super(StartTakeWallPuckPlatform, self).__init__(cmd, action_client_id)

class CompleteTakeWallPuck(ActionClientNode):
    def __init__(self, action_client_id):
        cmd = "complete_collect_wall"
        super(CompleteTakeWallPuck, self).__init__(cmd, action_client_id)

class CompleteCollectLastWall(ActionClientNode):
    def __init__(self, action_client_id):
        cmd = "complete_collect_last_wall"
        super(CompleteCollectLastWall, self).__init__(cmd, action_client_id)


class CompleteCollectLastPuck(ActionClientNode):
    def __init__(self, action_client_id):
        cmd = "complete_collect_last_puck"
        super(CompleteCollectLastPuck, self).__init__(cmd, action_client_id)


class MoveLineToPoint(ActionClientNode):
    def __init__(self, point, action_client_id):
        cmd = "move_line " + str(point[0]) + " " + str(point[1]) + " " + str(point[2])
        super(MoveLineToPoint, self).__init__(cmd, action_client_id)


class MoveArcToPoint(ActionClientNode):
    def __init__(self, point, action_client_id):
        cmd = "move_arc " + str(point[0]) + " " + str(point[1]) + " " + str(point[2])
        super(MoveArcToPoint, self).__init__(cmd, action_client_id)


class ReleaseFivePucks(ActionClientNode):
    def __init__(self, action_client_id):
        cmd = "release_5"
        super(ReleaseFivePucks, self).__init__(cmd, action_client_id)

# ===========================================================


# Commands for collecting ground pucks
class StartCollectGround(ActionClientNode):
    def __init__(self, action_client_id):
        cmd = "start_collect_ground"
        super(StartCollectGround, self).__init__(cmd, action_client_id)


class CompleteCollectGround(ActionClientNode):
    def __init__(self, action_client_id):
        cmd = "complete_collect_ground"
        super(CompleteCollectGround, self).__init__(cmd, action_client_id)


# ===========================================================


class StartCollectBlunium(ActionClientNode):
    def __init__(self, action_client_id):
        cmd = "start_collect_blunium"
        super(StartCollectBlunium, self).__init__(cmd, action_client_id)


# Command to unload in Accelerator
class UnloadAccelerator(ActionClientNode):
    def __init__(self, action_client_id):
        cmd = "release_accelerator"
        super(UnloadAccelerator, self).__init__(cmd, action_client_id)


# Command to grab Goldenium, push up and hold it there
class GrabGoldeniumAndHoldUp(ActionClientNode):
    def __init__(self, action_client_id):
        cmd = "grab_goldenium_and_hold_up"
        super(GrabGoldeniumAndHoldUp, self).__init__(cmd, action_client_id)


# Command to unload Goldenium on Scales
class UnloadGoldenium(ActionClientNode):
    def __init__(self, action_client_id):
        cmd = "release_goldenium_on_scales"
        super(UnloadGoldenium, self).__init__(cmd, action_client_id)


class StepUp(ActionClientNode):
    def __init__(self, action_client_id):
        cmd = "stepper_step_up"
        super(StepUp, self).__init__(cmd, action_client_id)


class SetManipulatortoGoldenium(ActionClientNode):
    def __init__(self, action_client_id):
        cmd = "set_angle_to_grab_goldenium"
        super(SetManipulatortoGoldenium, self).__init__(cmd, action_client_id)


class SetToWall_ifReachedGoal(bt.SequenceNode):
    def __init__(self, goal, action_client_id, threshold=0.3):
        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer)

        self.robot_name = rospy.get_param("robot_name")
        self.robot_coordinates = None

        self.goal = goal
        self.threshold = bt.BTVariable(threshold)

        self.set_to_wall_node = ActionClientNode("manipulator_wall", action_client_id, name="manipulator_to_wall")

        super(SetToWall_ifReachedGoal, self).__init__([
            bt.ConditionNode(self.is_coordinates_reached),
            self.set_to_wall_node
        ])

    def update_coordinates(self):
        try:
            trans = self.tfBuffer.lookup_transform('map', self.robot_name, rospy.Time(0))
            q = [trans.transform.rotation.x,
                 trans.transform.rotation.y,
                 trans.transform.rotation.z,
                 trans.transform.rotation.w]
            angle = euler_from_quaternion(q)[2] % (2 * np.pi)
            self.robot_coordinates = np.array([trans.transform.translation.x,
                                               trans.transform.translation.y,
                                               angle])
            rospy.loginfo("Robot coords:\t" + str(self.robot_coordinates))
            return True
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as msg:
            rospy.logwarn(str(msg))
            rospy.logwarn("SMT WROND QQ")
            return False

    def is_coordinates_reached(self):
        # FIXME:: self.update_coordinates() replace???!!!
        self.update_coordinates()
        distance, _ = calculate_distance(self.robot_coordinates, self.goal)
        norm_distance = np.linalg.norm(distance)
        print ("DISTANCE=",norm_distance)
        if norm_distance < self.threshold:
            return bt.Status.SUCCESS
        else:
            return bt.Status.RUNNING


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
