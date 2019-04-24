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

#-----------------------------


class Calibrate(ActionClientNode):
    def __init__(self, action_client_id):
        cmd = "calibrate"
        super(Calibrate, self).__init__(cmd, action_client_id)


class SetManipulatortoWall(ActionClientNode):
    def __init__(self, action_client_id):
        cmd = "manipulator_wall"
        super(SetManipulatortoWall, self).__init__(cmd, action_client_id)


class SetManipulatortoUp(ActionClientNode):  # FIXME SetManipulatorUp
    def __init__(self, action_client_id):
        cmd = "manipulator_up"
        super(SetManipulatortoUp, self).__init__(cmd, action_client_id)


class SetManipulatortoGround(ActionClientNode):
    def __init__(self, action_client_id):
        cmd = "manipulator_ground"
        super(SetManipulatortoGround, self).__init__(cmd, action_client_id)


class MainSetManipulatortoGround(ActionClientNode):
    def __init__(self, action_client_id):
        cmd = "set_manipulator_ground_main"
        super(MainSetManipulatortoGround, self).__init__(cmd, action_client_id)


class StartPump(ActionClientNode):
    def __init__(self, action_client_id):
        cmd = "start_pump"
        super(StartPump, self).__init__(cmd, action_client_id)


class StopPump(ActionClientNode):
    def __init__(self, action_client_id):
        cmd = "stop_pump"
        super(StopPump, self).__init__(cmd, action_client_id)

class Delay500(ActionClientNode):
    def __init__(self, action_client_id):
        cmd = "delay_500"
        super(Delay500, self).__init__(cmd, action_client_id)

# ===========================================================
class MovingDefault(ActionClientNode):
    def __init__(self, action_client_id):
        cmd = "moving_default"
        super(MovingDefault, self).__init__(cmd, action_client_id)


class MoveLineToPoint(ActionClientNode):
    def __init__(self, point, action_client_id):
        rospy.loginfo(point)
        cmd = "move_line " + str(point[0]) + " " + str(point[1]) + " " + str(point[2])
        super(MoveLineToPoint, self).__init__(cmd, action_client_id)


class SlowMoveLineToPoint(ActionClientNode):
    def __init__(self, point, action_client_id):
        cmd = "slow_move_line " + str(point[0]) + " " + str(point[1]) + " " + str(point[2])
        super(SlowMoveLineToPoint, self).__init__(cmd, action_client_id)


class MoveArcToPoint(ActionClientNode):
    def __init__(self, point, action_client_id):
        cmd = "move_arc " + str(point[0]) + " " + str(point[1]) + " " + str(point[2])
        super(MoveArcToPoint, self).__init__(cmd, action_client_id)


class SetSpeedSTM(ActionClientNode):
    def __init__(self, speed, time, action_client_id):
        self.delay = time
        self.speed = speed
        cmd = "8 " + str(self.speed[0]) + " " + str(self.speed[1]) + " " + str(self.speed[2])
        super(SetSpeedSTM, self).__init__(cmd, action_client_id)

    def start_action(self):
        print("Start BT Action: " + self.cmd.get())
        self.cmd_id.set(self.root.action_clients[self.action_client_id].set_cmd(self.cmd.get()))
        rospy.sleep(self.delay)
        self.cmd_id.set(self.root.action_clients[self.action_client_id].set_cmd("8 0 0 0"))
        # rospy.sleep(self.delay/2)
        # self.cmd_id.set(self.root.action_clients[self.action_client_id].set_cmd("8 0 -0.05 0"))  # "8 " + str(-1*self.speed[0]) + "0 " + "0"
        # rospy.sleep(self.delay*2)
        # self.cmd_id.set(self.root.action_clients[self.action_client_id].set_cmd("8 0 0 0"))
        # rospy.sleep(self.delay/2)

    def action_status(self):
        status = self.root.action_clients[self.action_client_id].get_status(self.cmd_id.get())
        return bt.Status.SUCCESS

# ===========================================================


class CheckLimitSwitchInf(ActionClientNode):
    def __init__(self, action_client_id):
        cmd = "check_limit switch_infinitely"
        super(CheckLimitSwitchInf, self).__init__(cmd, action_client_id)


class StartTakeWallPuck(ActionClientNode):
    def __init__(self, action_client_id):
        cmd = "start_collect_wall"
        super(StartTakeWallPuck, self).__init__(cmd, action_client_id)


class CompleteCollectLastPuck(ActionClientNode):
    def __init__(self, action_client_id):
        cmd = "complete_collect_last_puck"
        super(CompleteCollectLastPuck, self).__init__(cmd, action_client_id)


class StartTakeWallPuckWithoutGrabber(ActionClientNode):
    def __init__(self, action_client_id):
        cmd = "start_collect_wall_without_grabber"
        super(StartTakeWallPuckWithoutGrabber, self).__init__(cmd, action_client_id)


class ReleaseFromManipulator(ActionClientNode):
    def __init__(self, action_client_id):
        cmd = "release_from_manipulator"
        super(ReleaseFromManipulator, self).__init__(cmd, action_client_id)

# ===========================================================
# Commands for collecting ground pucks
class StartCollectGround(ActionClientNode):
    def __init__(self, action_client_id):
        cmd = "start_collect_ground"
        super(StartCollectGround, self).__init__(cmd, action_client_id)


class BlindStartCollectGround(ActionClientNode):
    def __init__(self, action_client_id):
        cmd = "blind_start_collect_ground"
        super(BlindStartCollectGround, self).__init__(cmd, action_client_id)


class CompleteCollectGround(ActionClientNode):
    def __init__(self, action_client_id):
        cmd = "complete_collect_ground"
        super(CompleteCollectGround, self).__init__(cmd, action_client_id)


class StartCollectBlunium(ActionClientNode):
    def __init__(self, action_client_id):
        cmd = "start_collect_blunium"
        super(StartCollectBlunium, self).__init__(cmd, action_client_id)


class FinishCollectBlunium(ActionClientNode):
    def __init__(self, action_client_id):
        cmd = "finish_collect_blunium"
        super(FinishCollectBlunium, self).__init__(cmd, action_client_id)


class UnloadAccelerator(ActionClientNode):
    def __init__(self, action_client_id):
        cmd = "release_accelerator"
        super(UnloadAccelerator, self).__init__(cmd, action_client_id)


class StartCollectGoldenium(ActionClientNode):
    def __init__(self, action_client_id):
        cmd = "start_collect_goldenium"
        super(StartCollectGoldenium, self).__init__(cmd, action_client_id)


# Command to grab Goldenium, push up and hold it there
class GrabGoldeniumAndHoldUp(ActionClientNode):
    def __init__(self, action_client_id):
        cmd = "grab_goldenium_and_hold_up"
        super(GrabGoldeniumAndHoldUp, self).__init__(cmd, action_client_id)


class UnloadGoldenium(ActionClientNode):
    def __init__(self, action_client_id):
        cmd = "release_goldenium_on_scales"
        super(UnloadGoldenium, self).__init__(cmd, action_client_id)

# to push blunium in acc
class SetManipulatorToPushBlunium(ActionClientNode):
    def __init__(self, action_client_id):
        cmd = "swing_puck"
        super(SetManipulatorToPushBlunium, self).__init__(cmd, action_client_id)


class ReleaseInAccFirstMove(ActionClientNode):
    def __init__(self, action_client_id):
        cmd = "release_accelerator_first_move_when_full"
        super(ReleaseInAccFirstMove, self).__init__(cmd, action_client_id)
# ===========================================================
#-------------
class CompleteTakePuckAndMoveToNext(bt.ParallelWithMemoryNode):
    def __init__(self, current_puck_coordinates, next_puck_coordinates, score_master, puck_type):
        super(CompleteTakePuckAndMoveToNext, self).__init__([
            bt.SequenceWithMemoryNode([
                MoveLineToPoint(current_puck_coordinates + (0, -0.05, 0), "move_client"),
                MoveLineToPoint(next_puck_coordinates + (0, -0.05, 0), "move_client")
            ]),
            bt.SequenceWithMemoryNode([
                CompleteTakeWallPuck("manipulator_client"),
                bt.ActionNode(lambda: score_master.add(puck_type))
            ])
        ], threshold=2)


class MoveToNextPuckIfFailedToScales(bt.SequenceWithMemoryNode):
    def __init__(self, current_puck_coordinates, next_puck_coordinates):
        super(MoveToNextPuckIfFailedToScales, self).__init__([
            StopPump("manipulator_client"),
            MoveLineToPoint(current_puck_coordinates + (0, -0.05, 0), "move_client"),
            bt.ParallelWithMemoryNode([
                MoveLineToPoint(next_puck_coordinates + (0, -0.05, 0), "move_client"),
                SetToWall_ifReachedGoal(next_puck_coordinates + (0, -0.15, 0), "manipulator_client")
            ], threshold = 2)

        ])


class TryToPumpWallPuck(bt.FallbackWithMemoryNode):
        def __init__(self, puck_coordinates):
            super(TryToPumpWallPuck, self).__init__([
                StartTakeWallPuck("manipulator_client"),
                bt.SequenceWithMemoryNode([
                    MoveLineToPoint(puck_coordinates + (0, -0.05, 0), "move_client"),
                    MoveLineToPoint(puck_coordinates + (0, 0.02, 0), "move_client"),
                    StartTakeWallPuck("manipulator_client"),
                ])
            ])


class TryToPumpWallPuckWithoutGrabber(bt.FallbackWithMemoryNode):
    def __init__(self, puck_coordinates):
        super(TryToPumpWallPuckWithoutGrabber, self).__init__([
            StartTakeWallPuckWithoutGrabber("manipulator_client"),
            bt.SequenceWithMemoryNode([
                MoveLineToPoint(puck_coordinates + (0, -0.05, 0), "move_client"),
                MoveLineToPoint(puck_coordinates + (0, 0.02, 0), "move_client"),
                StartTakeWallPuckWithoutGrabber("manipulator_client"),
            ])
        ])


class MoveToNextPuckIfFailedToStartZone(bt.SequenceWithMemoryNode):
    def __init__(self, current_puck_coordinates, next_puck_coordinates):
        super(MoveToNextPuckIfFailedToStartZone, self).__init__([
                    StopPump("manipulator_client"),
                    MoveLineToPoint(current_puck_coordinates + (0, -0.05, 0), "move_client"),
                    bt.ParallelWithMemoryNode([
                        MoveLineToPoint(next_puck_coordinates + (0, -0.05, 0), "move_client"),
                        SetToWall_ifReachedGoal(next_puck_coordinates, "manipulator_client")
                    ], threshold=2)
        ])


# class CheckLimitSwitch(ActionClientNode):
#     def __init__(self, action_client_id):
#         cmd = "check_limit_switch_inf"
#         super(CheckLimitSwitch, self).__init__(cmd, action_client_id)


class ReleaseOnePuck(ActionClientNode):
    def __init__(self, action_client_id):
        cmd = "secondary_release_puck"
        super(ReleaseOnePuck, self).__init__(cmd, action_client_id)


class StepperUp(ActionClientNode):
    def __init__(self, action_client_id):
        cmd = "stepper_up"
        super(StepperUp, self).__init__(cmd, action_client_id)


class CompleteCollectLastWall(ActionClientNode):
    def __init__(self, action_client_id):
        cmd = "complete_collect_last_wall"
        super(CompleteCollectLastWall, self).__init__(cmd, action_client_id)


class CompleteTakeWallPuck(ActionClientNode):
    def __init__(self, action_client_id):
        cmd = "complete_collect_wall"
        super(CompleteTakeWallPuck, self).__init__(cmd, action_client_id)


class SetToGround_ifReachedGoal(bt.SequenceNode):
    def __init__(self, goal, action_client_id, threshold=0.2):
        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer)

        self.robot_name = rospy.get_param("robot_name")
        self.robot_coordinates = None

        self.goal = goal
        self.threshold = bt.BTVariable(threshold)

        self.set_to_wall_node = ActionClientNode("manipulator_ground", action_client_id, name="manipulator_to_ground")

        super(SetToGround_ifReachedGoal, self).__init__([
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
        self.update_coordinates()
        if self.robot_coordinates is None:
            return bt.Status.RUNNING
        distance, _ = calculate_distance(self.robot_coordinates, self.goal)
        norm_distance = np.linalg.norm(distance)
        if norm_distance < self.threshold.get():
            return bt.Status.SUCCESS
        else:
            return bt.Status.RUNNING


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
        self.update_coordinates()
        if self.robot_coordinates is None:
            return bt.Status.RUNNING
        distance, _ = calculate_distance(self.robot_coordinates, self.goal)
        norm_distance = np.linalg.norm(distance)
        if norm_distance < self.threshold.get():
            return bt.Status.SUCCESS
        else:
            return bt.Status.RUNNING


class PublishScore_ifReachedGoal(bt.SequenceNode):
    def __init__(self, goal, score_controller, unload_zone ,threshold=0.3):
        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer)

        self.robot_name = rospy.get_param("robot_name")
        self.robot_coordinates = None

        self.goal = goal
        self.threshold = bt.BTVariable(threshold)

        super(PublishScore_ifReachedGoal, self).__init__([
            bt.ConditionNode(self.is_coordinates_reached),
            bt.ActionNode(lambda: score_controller.unload(unload_zone))
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
        self.update_coordinates()
        if self.robot_coordinates is None:
            return bt.Status.RUNNING
        distance, _ = calculate_distance(self.robot_coordinates, self.goal)
        norm_distance = np.linalg.norm(distance)
        if norm_distance < self.threshold.get():
            return bt.Status.SUCCESS
        else:
            return bt.Status.RUNNING


class SetSpeedSTM(ActionClientNode):
    def __init__(self, speed, time, action_client_id):
        self.delay = time
        self.speed = speed
        cmd = "8 " + str(self.speed[0]) + " " + str(self.speed[1]) + " " + str(self.speed[2])
        super(SetSpeedSTM, self).__init__(cmd, action_client_id)

    def start_action(self):
        print("Start BT Action: " + self.cmd.get())
        self.cmd_id.set(self.root.action_clients[self.action_client_id].set_cmd(self.cmd.get()))
        rospy.sleep(self.delay)
        self.cmd_id.set(self.root.action_clients[self.action_client_id].set_cmd("8 0 0 0"))
        # rospy.sleep(self.delay/2)
        # self.cmd_id.set(self.root.action_clients[self.action_client_id].set_cmd("8 0 -0.05 0"))  # "8 " + str(-1*self.speed[0]) + "0 " + "0"
        # rospy.sleep(self.delay*2)
        # self.cmd_id.set(self.root.action_clients[self.action_client_id].set_cmd("8 0 0 0"))
        # rospy.sleep(self.delay/2)

    def action_status(self):
        status = self.root.action_clients[self.action_client_id].get_status(self.cmd_id.get())
        return bt.Status.SUCCESS


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
