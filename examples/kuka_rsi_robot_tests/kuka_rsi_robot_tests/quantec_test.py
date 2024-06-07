import copy

import rclpy
import std_srvs.srv
from builtin_interfaces.msg import Duration
from rclpy.node import Node
from lifecycle_msgs.srv import ChangeState, GetState
from lifecycle_msgs.msg import Transition, State
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from rclpy.action import ActionClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from threading import Event


class RobotManagerClient(Node):
    def __init__(self, service_group):
        super().__init__("robot_manager_client")
        self.times = None
        self.moves = None
        self.current_index = None
        self.actionFinished = None
        self.declare_parameter("start_service", False)
        self.start_service = self.get_parameter("start_service").get_parameter_value().bool_value
        self.action_client = None
        self.start_movement_service = None
        self.movement_done = Event()
        if self.start_service is True:
            self.service_group = service_group
            self.start_movement_service = self.create_service(
                std_srvs.srv.Empty,
                "start_robot_movements",
                self.start_robot_movements_callback,
                callback_group=service_group,
            )
            self.stop_movement_service = self.create_service(
                std_srvs.srv.Empty,
                "stop_robot_movements",
                self.stop_robot_movements_callback,
                callback_group=service_group,
            )
        state_client_group = MutuallyExclusiveCallbackGroup()
        self.cli = self.create_client(
            ChangeState, "/robot_manager/change_state", callback_group=state_client_group
        )
        self.state_client = self.create_client(
            GetState, "/robot_manager/get_state", callback_group=state_client_group
        )
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("service not available, waiting again...")
        self.req = ChangeState.Request()
        action_client_group = MutuallyExclusiveCallbackGroup()
        self.action_client = ActionClient(
            self,
            FollowJointTrajectory,
            "/joint_trajectory_controller/follow_joint_trajectory",
            callback_group=action_client_group,
        )
        self.create_movements()

    def create_movements(self):
        home_pos_1 = 0.0299
        home_pos_2 = -2.38422
        home_pos_3 = 2.3471
        home_pos_4 = 0.0
        home_pos_5 = 1.5943
        home_pos_6 = -2.83578
        home_pos = [home_pos_1, home_pos_2, home_pos_3, home_pos_4, home_pos_5, home_pos_6]
        # moving around joint_!
        self.moves = []
        self.times = []
        self.moves.append(
            [home_pos_1 + 0.4, home_pos_2, home_pos_3, home_pos_4, home_pos_5, home_pos_6]
        )
        self.times.append(6)
        self.moves.append(
            [home_pos_1 - 0.4, home_pos_2, home_pos_3, home_pos_4, home_pos_5, home_pos_6]
        )
        self.times.append(12)
        self.moves.append(home_pos)
        self.times.append(6)
        # joint2 only moving in one direction since we are at the limit
        self.moves.append(
            [home_pos_1, home_pos_2 + 0.4, home_pos_3, home_pos_4, home_pos_5, home_pos_6]
        )
        self.times.append(10)
        self.moves.append(home_pos)
        self.times.append(10)
        # joint3 only moving in one direction since we are at the limit
        self.moves.append(
            [home_pos_1, home_pos_2, home_pos_3 - 0.4, home_pos_4, home_pos_5, home_pos_6]
        )
        self.times.append(10)
        self.moves.append(home_pos)
        self.times.append(10)
        # joint4
        self.moves.append(
            [home_pos_1, home_pos_2, home_pos_3, home_pos_4 + 0.4, home_pos_5, home_pos_6]
        )
        self.times.append(5)
        self.moves.append(
            [home_pos_1, home_pos_2, home_pos_3, home_pos_4 - 0.4, home_pos_5, home_pos_6]
        )
        self.times.append(5)
        self.moves.append(home_pos)
        self.times.append(5)
        # joint5
        self.moves.append(
            [home_pos_1, home_pos_2, home_pos_3, home_pos_4, home_pos_5 + 0.4, home_pos_6]
        )
        self.times.append(5)
        self.moves.append(
            [home_pos_1, home_pos_2, home_pos_3, home_pos_4, home_pos_5 - 0.4, home_pos_6]
        )
        self.times.append(10)
        self.moves.append(home_pos)
        self.times.append(5)
        # joint6
        self.moves.append(
            [home_pos_1, home_pos_2, home_pos_3, home_pos_4, home_pos_5, home_pos_6 + 0.4]
        )
        self.times.append(5)
        self.moves.append(
            [home_pos_1, home_pos_2, home_pos_3, home_pos_4, home_pos_5, home_pos_6 - 0.4]
        )
        self.times.append(10)
        self.moves.append(home_pos)
        self.times.append(5)

    def send_configure(self):
        self.req.transition.id = Transition.TRANSITION_CONFIGURE
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("service not available, waiting again...")
        self.get_logger().info("calling  configure")
        future = self.cli.call_async(self.req)
        future.add_done_callback(self.configure_done_callback)

    def configure_done_callback(self, future):
        self.get_logger().info("configure done. Calling activate")
        self.send_activate()

    def send_activate(self):
        self.req.transition.id = Transition.TRANSITION_ACTIVATE
        future = self.cli.call_async(self.req)
        future.add_done_callback(self.activate_done_callback)

    def send_deactivate(self):
        self.req.transition.id = Transition.TRANSITION_DEACTIVATE
        future = self.cli.call_async(self.req)
        future.add_done_callback(self.deactivate_done_callback)

    def activate_done_callback(self, future):
        self.get_logger().info("activate done")
        self.get_state()

    def deactivate_done_callback(self, future):
        self.get_logger().info(
            "deactivate call done. If it failed, probably manual intervention is required"
        )
        self.movement_done.set()

    def activate_robot(self):
        self.send_configure()  # this will in callback send the activate

    def get_state(self):
        state_future = self.state_client.call_async(GetState.Request())
        state_future.add_done_callback(self.get_state_callback)
        self.get_logger().info("waiting for getting state")

    def get_state_callback(self, future):
        self.get_logger().info("got current state")
        state_resp = future.result()
        if state_resp.current_state.id == State.PRIMARY_STATE_ACTIVE:
            self.get_logger().info("Robot is active")
            self.current_index = 0
            self.start_next_movement()
        else:
            self.get_logger().error("could not activate robot")
            if self.start_service:
                self.movement_done.set()
            else:
                self.executor.shutdown()

    def start_next_movement(self):
        if self.current_index < len(self.times):
            self.get_logger().info("going to submit movement")
            position = self.moves[self.current_index]
            duration = self.times[self.current_index]
            self.current_index = self.current_index + 1
            self.start_robot_movement(position, duration)
        else:
            self.get_logger().info("All movements done")
            if self.start_service:
                self.movement_done.set()
            else:
                self.executor.shutdown()

    def start_robot_movement(self, position, duration):
        goal = FollowJointTrajectory.Goal()
        trajectory = JointTrajectory()
        trajectory.joint_names = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]
        goal_point = JointTrajectoryPoint()
        goal_point.time_from_start = Duration(sec=duration)
        goal_point.positions = position
        trajectory.points.append(copy.deepcopy(goal_point))
        goal.trajectory = trajectory
        self.action_client.wait_for_server()
        cur_action_submit = self.action_client.send_goal_async(goal)
        self.get_logger().info("waiting for action to be submitted")
        # self.executor.spin_until_future_complete(cur_action_submit)
        cur_action_submit.add_done_callback(self.action_submit_callback)

    def action_submit_callback(self, future):
        self.get_logger().info("action submitted, waiting for action to be done")
        result_action = future.result().get_result_async()
        result_action.add_done_callback(self.action_done_callback)

    def action_done_callback(self, future):
        self.get_logger().info("action finished in callback")
        self.start_next_movement()

    def start_robot_movements(self):
        self.get_logger().info("Robot movement starting")
        home_pos_1 = 0.0
        home_pos_2 = -2.3911
        home_pos_3 = 2.6529
        home_pos_4 = 0.0
        home_pos_5 = 1.308
        home_pos_6 = 0.0
        home_pos = [home_pos_1, home_pos_2, home_pos_3, home_pos_4, home_pos_5, home_pos_6]
        # moving around joint_!
        self.start_robot_movement(
            [home_pos_1 + 0.4, home_pos_2, home_pos_3, home_pos_4, home_pos_5, home_pos_6], 6
        )
        self.start_robot_movement(
            [home_pos_1 - 0.4, home_pos_2, home_pos_3, home_pos_4, home_pos_5, home_pos_6], 12
        )
        self.start_robot_movement(home_pos, 6)
        # joint2 only moving in one direction since we are at the limit
        self.start_robot_movement(
            [home_pos_1, home_pos_2 + 0.4, home_pos_3, home_pos_4, home_pos_5, home_pos_6], 10
        )
        self.start_robot_movement(home_pos, 10)
        # joint3 only moving in one direction since we are at the limit
        self.start_robot_movement(
            [home_pos_1, home_pos_2, home_pos_3 - 0.4, home_pos_4, home_pos_5, home_pos_6], 10
        )
        self.start_robot_movement(home_pos, 10)
        # joint4
        self.start_robot_movement(
            [home_pos_1, home_pos_2, home_pos_3, home_pos_4 + 0.4, home_pos_5, home_pos_6], 5
        )
        self.start_robot_movement(
            [home_pos_1, home_pos_2, home_pos_3, home_pos_4 - 0.4, home_pos_5, home_pos_6], 5
        )
        self.start_robot_movement(home_pos, 5)
        # joint5
        self.start_robot_movement(
            [home_pos_1, home_pos_2, home_pos_3, home_pos_4, home_pos_5 + 0.4, home_pos_6], 5
        )
        self.start_robot_movement(
            [home_pos_1, home_pos_2, home_pos_3, home_pos_4, home_pos_5 - 0.4, home_pos_6], 10
        )
        self.start_robot_movement(home_pos, 5)
        # joint6
        self.start_robot_movement(
            [home_pos_1, home_pos_2, home_pos_3, home_pos_4, home_pos_5, home_pos_6 + 0.4], 5
        )
        self.start_robot_movement(
            [home_pos_1, home_pos_2, home_pos_3, home_pos_4, home_pos_5, home_pos_6 - 0.4], 10
        )
        self.start_robot_movement(home_pos, 5)

    def do_robot_movements(self):
        self.activate_robot()

    def start_robot_movements_callback(self, request, response):
        self.get_logger().info("Processing start movement service call")
        self.movement_done.clear()
        self.do_robot_movements()
        self.get_logger().info("waiting for all actions to be done")
        self.movement_done.wait()
        self.get_logger().info("service returning")
        return response

    def stop_robot_movements_callback(self, request, response):
        self.get_logger().info("Processing start movement service call")
        self.movement_done.clear()
        self.send_deactivate()
        self.movement_done.wait()
        return response


def main(args=None):
    rclpy.init(args=args)
    service_group = rclpy.callback_groups.MutuallyExclusiveCallbackGroup()
    robot_manager_client = RobotManagerClient(service_group)
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(robot_manager_client)
    if robot_manager_client.start_service is False:
        robot_manager_client.do_robot_movements()
        executor.spin()
        robot_manager_client.destroy_node()
    else:
        executor.spin()
    rclpy.shutdown()


if __name__ == "__main__":
    main()