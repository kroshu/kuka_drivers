import copy

import rclpy
import std_srvs.srv
from builtin_interfaces.msg import Duration
from rclpy import Future
from rclpy.node import Node
from lifecycle_msgs.srv import ChangeState, GetState
from lifecycle_msgs.msg import Transition, State
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from rclpy.action import ActionClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup


class RobotManagerClient(Node):
    def __init__(self, service_group):
        super().__init__('robot_manager_client')
        self.actionFinished = None
        self.declare_parameter('start_service', False)
        self.start_service = self.get_parameter('start_service').get_parameter_value().bool_value
        self.action_client = None
        self.movement_service = None
        if self.start_service is True:
            self.service_group = service_group
            self.movement_service = self.create_service(std_srvs.srv.Empty, 'start_robot_movements',
                                                        self.start_robot_movements_callback,
                                                        callback_group=service_group)
        state_client_group = MutuallyExclusiveCallbackGroup()
        self.cli = self.create_client(ChangeState, '/robot_manager/change_state', callback_group=state_client_group)
        self.state_client = self.create_client(GetState, '/robot_manager/get_state', callback_group=state_client_group)
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = ChangeState.Request()
        action_client_group = MutuallyExclusiveCallbackGroup()
        self.action_client = ActionClient(self, FollowJointTrajectory,
                                          '/joint_trajectory_controller/follow_joint_trajectory',
                                          callback_group=action_client_group)

    def send_configure(self):
        self.req.transition.id = Transition.TRANSITION_CONFIGURE
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.get_logger().info('calling client')
        future = self.cli.call_async(self.req)
        self.get_logger().info("waiting for configure to end")
        self.executor.spin_until_future_complete(future)
        self.get_logger().info("configure complete")
        return future.result()

    def send_activate(self):
        self.req.transition.id = Transition.TRANSITION_ACTIVATE
        future = self.cli.call_async(self.req)
        self.executor.spin_until_future_complete(future)
        return future.result()

    def activate_robot(self):
        self.send_configure()
        return self.send_activate()

    def get_state(self):
        state_future = self.state_client.call_async(GetState.Request())
        self.get_logger().info('waiting for getting state')
        self.executor.spin_until_future_complete(state_future)
        self.get_logger().info('got state')
        return state_future.result()

    def start_robot_movement(self, position, duration):
        goal = FollowJointTrajectory.Goal()
        trajectory = JointTrajectory()
        trajectory.joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
        goal_point = JointTrajectoryPoint()
        goal_point.time_from_start = Duration(sec=duration)
        goal_point.positions = position
        trajectory.points.append(copy.deepcopy(goal_point))
        goal.trajectory = trajectory
        self.action_client.wait_for_server()
        cur_action_submit = self.action_client.send_goal_async(goal)
        self.get_logger().info("waiting for action to be submitted")
        self.executor.spin_until_future_complete(cur_action_submit)
        result_action = cur_action_submit.result().get_result_async()
        result_action.add_done_callback(self.action_done_callback)
        self.get_logger().info("waiting for action to finish")
        self.executor.spin_until_future_complete(result_action)
        self.get_logger().info("action finished")

    def action_done_callback(self, future):
        self.get_logger().info("action finished in callback")
        self.actionFinished.set_result(True)
        future.set_result(True)

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
        self.start_robot_movement([home_pos_1 + 0.4, home_pos_2, home_pos_3, home_pos_4, home_pos_5, home_pos_6], 6)
        self.start_robot_movement([home_pos_1 - 0.4, home_pos_2, home_pos_3, home_pos_4, home_pos_5, home_pos_6], 12)
        self.start_robot_movement(home_pos, 6)
        # joint2 only moving in one direction since we are at the limit
        self.start_robot_movement([home_pos_1, home_pos_2 + 0.4, home_pos_3, home_pos_4, home_pos_5, home_pos_6], 10)
        self.start_robot_movement(home_pos, 10)
        # joint3 only moving in one direction since we are at the limit
        self.start_robot_movement([home_pos_1, home_pos_2, home_pos_3 - 0.4, home_pos_4, home_pos_5, home_pos_6], 10)
        self.start_robot_movement(home_pos, 10)
        # joint4
        self.start_robot_movement([home_pos_1, home_pos_2, home_pos_3, home_pos_4 + 0.4, home_pos_5, home_pos_6], 5)
        self.start_robot_movement([home_pos_1, home_pos_2, home_pos_3, home_pos_4 - 0.4, home_pos_5, home_pos_6], 5)
        self.start_robot_movement(home_pos, 5)
        # joint5
        self.start_robot_movement([home_pos_1, home_pos_2, home_pos_3, home_pos_4, home_pos_5 + 0.4, home_pos_6], 5)
        self.start_robot_movement([home_pos_1, home_pos_2, home_pos_3, home_pos_4, home_pos_5 - 0.4, home_pos_6], 10)
        self.start_robot_movement(home_pos, 5)
        # joint6
        self.start_robot_movement([home_pos_1, home_pos_2, home_pos_3, home_pos_4, home_pos_5, home_pos_6 + 0.4], 5)
        self.start_robot_movement([home_pos_1, home_pos_2, home_pos_3, home_pos_4, home_pos_5, home_pos_6 - 0.4], 10)
        self.start_robot_movement(home_pos, 5)

    def do_robot_movements(self):
        self.activate_robot()
        self.get_logger().info('Finished calling client')
        state_resp = self.get_state()
        if state_resp.current_state.id == State.PRIMARY_STATE_ACTIVE:
            self.get_logger().info('Robot is active')
            self.start_robot_movements()
        else:
            self.get_logger().error("could not activate robot")

    def start_robot_movements_callback(self, request, response):
        self.get_logger().info("Got service call")
        self.do_robot_movements()
        return response


def main(args=None):
    rclpy.init(args=args)
    service_group = rclpy.callback_groups.MutuallyExclusiveCallbackGroup()
    robot_manager_client = RobotManagerClient(service_group)
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(robot_manager_client)
    if robot_manager_client.start_service is False:
        robot_manager_client.do_robot_movements()
        robot_manager_client.destroy_node()
    else:
        executor.spin()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
