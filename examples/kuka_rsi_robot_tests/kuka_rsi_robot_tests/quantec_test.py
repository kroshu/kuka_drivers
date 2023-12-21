import copy

import rclpy
from builtin_interfaces.msg import Duration
from rclpy.node import Node
from lifecycle_msgs.srv import ChangeState, GetState
from lifecycle_msgs.msg import Transition, State
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from rclpy.action import ActionClient


class RobotManagerClient(Node):
    def __init__(self):
        super().__init__('robot_manager_client')
        self.action_client = None
        self.cli = self.create_client(ChangeState, '/robot_manager/change_state')
        self.state_client = self.create_client(GetState, '/robot_manager/get_state')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = ChangeState.Request()

    def send_configure(self):
        self.req.transition.id = Transition.TRANSITION_CONFIGURE
        future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def send_activate(self):
        self.req.transition.id = Transition.TRANSITION_ACTIVATE
        future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def activate_robot(self):
        self.send_configure()
        return self.send_activate()

    def get_state(self):
        state_future = self.state_client.call_async(GetState.Request())
        rclpy.spin_until_future_complete(self, state_future)
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
        rclpy.spin_until_future_complete(self, cur_action_submit)
        result_action = cur_action_submit.result().get_result_async()
        self.get_logger().info("waiting for action to finish")
        rclpy.spin_until_future_complete(self, result_action)

    def start_robot_movements(self):
        self.get_logger().info("Robot movement starting")
        self.action_client = ActionClient(self, FollowJointTrajectory,
                                          '/joint_trajectory_controller/follow_joint_trajectory')
        self.start_robot_movement([0.0, -2.1995, 2.4529, 0.0, 1.308085, 0.0], 10)
        self.start_robot_movement([0.0, -2.1995, 2.6529, 0.0, 1.308085, 0.0], 10)


def main(args=None):
    rclpy.init(args=args)

    robot_manager_client = RobotManagerClient()
    robot_manager_client.activate_robot()
    robot_manager_client.get_logger().info(
        'Finished calling client')
    state_resp = robot_manager_client.get_state()

    if state_resp.current_state.id == State.PRIMARY_STATE_ACTIVE:
        robot_manager_client.get_logger().info('Robot is active')
        robot_manager_client.start_robot_movements()
    else:
        robot_manager_client.get_logger().error("could not activate robot")

    robot_manager_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
