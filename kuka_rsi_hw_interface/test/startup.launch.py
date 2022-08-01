import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

import xacro


def generate_launch_description():

    # Get URDF via xacro
    robot_description_path = os.path.join(
        get_package_share_directory('kuka_kr6_support'),
        'urdf',
        'kr6r700sixx_ros2_control.xacro')
    robot_description_config = xacro.process_file(robot_description_path)
    robot_description = {'robot_description': robot_description_config.toxml()}

    controller_config = (get_package_share_directory('kuka_rsi_hw_interface') +
                         "/config/ros2_controller_config.yaml")

    joint_traj_controller_config = (get_package_share_directory('kuka_rsi_hw_interface') +
                                    "/config/joint_trajectory_controller_config.yaml")

    controller_manager_node = '/controller_manager'

    rviz_config_file = os.path.join(
        get_package_share_directory('kuka_kr6_support'),
        'rviz',
        'rviz.rviz')

    return LaunchDescription([
        Node(
            package='kuka_rsi_hw_interface',
            executable='rsi_control_node',
            parameters=[robot_description, controller_config]
        ),
        # Node(
        #     package='robot_state_publisher',
        #     executable='robot_state_publisher',
        #     output='both',
        #     parameters=[robot_description]
        # ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster", "-c", controller_manager_node],
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_trajectory_controller", "-c", controller_manager_node, "-p",
                       joint_traj_controller_config]
        ),
        # Node(
        #     package="rviz2",
        #     executable="rviz2",
        #     name="rviz2",
        #     output="log",
        #     arguments=["-d", rviz_config_file],
        # )

    ])
