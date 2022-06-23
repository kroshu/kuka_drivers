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

    robot_forward_controller = os.path.join(
        get_package_share_directory('kuka_kr6_support'),
        'config',
        'kr6r700sixx_ros2_controller_config.yaml'
        )

    rviz_config_file = os.path.join(
        get_package_share_directory('kuka_kr6_support'),
        'rviz',
        'rviz.rviz')

    return LaunchDescription([
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[robot_description, robot_forward_controller],
            # output={
            # 'stdout': 'screen',
            # 'stderr': 'screen',
            # },
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='both',
            parameters=[robot_description]
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["forward_command_controller_position", "--controller-manager", "/controller_manager"],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="log",
            arguments=["-d", rviz_config_file],
        )

    ])
