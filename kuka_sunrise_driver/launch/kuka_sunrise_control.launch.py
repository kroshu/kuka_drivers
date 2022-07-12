import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.actions import LifecycleNode

def load_file(absolute_file_path):
    try:
        with open(absolute_file_path, 'r') as file:
            return file.read()
    except EnvironmentError: # parent of IOError, OSError *and* WindowsError where available
        return None


def generate_launch_description():
    controller_config = get_package_share_directory('kuka_sunrise') + "/config/iiwa_ros2_controller_config.yaml"
    forward_controller_config = get_package_share_directory('kuka_sunrise') + "/config/forward_controller.yaml"
    robot_description_config = load_file(get_package_share_directory('kuka_lbr_iiwa7_support') + "/urdf/urdflbriiwa7.urdf")
    robot_description = {'robot_description' : robot_description_config}

    return LaunchDescription([
        Node(
            package='kuka_sunrise',
            executable='sunrise_control_node',
            parameters=[robot_description, controller_config]
        ),
        LifecycleNode(
            namespace = '', package='kuka_sunrise', executable='robot_manager_node', output='screen',
            name=['robot_manager'], parameters=[{'controller_ip': '<insert ip here>'}]
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster", "-c", "/controller_manager", "--load-only"]
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["forward_command_controller_position", "-c", "/controller_manager", "-p",
                       forward_controller_config, "--load-only"]
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["timing_controller", "-c", "/controller_manager"]
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["robot_state_broadcaster", "-c", "/controller_manager"]
        )
    ])