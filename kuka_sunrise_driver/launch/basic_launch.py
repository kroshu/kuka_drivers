import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

def load_file(absolute_file_path):
    # package_path = get_package_share_directory(package_name)
    # absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, 'r') as file:
            return file.read()
    except EnvironmentError: # parent of IOError, OSError *and* WindowsError where available
        return None


def generate_launch_description():
    controller_config = get_package_share_directory('urdflbriiwa7') + "/config/iiwa_ros2_controller_config.yaml"

    robot_description_config = load_file("/home/rosdeveloper/ros2_ws/src/urdflbriiwa7/urdf/urdflbriiwa7.urdf")
    robot_description = {'robot_description' : robot_description_config}

    return LaunchDescription([
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[robot_description, controller_config],
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
    ])