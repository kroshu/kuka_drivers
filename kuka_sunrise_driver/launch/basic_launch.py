import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

def load_file(absolute_file_path):
    try:
        with open(absolute_file_path, 'r') as file:
            return file.read()
    except EnvironmentError: # parent of IOError, OSError *and* WindowsError where available
        return None


def generate_launch_description():
    controller_config = get_package_share_directory('urdflbriiwa7') + "/config/iiwa_ros2_controller_config.yaml"
    forward_controller_config = get_package_share_directory('kuka_sunrise') + "/config/forward_controller.yaml"
    robot_description_config = load_file(get_package_share_directory('urdflbriiwa7') + "/urdf/urdflbriiwa7.urdf")
    robot_description = {'robot_description' : robot_description_config}

    rviz_config_file = os.path.join(
    get_package_share_directory('urdflbriiwa7'), 'launch', 'urdf.rviz')

    return LaunchDescription([
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[robot_description, controller_config]
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster", "-c", "/controller_manager", "--stopped"]
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["forward_command_controller_position", "-c", "/controller_manager", "-p",
                       get_package_share_directory('kuka_sunrise') + "/config/forward_controller.yaml", "--stopped"]
        ),
        # Node(
        #     package="rviz2",
        #     executable="rviz2",
        #     name="rviz2",
        #     output="log",
        #     arguments=["-d", rviz_config_file],
        # ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='both',
            parameters=[robot_description]
        ),
    ])