# Copyright 2022 Áron Svastits
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.actions import LifecycleNode
from launch_ros.descriptions import ParameterValue
from launch.substitutions import Command


def load_file(absolute_file_path):
    try:
        with open(absolute_file_path, 'r') as file:
            return file.read()
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def generate_launch_description():
    controller_config = (get_package_share_directory('kuka_sunrise') +
                         "/config/iiwa_ros2_controller_config.yaml")
    joint_traj_controller_config = (get_package_share_directory('kuka_sunrise') +
                                   "/config/joint_trajectory_controller_config.yaml")
    robot_description_path = (get_package_share_directory('kuka_lbr_iiwa7_support') +
                              "/urdf/lbriiwa7.xacro")
    robot_description = {'robot_description': ParameterValue(
            Command(['xacro ', str(robot_description_path)]), value_type=str
        )}

    conntroller_manager_node = '/controller_manager'

    return LaunchDescription([
        Node(
            package='kuka_sunrise',
            executable='sunrise_control_node',
            parameters=[robot_description, controller_config]
        ),
        LifecycleNode(
            namespace='', package='kuka_sunrise', executable='robot_manager_node', output='screen',
            name=['robot_manager'],
            parameters=[{'controller_ip': '<insert ip here>'},
                        {'position_controller_name': 'joint_trajectory_controller'},
                        {'torque_controller_name': ''}]
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster", "-c", conntroller_manager_node, "--inactive"]
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_trajectory_controller", "-c", conntroller_manager_node, "-p",
                       joint_traj_controller_config, "--inactive"]
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["timing_controller", "-c", conntroller_manager_node, "--inactive"]
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["robot_state_broadcaster", "-c", conntroller_manager_node, "--inactive"]
        )
    ])
