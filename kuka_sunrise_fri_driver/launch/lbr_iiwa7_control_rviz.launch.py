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

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch.substitutions import Command


def load_file(absolute_file_path):
    try:
        with open(absolute_file_path, 'r') as file:
            return file.read()
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def generate_launch_description():
    controller_config = (get_package_share_directory('kuka_sunrise_fri_driver') +
                         "/config/iiwa_ros2_controller_config.yaml")
    forward_controller_config = (get_package_share_directory('kuka_sunrise_fri_driver') +
                                 "/config/forward_controller_config.yaml")
    robot_description_path = (get_package_share_directory('kuka_lbr_iiwa7_support') +
                              "/urdf/lbriiwa7.xacro")
    robot_description = {'robot_description': ParameterValue(
        Command(['xacro ', str(robot_description_path)]), value_type=str
    )}

    rviz_config_file = os.path.join(get_package_share_directory('kuka_lbr_iiwa7_support'),
                                    'launch', 'urdf.rviz')

    return LaunchDescription([
        Node(
            package='kroshu_ros2_core',
            executable='control_node',
            parameters=[robot_description, controller_config]
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster", "-c",
                       "/controller_manager", "--inactive"]
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["forward_command_controller", "-c", "/controller_manager", "-p",
                       forward_controller_config, "--inactive"]
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="log",
            arguments=["-d", rviz_config_file],
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='both',
            parameters=[robot_description]
        ),
    ])
