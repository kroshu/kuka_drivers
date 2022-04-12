# Copyright 2020 Zoltán Rési
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
from launch.actions.include_launch_description import IncludeLaunchDescription
from launch.launch_description_sources.python_launch_description_source import (
    PythonLaunchDescriptionSource)
import launch_ros.actions


def generate_launch_description():
    kuka_sunrise_dir = get_package_share_directory('kuka_sunrise')

    kuka_sunrise_interface = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([kuka_sunrise_dir, '/launch/kuka_sunrise.launch.py'])
        )

    joint_controller = launch_ros.actions.LifecycleNode(
        package='robot_control', node_executable='joint_controller', output='screen',
        node_name='joint_controller', remappings=[('measured_joint_state', 'lbr_joint_state'),
                                                  ('joint_command', 'lbr_joint_command')]
    )

    keyboard_control = launch_ros.actions.LifecycleNode(
        package='teleop_guided_robot', node_executable='keyboard_control', output='screen',
        node_name='keyboard_control'
    )

    system_manager = launch_ros.actions.LifecycleNode(
        package='teleop_guided_robot', node_executable='system_manager', output='screen',
        node_name='system_manager'
    )
    """
    key_teleop = launch_ros.actions.Node(
        package='key_teleop', node_executable='key_teleop', output='screen',
        node_name='key_teleop')
    """
    return LaunchDescription([
        kuka_sunrise_interface,
        joint_controller,
        keyboard_control,
        system_manager
        ])
