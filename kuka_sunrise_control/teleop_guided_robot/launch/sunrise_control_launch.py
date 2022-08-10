# Copyright 2022 Aron Svastits
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

from launch.actions.include_launch_description import IncludeLaunchDescription
from launch.launch_description_sources.python_launch_description_source import PythonLaunchDescriptionSource  # noqa: E501
import launch_ros.actions
import yaml


def load_file(package_name, file_path):

    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, 'r') as file:
            return file.read()
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        print('Couldnt load file ' + absolute_file_path)
        return None


def load_yaml(package_name, file_path):

    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        print('Couldnt load yaml ' + absolute_file_path)
        return None


def generate_launch_description():

    robot_config_file = get_package_share_directory('robot_control') + "/config/lbr_iiwa.yaml"
    kuka_sunrise_dir = get_package_share_directory('kuka_sunrise')

    kuka_sunrise_interface = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([kuka_sunrise_dir, '/launch/kuka_sunrise.launch.py'])
        )

    joint_controller = launch_ros.actions.LifecycleNode(
        namespace="", package='robot_control', executable='interpolating_controller', output='both',
        arguments=['--ros-args', '--log-level', 'info'], parameters=[robot_config_file],
        name='joint_controller', remappings=[('measured_joint_state', 'lbr_joint_state'),
                                             ('joint_command', 'lbr_joint_command')]
        )

    system_manager = launch_ros.actions.LifecycleNode(
        package='teleop_guided_robot', executable='system_manager', output='screen',
        name='system_manager', namespace=""
        )

    return LaunchDescription([
        kuka_sunrise_interface,
        system_manager,
        joint_controller
        ])
