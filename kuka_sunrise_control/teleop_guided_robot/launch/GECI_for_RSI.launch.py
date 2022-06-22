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

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription

import launch_ros.actions
import yaml


def load_file(package_name, file_path):

    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, 'r') as file:
            return file.read()
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        print('Couldn\'t load file ' + absolute_file_path)
        return None


def load_yaml(package_name, file_path):

    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        print('Couldn\'t load yaml ' + absolute_file_path)
        return None


def generate_launch_description():

    robot_config_file = get_package_share_directory('robot_control') + "/config/kr6.yaml"

    joint_controller = launch_ros.actions.LifecycleNode(
        package='robot_control', executable='interpolating_controller', output='both',
        arguments=['--ros-args', '--log-level', 'info'], parameters=[robot_config_file],
        name='joint_controller', remappings=[('measured_joint_state', 'rsi_joint_state'),
                                             ('joint_command', 'rsi_joint_command')]
        )

    return LaunchDescription([
        joint_controller
        ])
