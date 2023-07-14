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
from launch.actions.include_launch_description import IncludeLaunchDescription
from launch.launch_description_sources.python_launch_description_source import PythonLaunchDescriptionSource  # noqa: E501


def generate_launch_description():
    rviz_config_file = os.path.join(
        get_package_share_directory('kuka_agilus_support'), 'config', 'agilus_urdf.rviz')

    startup_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        [get_package_share_directory('kuka_rsi_hw_interface'), '/launch/startup.launch.py']))

    return LaunchDescription([
        startup_launch,
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="log",
            arguments=["-d", rviz_config_file,
                       "--ros-args", "--log-level", "error"],
        )
    ])
