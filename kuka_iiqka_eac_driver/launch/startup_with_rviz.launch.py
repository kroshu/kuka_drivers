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
from launch_ros.actions import Node
from launch.actions.include_launch_description import IncludeLaunchDescription
from launch.conditions import LaunchConfigurationEquals
from launch.launch_description_sources.python_launch_description_source import (
    PythonLaunchDescriptionSource,
)
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    rviz_config = LaunchConfiguration("rviz_config")
    robot_model_launch_arg = DeclareLaunchArgument(
        "robot_model",
        default_value="KUKA_MR"
    )
    
    if LaunchConfigurationEquals('robot_model', 'KUKA_MR'):
        rviz_view="view_KMR_urdf"
    else:
        rviz_view="view_6_axis_urdf"
    rviz_config_launch_arg = DeclareLaunchArgument(
        "rviz_config",
        default_value=os.path.join(
            get_package_share_directory("kuka_resources"), "config", rviz_view + ".rviz"
        ),
    )

    startup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [get_package_share_directory("kuka_iiqka_eac_driver"), "/launch/startup.launch.py"]
        )
    )

    return LaunchDescription(
        [
            robot_model_launch_arg,
            rviz_config_launch_arg,
            startup_launch,
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="log",
                arguments=["-d", rviz_config, "--ros-args", "--log-level", "error"],
            ),
        ]
    )
