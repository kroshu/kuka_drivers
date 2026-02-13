# Copyright 2025 KUKA Hungaria Kft.
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

from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("kuka_external_axis_examples"),
                    "urdf",
                    "kr10_r1100_2_with_kl100_2.urdf.xacro",
                ]
            ),
            " ",
            "mode:=mock",
            " ",
            "use_gpio:=false",
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    # RViz
    rviz_config_file = PathJoinSubstitution(
        [
            FindPackageShare("kuka_external_axis_examples"),
            "config",
            "rviz",
            "view_6_axis_kl_urdf.rviz",
        ]
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_launch",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[robot_description],
    )

    # Publish TF
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    # Joint state publisher
    joint_state_publisher_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        output="log",
    )

    return LaunchDescription([robot_state_publisher, rviz_node, joint_state_publisher_gui])
