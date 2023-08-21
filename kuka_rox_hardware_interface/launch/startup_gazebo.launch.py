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
from launch.actions.include_launch_description import IncludeLaunchDescription
from launch.launch_description_sources.python_launch_description_source import PythonLaunchDescriptionSource  # noqa: E501

from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

from launch.actions import AppendEnvironmentVariable
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit


def generate_launch_description():
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("kuka_lbr_iisy_support"),
                 "urdf", "lbr_iisy3_r760.urdf.xacro"]
            ),
            " ",
            "gazebo_sim:=",
            "true",
        ]
    )
    robot_description = {'robot_description': robot_description_content}
    # Publish TF
    robot_state_publisher = Node(package='robot_state_publisher',
                                 executable='robot_state_publisher',
                                 name='robot_state_publisher',
                                 output='both',
                                 parameters=[robot_description])

    gazebo_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        [get_package_share_directory('gazebo_ros'), '/launch/gazebo.launch.py']))

    spawn_entity = Node(package='gazebo_ros',
                        executable='spawn_entity.py',
                        name='spawn_entity',
                        output='log',
                        arguments=['-topic', '/robot_description', '-entity', 'robot'])
    
    load_joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )

    load_joint_trajectory_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_trajectory_controller'],
        output='screen'
    )

    return LaunchDescription([
        AppendEnvironmentVariable(name='GAZEBO_MODEL_PATH', value=get_package_share_directory('kuka_lbr_iisy_support')),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[load_joint_state_controller],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_controller,
                on_exit=[load_joint_trajectory_controller],
            )
        ),
        spawn_entity,
        robot_state_publisher,
        gazebo_launch,
    ])

