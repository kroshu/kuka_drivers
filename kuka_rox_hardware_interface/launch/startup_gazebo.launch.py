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

from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare

from launch.actions import AppendEnvironmentVariable
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit


def generate_launch_description():
    # robot_model = LaunchConfiguration('robot_model')
    # Get URDF via xacro
    robot_support_package = "kuka_mobile_robot_support"

    #robot_support_package = "kuka_lbr_iisy_support"
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(robot_support_package),
                 "urdf", "KUKA_MR" + ".urdf.xacro"]
            ),
            " ",
            "gazebo_sim:=",
            "true",
        ]
    )
    conntroller_manager_node = '/controller_manager'
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
    
    
    # diffdrive controller
    controller_manager_node = '/controller_manager'

    diff_drive_controller_config = (get_package_share_directory('kuka_rox_hw_interface') +
                                   "/config/diff_drive_controller_config.yaml")
    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diffbot_controller", "-c", controller_manager_node, "-p", 
                   diff_drive_controller_config],
    )


    return LaunchDescription([
        AppendEnvironmentVariable(name='GAZEBO_MODEL_PATH', value=get_package_share_directory(robot_support_package)),
        
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[load_joint_state_controller],
           )
        ),
        spawn_entity,
        gazebo_launch,
        robot_state_publisher,
        diff_drive_spawner
        
    ])

