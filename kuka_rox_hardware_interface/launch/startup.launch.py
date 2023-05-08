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
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("kuka_lbr_iisy_support"),
                 "urdf", "lbr_iisy3_r760.urdf.xacro"]
            ),
            " ",
        ]
    )

    # Get URDF via xacro
    robot_description = {'robot_description': robot_description_content}

    controller_config = (get_package_share_directory('kuka_rox_hw_interface') +
                         "/config/ros2_controller_config.yaml")

    joint_traj_controller_config = (get_package_share_directory('kuka_rox_hw_interface') +
                                    "/config/joint_trajectory_controller_config.yaml")
    effort_controller_config = (get_package_share_directory('kuka_rox_hw_interface') +
                                "/config/effort_controller_config.yaml")

    joint_imp_controller_config = (get_package_share_directory('kuka_rox_hw_interface') +
                                   "/config/joint_impedance_controller_config.yaml")

    eci_config = (get_package_share_directory('kuka_rox_hw_interface') +
                  "/config/eci_config.yaml")

    controller_manager_node = '/controller_manager'

    return LaunchDescription([
        Node(
            package='kuka_rox_hw_interface',
            executable='rox_control_node',
            parameters=[robot_description, controller_config]
        ),
        LifecycleNode(
            name=['robot_manager'],
            namespace='',
            package="kuka_rox_hw_interface",
            executable="robot_manager_node",
            parameters=[eci_config]
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='both',
            parameters=[robot_description]
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_trajectory_controller", "-c", controller_manager_node, "-p",
                       joint_traj_controller_config, "--inactive"]
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_impedance_controller", "-c", controller_manager_node, "-p",
                       joint_imp_controller_config, "--inactive"],
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster", "-c",
                       controller_manager_node, "--inactive"],
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["effort_controller", "-c", controller_manager_node, "-p",
                       effort_controller_config, "--inactive"]
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["control_mode_handler", "-c",
                       controller_manager_node, "--inactive"]
        )
    ])
