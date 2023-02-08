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
from launch_ros.actions import LifecycleNode


def generate_launch_description():

    # Get URDF via xacro
    robot_description_path = os.path.join(
        get_package_share_directory('kuka_iisy_support'), 'urdf', 'iisy.urdf')

    with open(robot_description_path, 'r') as desc:
        robot_description = {'robot_description': desc.read()}

    controller_config = (get_package_share_directory('kuka_rox_hw_interface') +
                         "/config/ros2_controller_config.yaml")

    joint_traj_controller_config = (get_package_share_directory('kuka_rox_hw_interface') +
                                    "/config/joint_trajectory_controller_config.yaml")
    effort_controller_config = (get_package_share_directory('kuka_rox_hw_interface') +
                                "/config/effort_controller_config.yaml")

    eci_config = (get_package_share_directory('kuka_rox_hw_interface') +
                  "/config/eci_config.yaml")

    controller_manager_node = '/controller_manager'

    rviz_config_file = os.path.join(
       get_package_share_directory('kuka_iisy_support'), 'launch', 'urdf_wo_planning_scene.rviz')

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
            parameters=[eci_config,
                        {'position_controller_name': 'joint_trajectory_controller'},
                        {'impedance_controller_name': 'joint_impedance_controller'},
                        {'torque_controller_name': ''}]
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
           arguments=["joint_impedance_controller", "-c", controller_manager_node, "-t",
                      "kuka_controllers/JointImpedanceController", "--inactive"],
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster", "-c", controller_manager_node, "--inactive"],
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["effort_controller", "-c", controller_manager_node, "-p",
                       effort_controller_config, "--inactive"]
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="log",
            arguments=["-d", rviz_config_file, "--ros-args", "--log-level", "error"],
        )
    ])
