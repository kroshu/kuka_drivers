# Copyright 2023 Áron Svastits
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
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node, LifecycleNode
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    robot_model = LaunchConfiguration('robot_model')

    # TODO(Svastits):better way to handle supported robot models and families
    if robot_model.perform(context) in ["kr6_r700_sixx", "kr6_r900_sixx"]:
        robot_family = "agilus"
    elif robot_model.perform(context) in ["kr16_r2010-2"]:
        robot_family = "cybertech"
    else:
        print("[ERROR] [launch]: robot model not recognized")
        raise Exception

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare('kuka_{}_support'.format(robot_family)),
                 "urdf", robot_model.perform(context) + ".urdf.xacro"]
            ),
            " ",
        ]
    )

    robot_description = {'robot_description': robot_description_content}

    controller_config = (get_package_share_directory('kuka_rsi_hw_interface') +
                         "/config/ros2_controller_config.yaml")

    joint_traj_controller_config = (get_package_share_directory('kuka_rsi_hw_interface') +
                                    "/config/joint_trajectory_controller_config.yaml")

    controller_manager_node = '/controller_manager'

    control_node = Node(
        package='kuka_rsi_hw_interface',
        executable='rsi_control_node',
        parameters=[robot_description, controller_config]
    )
    robot_manager_node = LifecycleNode(
        name=['robot_manager'],
        namespace='',
        package="kuka_rsi_hw_interface",
        executable="robot_manager_node",
        parameters=[{'robot_model': robot_model}]
    )
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[robot_description]
    )

    # Spawn controllers
    def controller_spawner(controller_with_config):
        return Node(
            package="controller_manager",
            executable="spawner",
            arguments=[controller_with_config[0], "-c", controller_manager_node, "-p",
                       controller_with_config[1], "--inactive"]
        )

    controller_names_and_config = [
        ("joint_state_broadcaster", []),
        ("joint_trajectory_controller", joint_traj_controller_config),
    ]

    controller_spawners = [controller_spawner(controllers)
                           for controllers in controller_names_and_config]

    nodes_to_start = [
        control_node,
        robot_manager_node,
        robot_state_publisher
    ] + controller_spawners

    return nodes_to_start


def generate_launch_description():
    launch_arguments = []
    launch_arguments.append(DeclareLaunchArgument(
        'robot_model',
        default_value='kr6_r700_sixx'
    ))
    return LaunchDescription(launch_arguments + [OpaqueFunction(function=launch_setup)])
