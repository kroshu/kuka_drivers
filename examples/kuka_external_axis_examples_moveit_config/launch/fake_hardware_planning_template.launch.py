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

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import (Command, FindExecutable, LaunchConfiguration,
                                  PathJoinSubstitution)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context):
    robot_model = LaunchConfiguration("robot_model")
    ext_axes_family = LaunchConfiguration("ext_axes_family")
    robot_dof = LaunchConfiguration("robot_dof")

    rviz_config_file = (
        get_package_share_directory("kuka_external_axis_examples")
        + "/config/rviz/planning_"
        + f"{robot_dof.perform(context)}_axis_{ext_axes_family.perform(context)}.rviz"
    )

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("kuka_external_axis_examples"),
                    "urdf",
                    robot_model.perform(context) + ".urdf.xacro",
                ]
            ),
            " ",
            "mode:=mock"
        ]
    )

    robot_description = {"robot_description": robot_description_content}

    controller_config = (
        get_package_share_directory("kuka_external_axis_examples")
        + "/config/fake_hardware_config_"
        + f"{robot_dof.perform(context)}_axis_{ext_axes_family.perform(context)}.yaml"
    )

    controller_manager_node = "/controller_manager"

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, controller_config],
    )

    robot_description_kinematics = {
        "robot_description_kinematics": {
            "manipulator": {"kinematics_solver": "kdl_kinematics_plugin/KDLKinematicsPlugin"}
        }
    }

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file, "--ros-args", "--log-level", "error"],
        parameters=[
            robot_description_kinematics,
        ],
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    # Spawn controllers
    def controller_spawner(controller_with_config):
        arg_list = [controller_with_config[0], "-c", controller_manager_node]
        if controller_with_config[1] is not None:
            arg_list.append("-p")
            arg_list.append(controller_with_config[1])
        return Node(package="controller_manager", executable="spawner", arguments=arg_list)

    controller_names_and_config = [
        ("joint_state_broadcaster", None),
        ("joint_trajectory_controller", controller_config),
    ]

    controller_spawners = [
        controller_spawner(controllers) for controllers in controller_names_and_config
    ]

    to_start = [control_node, robot_state_publisher, rviz] + controller_spawners

    return to_start


def generate_launch_description():
    launch_arguments = []
    launch_arguments.append(DeclareLaunchArgument("robot_model", default_value="kr10_r1100_2_with_kl100_2"))
    launch_arguments.append(DeclareLaunchArgument("ext_axes_family", default_value="kl"))
    launch_arguments.append(DeclareLaunchArgument("robot_dof", default_value="6"))
    return LaunchDescription(launch_arguments + [OpaqueFunction(function=launch_setup)])
