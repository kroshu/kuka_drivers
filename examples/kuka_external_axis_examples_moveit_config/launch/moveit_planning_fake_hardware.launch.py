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
from launch.actions.include_launch_description import IncludeLaunchDescription
from launch.launch_description_sources.python_launch_description_source import \
    PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def launch_setup(context, *args, **kwargs):
    robot_model = LaunchConfiguration("robot_model")
    ext_axes_family = LaunchConfiguration("ext_axes_family")
    robot_dof = LaunchConfiguration("robot_dof")

    moveit_config = (
        MoveItConfigsBuilder("kuka_external_axis_examples")
        .robot_description(
            file_path=get_package_share_directory("kuka_external_axis_examples")
            + f"/urdf/{robot_model.perform(context)}.urdf.xacro"
        )
        .robot_description_semantic(
            file_path=get_package_share_directory("kuka_external_axis_examples_moveit_config")
            + f"/srdf/{robot_model.perform(context)}.srdf.xacro"
        )
        .robot_description_kinematics(
            file_path=get_package_share_directory("kuka_external_axis_examples_moveit_config")
            + "/config/kinematics.yaml"
        )
        .trajectory_execution(
            file_path=get_package_share_directory("kuka_external_axis_examples_moveit_config")
            + "/config/moveit_controllers_"
            + f"{robot_dof.perform(context)}_axis_{ext_axes_family.perform(context)}.yaml"
        )
        .planning_scene_monitor(
            publish_robot_description=True, publish_robot_description_semantic=True
        )
        .joint_limits(
            file_path=get_package_share_directory("kuka_external_axis_examples")
            + f"/config/{robot_model.perform(context)}_joint_limits.yaml"
        )
        # No explicit planning pipeline setup needed: MoveIt loads any config/*_planning.yaml files automatically
        .to_moveit_configs()
    )

    move_group_server = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()],
    )

    fake_hardware_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                get_package_share_directory("kuka_external_axis_examples_moveit_config"),
                "/launch/fake_hardware_planning_template.launch.py",
            ]
        ),
        launch_arguments={
            "robot_model": robot_model,
            "ext_axes_family": ext_axes_family,
            "robot_dof": robot_dof,
        }.items(),
    )

    to_start = [fake_hardware_launch, move_group_server]

    return to_start


def generate_launch_description():
    launch_arguments = []
    launch_arguments.append(
        DeclareLaunchArgument("robot_model", default_value="kr10_r1100_2_with_kl100_2")
    )
    launch_arguments.append(DeclareLaunchArgument("ext_axes_family", default_value="kl"))
    launch_arguments.append(DeclareLaunchArgument("robot_dof", default_value="6"))
    return LaunchDescription(launch_arguments + [OpaqueFunction(function=launch_setup)])
