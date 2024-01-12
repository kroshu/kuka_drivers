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

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
from launch.actions.include_launch_description import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.launch_description_sources.python_launch_description_source import (
    PythonLaunchDescriptionSource,
)
from launch.substitutions import LaunchConfiguration


def launch_setup(context, *args, **kwargs):
    robot_model = LaunchConfiguration("robot_model")
    ns = LaunchConfiguration("namespace")
    x = LaunchConfiguration("x")
    y = LaunchConfiguration("y")
    z = LaunchConfiguration("z")
    roll = LaunchConfiguration("roll")
    pitch = LaunchConfiguration("pitch")
    yaw = LaunchConfiguration("yaw")

    if ns.perform(context) == "":
        tf_prefix = ""
    else:
        tf_prefix = ns.perform(context) + "_"

    moveit_config = (
        MoveItConfigsBuilder("kuka_lbr_iisy")
        .robot_description(
            file_path=get_package_share_directory("kuka_lbr_iisy_support")
            + f"/urdf/{robot_model.perform(context)}.urdf.xacro",
            mappings={
                "x": x.perform(context),
                "y": y.perform(context),
                "z": z.perform(context),
                "roll": roll.perform(context),
                "pitch": pitch.perform(context),
                "yaw": yaw.perform(context),
                "prefix": tf_prefix,
            },
        )
        .robot_description_semantic(
            get_package_share_directory("kuka_lbr_iisy_moveit_config")
            + f"/urdf/{robot_model.perform(context)}.srdf"
        )
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_scene_monitor(
            publish_robot_description=True, publish_robot_description_semantic=True
        )
        .joint_limits(
            file_path=get_package_share_directory("kuka_lbr_iisy_support")
            + f"/config/{robot_model.perform(context)}_joint_limits.yaml"
        )
        .to_moveit_configs()
    )

    rviz_config_file = (
        get_package_share_directory("kuka_resources") + "/config/view_6_axis_planning_scene.rviz"
    )

    startup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [get_package_share_directory("kuka_iiqka_eac_driver"), "/launch/startup.launch.py"]
        ),
    )

    move_group_server = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict(), {"publish_planning_scene_hz": 30.0}],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file, "--ros-args", "--log-level", "error"],
    )

    to_start = [startup_launch, move_group_server, rviz]

    return to_start


def generate_launch_description():
    launch_arguments = []
    launch_arguments.append(DeclareLaunchArgument("robot_model", default_value="lbr_iisy3_r760"))
    launch_arguments.append(DeclareLaunchArgument("namespace", default_value=""))
    launch_arguments.append(DeclareLaunchArgument("x", default_value="0"))
    launch_arguments.append(DeclareLaunchArgument("y", default_value="0"))
    launch_arguments.append(DeclareLaunchArgument("z", default_value="0"))
    launch_arguments.append(DeclareLaunchArgument("roll", default_value="0"))
    launch_arguments.append(DeclareLaunchArgument("pitch", default_value="0"))
    launch_arguments.append(DeclareLaunchArgument("yaw", default_value="0"))
    return LaunchDescription(launch_arguments + [OpaqueFunction(function=launch_setup)])
