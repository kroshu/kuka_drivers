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
from launch.launch_description_sources.python_launch_description_source import PythonLaunchDescriptionSource  # noqa: E501
from launch.substitutions import LaunchConfiguration


def launch_setup(context, *args, **kwargs):
    robot_model = LaunchConfiguration('robot_model')
    moveit_config = (
        MoveItConfigsBuilder("kuka_lbr_iisy")
        .robot_description(file_path=get_package_share_directory('kuka_lbr_iisy_support')
                           + "/urdf/{}.urdf.xacro".format(robot_model.perform(context)))
        .robot_description_semantic(get_package_share_directory('kuka_lbr_iisy_moveit_config')  # noqa: E501
                                    + "/urdf/{}.srdf".format(robot_model.perform(context)))
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_scene_monitor(
            publish_robot_description=True, publish_robot_description_semantic=True
        )
        .joint_limits(file_path=get_package_share_directory('kuka_lbr_iisy_support')
                      + "/config/{}_joint_limits.yaml".format(robot_model.perform(context)))
        .to_moveit_configs()
    )

    rviz_config_file = get_package_share_directory(
        'kuka_lbr_iisy_moveit_config') + "/config/urdf_planning_scene.rviz"

    startup_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        [get_package_share_directory('kuka_rox_hw_interface'), '/launch/startup.launch.py']),
        launch_arguments={'robot_model': "{}".format(robot_model.perform(context))}.items())

    move_group_server = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file, "--ros-args", "--log-level", "error"],
    )

    to_start = [
        startup_launch,
        move_group_server,
        rviz
    ]

    return to_start


def generate_launch_description():
    launch_arguments = []
    launch_arguments.append(DeclareLaunchArgument(
        'robot_model',
        default_value='lbr_iisy3_r760'
    ))
    return LaunchDescription(launch_arguments + [OpaqueFunction(function=launch_setup)])
