# Copyright 2022 Aron Svastits
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
    robot_model = LaunchConfiguration("robot_model")
    controller_ip = LaunchConfiguration("controller_ip")
    client_ip = LaunchConfiguration("client_ip")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    ns = LaunchConfiguration("namespace")
    x = LaunchConfiguration("x")
    y = LaunchConfiguration("y")
    z = LaunchConfiguration("z")
    roll = LaunchConfiguration("roll")
    pitch = LaunchConfiguration("pitch")
    yaw = LaunchConfiguration("yaw")
    roundtrip_time = LaunchConfiguration("roundtrip_time")
    qos_config = LaunchConfiguration("qos_config")
    controller_config = LaunchConfiguration("controller_config")
    tc_config = LaunchConfiguration("tc_config")
    cpb_config= LaunchConfiguration("cpb_config")
    if ns.perform(context) == "":
        tf_prefix = ""
    else:
        tf_prefix = ns.perform(context) + "_"
    
    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("kuka_kmr_iisy_support"),
                    "urdf",
                    robot_model.perform(context) + ".urdf.xacro",
                ]
            ),
            " ",
            "use_fake_hardware:=",
            use_fake_hardware,
            " ",
            "controller_ip:=",
            controller_ip,
            " ",
            "client_ip:=",
            client_ip,
            " ",
            "prefix:=",
            tf_prefix,
            " ",
            "x:=",
            x,
            " ",
            "y:=",
            y,
            " ",
            "z:=",
            z,
            " ",
            "roll:=",
            roll,
            " ",
            "pitch:=",
            pitch,
            " ",
            "yaw:=",
            yaw,
            " ",
            "roundtrip_time:=",
            roundtrip_time,
            " ",
            "qos_config_file:=",
            qos_config,
        ],
        on_stderr="capture",
    )

    # Get URDF via xacro
    robot_description = {"robot_description": robot_description_content}

    # The driver config contains only parameters that can be changed after startup
    driver_config = (
        get_package_share_directory("kuka_kmr_iisy_eac_driver") + "/config/driver_config.yaml"
    )

    controller_manager_node = ns.perform(context) + "/controller_manager"

    control_node = Node(
        namespace=ns,
        package="kuka_drivers_core",
        executable="control_node",
        parameters=[
            robot_description,
            controller_config,
            tc_config,
            cpb_config,
            {
                "hardware_components_initial_state": {
                    "unconfigured": [tf_prefix + robot_model.perform(context)]
                },
            },
        ],
        #arguments=['--ros-args', '--log-level', 'debug'],
        #prefix=['gdbserver localhost:3000'],
        #output='screen',
    )
    robot_manager_node = LifecycleNode(
        name=["robot_manager"],
        namespace=ns,
        package="kuka_kmr_iisy_eac_driver",
        executable="robot_manager_node",
        parameters=[
            driver_config,
            {
                "robot_model": robot_model,
                "controller_ip": controller_ip,
            },
        ],
    )
    robot_state_publisher = Node(
        namespace=ns,
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    # Spawn controllers
    def controller_spawner(controller_names, activate=False):
        arg_list = [
            controller_names,
            "-c",
            controller_manager_node,
            "-n",
            ns,
        ]
        if not activate:
            arg_list.append("--inactive")
        return Node(package="controller_manager", executable="spawner", arguments=arg_list)

    controller_names = [
        "joint_state_broadcaster",
        "control_mode_handler",
        "event_broadcaster",
        "twist_controller",
        "cartesian_pose_broadcaster"
    ]

    controller_spawners = [controller_spawner(name) for name in controller_names]

    nodes_to_start = [
        control_node,
        robot_manager_node,
        robot_state_publisher,
    ] + controller_spawners

    return nodes_to_start


def generate_launch_description():
    launch_arguments = []
    launch_arguments.append(DeclareLaunchArgument("robot_model", default_value="kmr_iisy"))
    launch_arguments.append(DeclareLaunchArgument("controller_ip", default_value="0.0.0.0"))
    launch_arguments.append(DeclareLaunchArgument("client_ip", default_value="0.0.0.0"))
    launch_arguments.append(DeclareLaunchArgument("use_fake_hardware", default_value="false"))
    launch_arguments.append(DeclareLaunchArgument("namespace", default_value=""))
    launch_arguments.append(DeclareLaunchArgument("x", default_value="0"))
    launch_arguments.append(DeclareLaunchArgument("y", default_value="0"))
    launch_arguments.append(DeclareLaunchArgument("z", default_value="0"))
    launch_arguments.append(DeclareLaunchArgument("roll", default_value="0"))
    launch_arguments.append(DeclareLaunchArgument("pitch", default_value="0"))
    launch_arguments.append(DeclareLaunchArgument("yaw", default_value="0"))
    launch_arguments.append(DeclareLaunchArgument("roundtrip_time", default_value="2500"))
    launch_arguments.append(
        DeclareLaunchArgument(
            "qos_config",
            default_value=get_package_share_directory("kuka_kmr_iisy_eac_driver")
            + "/config/qos_config.yaml",
        )
    )
    launch_arguments.append(
        DeclareLaunchArgument(
            "tc_config",
            default_value=get_package_share_directory("kuka_kmr_iisy_eac_driver")
            + "/config/twist_controller_config.yaml",
        )
    )
    launch_arguments.append(
        DeclareLaunchArgument(
            "cpb_config",
            default_value=get_package_share_directory("kuka_kmr_iisy_eac_driver")
            + "/config/cartesian_pose_broadcaster_config.yaml",
        )
    )
    launch_arguments.append(
        DeclareLaunchArgument(
            "controller_config",
            default_value=get_package_share_directory("kuka_kmr_iisy_eac_driver")
            + "/config/ros2_controller_config.yaml",
        )
    )
    return LaunchDescription(launch_arguments + [OpaqueFunction(function=launch_setup)])
