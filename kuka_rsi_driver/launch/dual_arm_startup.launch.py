# Copyright 2026 KUKA Hungaria Kft.
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
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import LifecycleNode, Node
from launch_ros.substitutions import FindPackageShare


DUAL_ARM_TEMPLATE_XACRO = "dual_arm_template.urdf.xacro"


def launch_setup(context, *args, **kwargs):
    mode = LaunchConfiguration("mode")
    driver_version = LaunchConfiguration("driver_version")
    ns = LaunchConfiguration("namespace")
    controller_config_dir = LaunchConfiguration("controller_config_dir")
    event_broadcaster_robot_prefixes = LaunchConfiguration("event_broadcaster_robot_prefixes")
    non_rt_cores = LaunchConfiguration("non_rt_cores")
    rt_core = LaunchConfiguration("rt_core")
    rt_prio = LaunchConfiguration("rt_prio")
    lock_memory = LaunchConfiguration("lock_memory")

    # Robot 1 parameters
    robot1_model = LaunchConfiguration("robot1_model")
    robot1_family = LaunchConfiguration("robot1_family")
    robot1_prefix = LaunchConfiguration("robot1_prefix")
    robot1_client_ip = LaunchConfiguration("robot1_client_ip")
    robot1_client_port = LaunchConfiguration("robot1_client_port")
    robot1_mxa_client_port = LaunchConfiguration("robot1_mxa_client_port")
    robot1_controller_ip = LaunchConfiguration("robot1_controller_ip")
    robot1_x = LaunchConfiguration("robot1_x")
    robot1_y = LaunchConfiguration("robot1_y")
    robot1_z = LaunchConfiguration("robot1_z")
    robot1_roll = LaunchConfiguration("robot1_roll")
    robot1_pitch = LaunchConfiguration("robot1_pitch")
    robot1_yaw = LaunchConfiguration("robot1_yaw")
    robot1_roundtrip_time = LaunchConfiguration("robot1_roundtrip_time")
    robot1_verify_robot_model = LaunchConfiguration("robot1_verify_robot_model")
    robot1_rsi_xml_config_file = LaunchConfiguration("robot1_rsi_xml_config_file")
    robot1_use_gpio = LaunchConfiguration("robot1_use_gpio")
    robot1_async_thread_priority = LaunchConfiguration("robot1_async_thread_priority")
    robot1_async_affinity = LaunchConfiguration("robot1_async_affinity")

    # Robot 2 parameters
    robot2_model = LaunchConfiguration("robot2_model")
    robot2_family = LaunchConfiguration("robot2_family")
    robot2_prefix = LaunchConfiguration("robot2_prefix")
    robot2_client_ip = LaunchConfiguration("robot2_client_ip")
    robot2_client_port = LaunchConfiguration("robot2_client_port")
    robot2_mxa_client_port = LaunchConfiguration("robot2_mxa_client_port")
    robot2_controller_ip = LaunchConfiguration("robot2_controller_ip")
    robot2_x = LaunchConfiguration("robot2_x")
    robot2_y = LaunchConfiguration("robot2_y")
    robot2_z = LaunchConfiguration("robot2_z")
    robot2_roll = LaunchConfiguration("robot2_roll")
    robot2_pitch = LaunchConfiguration("robot2_pitch")
    robot2_yaw = LaunchConfiguration("robot2_yaw")
    robot2_roundtrip_time = LaunchConfiguration("robot2_roundtrip_time")
    robot2_verify_robot_model = LaunchConfiguration("robot2_verify_robot_model")
    robot2_rsi_xml_config_file = LaunchConfiguration("robot2_rsi_xml_config_file")
    robot2_use_gpio = LaunchConfiguration("robot2_use_gpio")
    robot2_async_thread_priority = LaunchConfiguration("robot2_async_thread_priority")
    robot2_async_affinity = LaunchConfiguration("robot2_async_affinity")

    # Parse allowed cores for taskset
    cores = []
    for part in non_rt_cores.perform(context).split(","):
        part = part.strip()
        if part == "":
            continue
        try:
            cores.append(int(part))
        except ValueError:
            raise RuntimeError(
                f"Invalid non_rt_cores entry: '{part}'. "
                "Provide a comma-separated list of integers, e.g. '2,3,4'."
            )

    prefix_cmd = None
    if cores:
        core_list_str = ",".join(str(c) for c in cores)
        prefix_cmd = f"taskset -c {core_list_str}"

    robot1_prefix_value = robot1_prefix.perform(context)
    robot2_prefix_value = robot2_prefix.perform(context)
    robot1_model_value = robot1_model.perform(context)
    robot2_model_value = robot2_model.perform(context)

    # Generate URDF via xacro using the dual-arm template
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("kuka_resources"), "urdf", DUAL_ARM_TEMPLATE_XACRO]
            ),
            " ",
            "mode:=", mode,
            " ",
            "driver_version:=", driver_version,
            " ",
            "robot1_model:=", robot1_model,
            " ",
            "robot1_family:=", robot1_family,
            " ",
            "robot1_prefix:=", robot1_prefix,
            " ",
            "robot1_x:=", robot1_x,
            " ",
            "robot1_y:=", robot1_y,
            " ",
            "robot1_z:=", robot1_z,
            " ",
            "robot1_roll:=", robot1_roll,
            " ",
            "robot1_pitch:=", robot1_pitch,
            " ",
            "robot1_yaw:=", robot1_yaw,
            " ",
            "robot1_client_ip:=", robot1_client_ip,
            " ",
            "robot1_client_port:=", robot1_client_port,
            " ",
            "robot1_mxa_client_port:=", robot1_mxa_client_port,
            " ",
            "robot1_controller_ip:=", robot1_controller_ip,
            " ",
            "robot1_roundtrip_time:=", robot1_roundtrip_time,
            " ",
            "robot1_use_gpio:=", robot1_use_gpio,
            " ",
            "robot1_verify_robot_model:=", robot1_verify_robot_model,
            " ",
            "robot1_rsi_xml_config_file:=", robot1_rsi_xml_config_file,
            " ",
            "robot1_async_thread_priority:=", robot1_async_thread_priority,
            " ",
            "robot1_async_affinity:=", robot1_async_affinity,
            " ",
            "robot2_model:=", robot2_model,
            " ",
            "robot2_family:=", robot2_family,
            " ",
            "robot2_prefix:=", robot2_prefix,
            " ",
            "robot2_x:=", robot2_x,
            " ",
            "robot2_y:=", robot2_y,
            " ",
            "robot2_z:=", robot2_z,
            " ",
            "robot2_roll:=", robot2_roll,
            " ",
            "robot2_pitch:=", robot2_pitch,
            " ",
            "robot2_yaw:=", robot2_yaw,
            " ",
            "robot2_client_ip:=", robot2_client_ip,
            " ",
            "robot2_client_port:=", robot2_client_port,
            " ",
            "robot2_mxa_client_port:=", robot2_mxa_client_port,
            " ",
            "robot2_controller_ip:=", robot2_controller_ip,
            " ",
            "robot2_roundtrip_time:=", robot2_roundtrip_time,
            " ",
            "robot2_use_gpio:=", robot2_use_gpio,
            " ",
            "robot2_verify_robot_model:=", robot2_verify_robot_model,
            " ",
            "robot2_rsi_xml_config_file:=", robot2_rsi_xml_config_file,
            " ",
            "robot2_async_thread_priority:=", robot2_async_thread_priority,
            " ",
            "robot2_async_affinity:=", robot2_async_affinity,
        ],
        on_stderr="capture",
    )

    robot_description = {"robot_description": robot_description_content}

    driver_config = (
        get_package_share_directory("kuka_rsi_driver") + "/config/driver_config.yaml"
    )
    config_dir_path = controller_config_dir.perform(context)

    def config_file(filename):
        return os.path.join(config_dir_path, filename)

    event_broadcaster_prefix_values = [
        prefix.strip()
        for prefix in event_broadcaster_robot_prefixes.perform(context).split(",")
        if prefix.strip()
    ]
    event_broadcaster_prefix_yaml = ", ".join(event_broadcaster_prefix_values)
    with tempfile.NamedTemporaryFile(mode="w", suffix=".yaml", delete=False) as temp_file:
        temp_file.write(
            "/**/event_broadcaster:\n"
            "  ros__parameters:\n"
            f"    robot_prefixes: [{event_broadcaster_prefix_yaml}]\n"
        )
        event_broadcaster_config_file = temp_file.name

    controller_config_file = config_file("ros2_controller_config_dual_arm.yaml")

    robot1_hw_name = robot1_prefix_value + robot1_model_value
    robot2_hw_name = robot2_prefix_value + robot2_model_value

    control_node = Node(
        namespace=ns,
        package="kuka_drivers_core",
        executable="control_node",
        parameters=[
            robot_description,
            controller_config_file,
            {
                "cpu_affinity": int(rt_core.perform(context)),
                "thread_priority": int(rt_prio.perform(context)),
                "lock_memory": lock_memory.perform(context) == "true",
                "hardware_components_initial_state": {
                    "unconfigured": [robot1_hw_name, robot2_hw_name]
                },
            },
        ],
        prefix=prefix_cmd,
    )

    robot_manager_node = LifecycleNode(
        name=["robot_manager"],
        namespace=ns,
        package="kuka_rsi_driver",
        executable=(
            "robot_manager_node_rsi_only"
            if driver_version.perform(context) == "rsi_only"
            else "robot_manager_node_extended"
        ),
        parameters=[
            driver_config,
            {
                "robot_models": [robot1_hw_name, robot2_hw_name],
                "use_gpio": False,
            },
        ],
        prefix=prefix_cmd,
    )

    robot_state_publisher = Node(
        namespace=ns,
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
        prefix=prefix_cmd,
    )

    # Spawn controllers
    def controller_spawner(controller_name, prefix_cmd, param_file=None, activate=False):
        arg_list = [
            controller_name,
            "-c",
            "controller_manager",
            "-n",
            ns,
        ]
        if param_file:
            arg_list.extend(["--param-file", param_file])
        if not activate:
            arg_list.append("--inactive")

        return Node(
            package="controller_manager",
            executable="spawner",
            prefix=prefix_cmd,
            arguments=arg_list,
        )

    controllers = {
        "joint_state_broadcaster": None,
        "joint_trajectory_controller": config_file(
            "joint_trajectory_controller_config_dual_arm.yaml"
        ),
        "event_broadcaster": event_broadcaster_config_file,
    }

    controller_spawners = [
        controller_spawner(name, prefix_cmd, param_file)
        for name, param_file in controllers.items()
    ]

    nodes_to_start = [
        control_node,
        robot_manager_node,
        robot_state_publisher,
    ] + controller_spawners

    return nodes_to_start


def generate_launch_description():
    launch_arguments = []

    # Global arguments
    launch_arguments.append(DeclareLaunchArgument("mode", default_value="hardware"))
    launch_arguments.append(
        DeclareLaunchArgument(
            "driver_version",
            default_value="rsi_only",
            choices=["rsi_only", "eki_rsi", "mxa_rsi"],
        )
    )
    launch_arguments.append(DeclareLaunchArgument("namespace", default_value=""))
    launch_arguments.append(
        DeclareLaunchArgument(
            "controller_config_dir",
            default_value=get_package_share_directory("kuka_rsi_driver") + "/config",
        )
    )
    launch_arguments.append(
        DeclareLaunchArgument(
            "event_broadcaster_robot_prefixes",
            default_value="robot1,robot2",
        )
    )
    launch_arguments.append(DeclareLaunchArgument("rt_core", default_value="-1"))
    launch_arguments.append(DeclareLaunchArgument("rt_prio", default_value="70"))
    launch_arguments.append(DeclareLaunchArgument("non_rt_cores", default_value=""))
    launch_arguments.append(DeclareLaunchArgument("lock_memory", default_value="true"))

    # Robot 1 arguments
    launch_arguments.append(DeclareLaunchArgument("robot1_model", default_value="kr6_r700_sixx"))
    launch_arguments.append(DeclareLaunchArgument("robot1_family", default_value="agilus"))
    launch_arguments.append(DeclareLaunchArgument("robot1_prefix", default_value="robot1_"))
    launch_arguments.append(DeclareLaunchArgument("robot1_client_ip", default_value="0.0.0.0"))
    launch_arguments.append(DeclareLaunchArgument("robot1_client_port", default_value="59152"))
    launch_arguments.append(DeclareLaunchArgument("robot1_mxa_client_port", default_value="1337"))
    launch_arguments.append(DeclareLaunchArgument("robot1_controller_ip", default_value="0.0.0.0"))
    launch_arguments.append(DeclareLaunchArgument("robot1_x", default_value="0"))
    launch_arguments.append(DeclareLaunchArgument("robot1_y", default_value="0"))
    launch_arguments.append(DeclareLaunchArgument("robot1_z", default_value="0"))
    launch_arguments.append(DeclareLaunchArgument("robot1_roll", default_value="0"))
    launch_arguments.append(DeclareLaunchArgument("robot1_pitch", default_value="0"))
    launch_arguments.append(DeclareLaunchArgument("robot1_yaw", default_value="0"))
    launch_arguments.append(DeclareLaunchArgument("robot1_roundtrip_time", default_value="4000"))
    launch_arguments.append(
        DeclareLaunchArgument("robot1_verify_robot_model", default_value="true", choices=["true", "false"])
    )
    launch_arguments.append(DeclareLaunchArgument("robot1_rsi_xml_config_file", default_value=""))
    launch_arguments.append(
        DeclareLaunchArgument("robot1_use_gpio", default_value="false", choices=["true", "false"])
    )
    launch_arguments.append(DeclareLaunchArgument("robot1_async_thread_priority", default_value="69"))
    launch_arguments.append(DeclareLaunchArgument("robot1_async_affinity", default_value="[]"))

    # Robot 2 arguments
    launch_arguments.append(DeclareLaunchArgument("robot2_model", default_value="kr6_r700_sixx"))
    launch_arguments.append(DeclareLaunchArgument("robot2_family", default_value="agilus"))
    launch_arguments.append(DeclareLaunchArgument("robot2_prefix", default_value="robot2_"))
    launch_arguments.append(DeclareLaunchArgument("robot2_client_ip", default_value="0.0.0.0"))
    launch_arguments.append(DeclareLaunchArgument("robot2_client_port", default_value="59153"))
    launch_arguments.append(DeclareLaunchArgument("robot2_mxa_client_port", default_value="1338"))
    launch_arguments.append(DeclareLaunchArgument("robot2_controller_ip", default_value="0.0.0.0"))
    launch_arguments.append(DeclareLaunchArgument("robot2_x", default_value="0"))
    launch_arguments.append(DeclareLaunchArgument("robot2_y", default_value="1.0"))
    launch_arguments.append(DeclareLaunchArgument("robot2_z", default_value="0"))
    launch_arguments.append(DeclareLaunchArgument("robot2_roll", default_value="0"))
    launch_arguments.append(DeclareLaunchArgument("robot2_pitch", default_value="0"))
    launch_arguments.append(DeclareLaunchArgument("robot2_yaw", default_value="0"))
    launch_arguments.append(DeclareLaunchArgument("robot2_roundtrip_time", default_value="4000"))
    launch_arguments.append(
        DeclareLaunchArgument("robot2_verify_robot_model", default_value="true", choices=["true", "false"])
    )
    launch_arguments.append(DeclareLaunchArgument("robot2_rsi_xml_config_file", default_value=""))
    launch_arguments.append(
        DeclareLaunchArgument("robot2_use_gpio", default_value="false", choices=["true", "false"])
    )
    launch_arguments.append(DeclareLaunchArgument("robot2_async_thread_priority", default_value="69"))
    launch_arguments.append(DeclareLaunchArgument("robot2_async_affinity", default_value="[]"))

    return LaunchDescription(launch_arguments + [OpaqueFunction(function=launch_setup)])
