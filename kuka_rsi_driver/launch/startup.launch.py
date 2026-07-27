# Copyright 2023 KUKA Hungaria Kft.
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
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import LifecycleNode, Node
from launch_ros.substitutions import FindPackageShare

COMPOSED_TEMPLATE_XACRO = "robot_with_external_axis_template.urdf.xacro"


def _ros2_control_macro_file_from_family(robot_family):
    if robot_family.startswith("lbr_"):
        return f"{robot_family}_ros2_control_macro.xacro"
    return f"kr_{robot_family}_ros2_control_macro.xacro"


def launch_setup(context, *args, **kwargs):
    robot_model = LaunchConfiguration("robot_model")
    robot_family = LaunchConfiguration("robot_family")
    use_external_axis = LaunchConfiguration("use_external_axis")
    kl_model = LaunchConfiguration("kl_model")
    kl_support_package = LaunchConfiguration("kl_support_package")
    kl_prefix = LaunchConfiguration("kl_prefix")
    kl_ros2_control_macro_file = LaunchConfiguration("kl_ros2_control_macro_file")
    kl_ros2_control_joints_macro = LaunchConfiguration("kl_ros2_control_joints_macro")
    mode = LaunchConfiguration("mode")
    use_gpio = LaunchConfiguration("use_gpio")
    driver_version = LaunchConfiguration("driver_version")
    client_ip = LaunchConfiguration("client_ip")
    client_port = LaunchConfiguration("client_port")
    mxa_client_port = LaunchConfiguration("mxa_client_port")
    controller_ip = LaunchConfiguration("controller_ip")
    x = LaunchConfiguration("x")
    y = LaunchConfiguration("y")
    z = LaunchConfiguration("z")
    roll = LaunchConfiguration("roll")
    pitch = LaunchConfiguration("pitch")
    yaw = LaunchConfiguration("yaw")
    roundtrip_time = LaunchConfiguration("roundtrip_time")
    verify_robot_model = LaunchConfiguration("verify_robot_model")
    rsi_xml_config_file = LaunchConfiguration("rsi_xml_config_file")
    ns = LaunchConfiguration("namespace")
    controller_config = LaunchConfiguration("controller_config")
    jtc_config = LaunchConfiguration("jtc_config")
    gpio_config = LaunchConfiguration("gpio_config")
    non_rt_cores = LaunchConfiguration("non_rt_cores")
    rt_core = LaunchConfiguration("rt_core")
    rt_prio = LaunchConfiguration("rt_prio")
    lock_memory = LaunchConfiguration("lock_memory")
    enable_rsi_monitoring = LaunchConfiguration("enable_rsi_monitoring")
    if ns.perform(context) == "":
        tf_prefix = ""
    else:
        tf_prefix = ns.perform(context) + "_"

    # Parse allowed cores into a list of integers; allow formats like "2,3, 4" or "  "
    cores = []
    for part in non_rt_cores.perform(context).split(","):
        part = part.strip()
        if part == "":
            continue
        try:
            cores.append(int(part))
        except ValueError:
            raise RuntimeError(
                f"Invalid allowed_cores entry: '{part}'. "
                "Provide a comma-separated list of integers, e.g. '2,3,4'."
            )

    # Compute the prefix: None if no cores; otherwise build 'taskset -c <list>'
    prefix_cmd = None
    if cores:
        # Build the string "2,3,4" for taskset
        core_list_str = ",".join(str(c) for c in cores)
        prefix_cmd = f"taskset -c {core_list_str}"

    if not controller_config.perform(context):
        rel_path_to_config_file = (
            "/config/ros2_controller_config_rsi_only.yaml"
            if driver_version.perform(context) == "rsi_only"
            else "/config/ros2_controller_config_extended.yaml"
        )
        controller_config = (
            get_package_share_directory("kuka_rsi_driver") + rel_path_to_config_file
        )

    robot_model_value = robot_model.perform(context)
    robot_family_value = robot_family.perform(context)
    use_external_axis_value = use_external_axis.perform(context) == "true"
    kl_model_value = kl_model.perform(context)
    kl_support_package_value = kl_support_package.perform(context)
    kl_prefix_value = kl_prefix.perform(context)
    kl_ros2_control_macro_file_value = kl_ros2_control_macro_file.perform(context)
    kl_ros2_control_joints_macro_value = kl_ros2_control_joints_macro.perform(context)

    robot_support_package = f"kuka_{robot_family_value}_support"
    urdf_source = PathJoinSubstitution(
        [FindPackageShare(robot_support_package), "urdf", robot_model_value + ".urdf.xacro"]
    )
    effective_robot_model = robot_model_value
    template_xacro_args = []

    if use_external_axis_value:
        kl_support_package = kl_support_package_value or "kuka_kl_support"
        robot_ros2_control_macro_file = _ros2_control_macro_file_from_family(robot_family_value)

        robot_model_macro_path = os.path.join(
            get_package_share_directory(robot_support_package),
            "urdf",
            robot_model_value + "_macro.xacro",
        )
        if not os.path.isfile(robot_model_macro_path):
            raise RuntimeError(
                f"Robot model macro file was not found: {robot_model_macro_path}. "
                "Check robot_model/robot_family values."
            )

        robot_ros2_control_macro_path = os.path.join(
            get_package_share_directory(robot_support_package),
            "urdf",
            robot_ros2_control_macro_file,
        )
        if not os.path.isfile(robot_ros2_control_macro_path):
            raise RuntimeError(
                f"Robot ros2_control macro file was not found: {robot_ros2_control_macro_path}."
            )

        kl_model_macro_path = os.path.join(
            get_package_share_directory(kl_support_package),
            "urdf",
            kl_model_value + "_macro.xacro",
        )
        if not os.path.isfile(kl_model_macro_path):
            raise RuntimeError(
                f"KL model macro file was not found: {kl_model_macro_path}. "
                "Check kl_model/kl_support_package values."
            )

        kl_ros2_control_macro_path = os.path.join(
            get_package_share_directory(kl_support_package),
            "urdf",
            kl_ros2_control_macro_file_value,
        )
        if not os.path.isfile(kl_ros2_control_macro_path):
            raise RuntimeError(
                f"KL ros2_control macro file was not found: {kl_ros2_control_macro_path}."
            )

        urdf_source = PathJoinSubstitution(
            [FindPackageShare("kuka_resources"), "urdf", COMPOSED_TEMPLATE_XACRO]
        )
        template_xacro_args = [
            " ",
            "robot_model:=",
            robot_model_value,
            " ",
            "robot_support_package:=",
            robot_support_package,
            " ",
            "robot_family:=",
            robot_family_value,
            " ",
            "kl_support_package:=",
            kl_support_package,
            " ",
            "robot_ros2_control_macro_file:=",
            robot_ros2_control_macro_file,
            " ",
            "kl_ros2_control_macro_file:=",
            kl_ros2_control_macro_file_value,
            " ",
            "kl_model:=",
            kl_model_value,
            " ",
            "kl_ros2_control_joints_macro:=",
            kl_ros2_control_joints_macro_value,
            " ",
            "rsi_xml_config_file:=",
            rsi_xml_config_file,
        ]
        effective_robot_model = f"{robot_model_value}_with_{kl_model_value}"

    jtc_config_param = jtc_config
    if jtc_config.perform(context) == "":
        jtc_config_file = (
            "joint_trajectory_controller_config_6_axis_kl.yaml"
            if use_external_axis_value
            else "joint_trajectory_controller_config.yaml"
        )
        jtc_config_param = (
            get_package_share_directory("kuka_rsi_driver") + "/config/" + jtc_config_file
        )

    # Get URDF via xacro
    xacro_arguments = [
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        urdf_source,
        " ",
        "mode:=",
        mode,
        " ",
        "use_gpio:=",
        use_gpio,
        " ",
        "driver_version:=",
        driver_version,
        " ",
        "client_port:=",
        client_port,
        " ",
        "mxa_client_port:=",
        mxa_client_port,
        " ",
        "client_ip:=",
        client_ip,
        " ",
        "controller_ip:=",
        controller_ip,
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
        "verify_robot_model:=",
        verify_robot_model,
    ]
    if use_external_axis_value:
        xacro_arguments.extend(
            [
                " ",
                "kl_prefix:=",
                kl_prefix_value,
                " ",
                "composed_model:=",
                effective_robot_model,
            ]
        )
        xacro_arguments.extend(template_xacro_args)

    robot_description_content = Command(xacro_arguments, on_stderr="capture")
    robot_description = {"robot_description": robot_description_content}

    # The driver config contains only parameters that can be changed after startup
    driver_config = get_package_share_directory("kuka_rsi_driver") + "/config/driver_config.yaml"

    control_node = Node(
        namespace=ns,
        package="kuka_drivers_core",
        executable="control_node",
        parameters=[
            robot_description,
            controller_config,
            {
                "cpu_affinity": int(rt_core.perform(context)),
                "thread_priority": int(rt_prio.perform(context)),
                "lock_memory": lock_memory.perform(context) == "true",
                "hardware_components_initial_state": {
                    "unconfigured": [tf_prefix + effective_robot_model]
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
            {"robot_model": effective_robot_model, "use_gpio": use_gpio},
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

        # Add param-file if it's provided
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
        "joint_trajectory_controller": jtc_config_param,
        "event_broadcaster": None,
    }

    if use_gpio.perform(context) == "true":
        controllers["gpio_controller"] = gpio_config

    if driver_version.perform(context) in {"eki_rsi", "mxa_rsi"}:
        controllers["control_mode_handler"] = None
        controllers["kss_message_handler"] = None

    controller_spawners = [
        controller_spawner(name, prefix_cmd, param_file)
        for name, param_file in controllers.items()
    ]

    nodes_to_start = [
        control_node,
        robot_manager_node,
        robot_state_publisher,
    ] + controller_spawners

    if enable_rsi_monitoring.perform(context) == "true":
        monitor_rsi_port = int(client_port.perform(context))
        nodes_to_start.append(
            Node(
                namespace=ns,
                package="kuka_rsi_driver",
                executable="rsi_monitor_node.py",
                parameters=[
                    {
                        "rsi_port": monitor_rsi_port,
                    }
                ],
            )
        )

    return nodes_to_start


def generate_launch_description():
    launch_arguments = []
    launch_arguments.append(DeclareLaunchArgument("robot_model", default_value="kr6_r700_sixx"))
    launch_arguments.append(DeclareLaunchArgument("robot_family", default_value="agilus"))
    launch_arguments.append(
        DeclareLaunchArgument(
            "use_external_axis",
            default_value="false",
            choices=["true", "false"],
            description=("Compose robot_model and kl_model with reusable template xacro."),
        )
    )
    launch_arguments.append(DeclareLaunchArgument("kl_model", default_value="kl100_2"))
    launch_arguments.append(
        DeclareLaunchArgument(
            "kl_support_package",
            default_value="",
            description=(
                "Package containing KL model and KL ros2_control xacro macros. "
                "If empty, falls back to kuka_kl_support."
            ),
        )
    )
    launch_arguments.append(
        DeclareLaunchArgument(
            "kl_ros2_control_macro_file",
            default_value="kl_ros2_control_macro.xacro",
            description=(
                "External-axis ros2_control macro file inside <kl_support_package>/urdf."
            ),
        )
    )
    launch_arguments.append(
        DeclareLaunchArgument(
            "kl_ros2_control_joints_macro",
            default_value="kuka_kl_ros2_control_joints",
            description=(
                "External-axis ros2_control joints macro name used by the composed URDF template."
            ),
        )
    )
    launch_arguments.append(DeclareLaunchArgument("kl_prefix", default_value="rail_"))
    launch_arguments.append(DeclareLaunchArgument("mode", default_value="hardware"))
    launch_arguments.append(
        DeclareLaunchArgument("use_gpio", default_value="false", choices=["true", "false"])
    )
    launch_arguments.append(
        DeclareLaunchArgument(
            "driver_version",
            default_value="rsi_only",
            description="Select the driver version to use",
            choices=["rsi_only", "eki_rsi", "mxa_rsi"],
        )
    )
    launch_arguments.append(DeclareLaunchArgument("namespace", default_value=""))
    launch_arguments.append(DeclareLaunchArgument("client_ip", default_value="0.0.0.0"))
    launch_arguments.append(DeclareLaunchArgument("client_port", default_value="59152"))
    launch_arguments.append(DeclareLaunchArgument("mxa_client_port", default_value="1337"))
    launch_arguments.append(DeclareLaunchArgument("controller_ip", default_value="0.0.0.0"))
    launch_arguments.append(DeclareLaunchArgument("x", default_value="0"))
    launch_arguments.append(DeclareLaunchArgument("y", default_value="0"))
    launch_arguments.append(DeclareLaunchArgument("z", default_value="0"))
    launch_arguments.append(DeclareLaunchArgument("roll", default_value="0"))
    launch_arguments.append(DeclareLaunchArgument("pitch", default_value="0"))
    launch_arguments.append(DeclareLaunchArgument("yaw", default_value="0"))
    launch_arguments.append(DeclareLaunchArgument("roundtrip_time", default_value="4000"))
    launch_arguments.append(
        DeclareLaunchArgument(
            "verify_robot_model", default_value="true", choices=["true", "false"]
        )
    )
    launch_arguments.append(
        DeclareLaunchArgument(
            "rsi_xml_config_file",
            default_value="",
            description=(
                "Absolute path to an RSI XML config YAML file. "
                "When set, configures the XML element/attribute names used in RSI messages. "
                "Leave empty to use the SDK defaults."
            ),
        )
    )
    launch_arguments.append(DeclareLaunchArgument("controller_config", default_value=""))
    launch_arguments.append(
        DeclareLaunchArgument(
            "jtc_config",
            default_value="",
            description=(
                "Optional JTC config file. Empty selects defaults from kuka_rsi_driver config: "
                "6-axis for standard setups, 6-axis+KL for use_external_axis=true."
            ),
        )
    )
    launch_arguments.append(
        DeclareLaunchArgument(
            "gpio_config",
            default_value=get_package_share_directory("kuka_rsi_driver")
            + "/config/gpio_controller_config.yaml",
        )
    )
    launch_arguments.append(
        DeclareLaunchArgument(
            "rt_core",
            default_value="-1",  # -1 means do not pin to core
            description=("CPU core index for taskset pinning of the RT thread"),
        )
    )
    launch_arguments.append(
        DeclareLaunchArgument(
            "rt_prio",
            default_value="70",
            description=("The priority of the thread that runs the control loop"),
        )
    )
    launch_arguments.append(
        DeclareLaunchArgument(
            "non_rt_cores",
            default_value="",
            description=(
                "Comma-separated CPU core indices for taskset pinning of non-RT threads "
                "(e.g. '2,3,4'). Leave empty to disable pinning."
            ),
        )
    )
    launch_arguments.append(
        DeclareLaunchArgument(
            "enable_rsi_monitoring", default_value="false", choices=["true", "false"]
        )
    )
    launch_arguments.append(
        DeclareLaunchArgument(
            "lock_memory",
            default_value="true",
            description=(
                "Whether to lock memory of the control loop with mlockall to avoid paging"
            ),
        )
    )

    return LaunchDescription(launch_arguments + [OpaqueFunction(function=launch_setup)])
