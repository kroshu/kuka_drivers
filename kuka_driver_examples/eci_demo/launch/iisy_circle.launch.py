import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, 'r') as file:
            return file.read()
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("kuka_iisy")
        .robot_description(file_path=get_package_share_directory(
        'kuka_iisy_support') + "/urdf/iisy.urdf.xacro")
        .robot_description_semantic(file_path=get_package_share_directory(
        'kuka_iisy_support') + "/urdf/iisy.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_scene_monitor(
            publish_robot_description=True, publish_robot_description_semantic=True
        )
        .to_moveit_configs()
    )

    robot_description_config = load_file(
        'kuka_iisy_support', 'urdf/iisy.urdf')
    robot_description = {'robot_description': robot_description_config}

    rviz_config_file = get_package_share_directory(
        'kuka_iisy_support') + "/launch/urdf.rviz"

    controller_config = (get_package_share_directory('kuka_rox_hw_interface') +
                         "/config/ros2_controller_config.yaml")
    joint_traj_controller_config = (get_package_share_directory('kuka_rox_hw_interface') +
                                    "/config/joint_trajectory_controller_config.yaml")    

    controller_manager_node = "/controller_manager"

    return LaunchDescription([
        Node(
            package='kuka_rox_hw_interface',
            executable='rox_control_node',
            parameters=[robot_description, controller_config]
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
            arguments=["joint_state_broadcaster", "-c", controller_manager_node],
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_trajectory_controller", "-c", controller_manager_node, "-p",
                       joint_traj_controller_config]
        ),
        # Start the actual move_group node/action server
        Node(
            package="moveit_ros_move_group",
            executable="move_group",
            output="screen",
            parameters=[moveit_config.to_dict()],
        ),
        # Node(
        #     package="tf2_ros",
        #     executable="static_transform_publisher",
        #     name="static_transform_publisher",
        #     output="log",
        #     arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "base_link", "LBR3R760_iisy_fixedbase"],
        # ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="log",
            arguments=["-d", rviz_config_file, "--ros-args", "--log-level", "error"],
        )
    ])