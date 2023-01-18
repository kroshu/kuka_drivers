from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    position_goals = PathJoinSubstitution(
        [
            FindPackageShare("eci_demo"),
            "config",
            "dummy_publisher.yaml",
        ]
    ) 
    return LaunchDescription(
        [
            Node(
                package="ros2_control_test_nodes",
                executable="publisher_joint_trajectory_controller",
                name="publisher_joint_trajectory_controller",
                parameters=[position_goals],
                output="both",
            )
        ]
    )