from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    impedance_config = PathJoinSubstitution(
        [
            FindPackageShare("eci_demo"),
            "config",
            "dummy_impedance_publisher.yaml",
        ]
    ) 
    return LaunchDescription( ## TODO: create a dummy publisher script in the ros2_control_test_nodes repo
        [
            Node(
                package="ros2_control_test_nodes",
                executable="publisher_joint_impedance_controller",
                name="publisher_joint_impedance_controller",
                parameters=[impedance_config],
                output="both",
            )
        ]
    )