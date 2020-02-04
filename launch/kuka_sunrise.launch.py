from launch import LaunchDescription
import launch.actions
import launch.substitutions
import launch_ros.actions


def generate_launch_description():
    return LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            'node_prefix',
            default_value=[''],
            description='prefix for node names'),
        launch_ros.actions.LifecycleNode(
            package='kuka_sunrise', node_executable='robot_manager_node', output='screen',
            node_name=[launch.substitutions.LaunchConfiguration('node_prefix'), 'robot_manager']),
        launch_ros.actions.LifecycleNode(
            package='kuka_sunrise', node_executable='robot_control_node', output='screen',
            node_name=[launch.substitutions.LaunchConfiguration('node_prefix'), 'robot_control'])
        ])
