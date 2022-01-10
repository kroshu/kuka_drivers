# Copyright 2020 Zoltán Rési
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
            package='kuka_sunrise', executable='robot_manager_node', output='screen',
            name=[launch.substitutions.LaunchConfiguration('node_prefix'), 'robot_manager'],
            parameters=[{'controller_ip': '<insert ip here>'}]),
        launch_ros.actions.LifecycleNode(
            package='kuka_sunrise', executable='robot_control_node', output='screen',
            name=[launch.substitutions.LaunchConfiguration('node_prefix'), 'robot_control'])
        ])
