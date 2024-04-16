# Copyright 2024 Aron Svastits
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

from launch.launch_description_sources.python_launch_description_source import (
    PythonLaunchDescriptionSource,
)
from launch.actions.include_launch_description import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription


def generate_launch_description():
    test_config_dir = get_package_share_directory("kuka_iiqka_eac_driver") + "/test/config/"
    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        get_package_share_directory("kuka_iiqka_eac_driver"),
                        "/launch/",
                        "startup_with_rviz.launch.py",
                    ]
                ),
                launch_arguments={
                    "namespace": "test1",
                    "controller_config": f"{test_config_dir + 'test1_ros2_controller_config.yaml'}",  # noqa: E501
                    "jtc_config": f"{test_config_dir + 'test1_joint_trajectory_controller_config.yaml'}",  # noqa: E501
                    "jic_config": f"{test_config_dir + 'test1_joint_impedance_controller_config.yaml'}",  # noqa: E501
                    "ec_config": f"{test_config_dir + 'test1_effort_controller_config.yaml'}",
                    "rviz_config": f"{get_package_share_directory('iiqka_moveit_example') + '/config/multi_robot.rviz'}",  # noqa: E501
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        get_package_share_directory("kuka_iiqka_eac_driver"),
                        "/launch/",
                        "startup.launch.py",
                    ]
                ),
                launch_arguments={
                    "namespace": "test2",
                    "controller_config": f"{test_config_dir + 'test2_ros2_controller_config.yaml'}",  # noqa: E501
                    "jtc_config": f"{test_config_dir + 'test2_joint_trajectory_controller_config.yaml'}",  # noqa: E501
                    "jic_config": f"{test_config_dir + 'test2_joint_impedance_controller_config.yaml'}",  # noqa: E501
                    "ec_config": f"{test_config_dir + 'test2_effort_controller_config.yaml'}",
                    "robot_model": "lbr_iisy11_r1300",
                    "x": "2",
                }.items(),
            ),
        ]
    )
