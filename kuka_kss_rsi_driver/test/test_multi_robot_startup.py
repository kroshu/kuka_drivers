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

import unittest

import launch
import launch.actions
import launch_testing.actions
import launch_testing.markers
import pytest

from launch.launch_description_sources.python_launch_description_source import (
    PythonLaunchDescriptionSource,
)
from launch.actions.include_launch_description import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory


# Launch 2 drivers with different namespaces
@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    test_config_dir = get_package_share_directory("kuka_kss_rsi_driver") + "/test/config/"
    return launch.LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        get_package_share_directory("kuka_kss_rsi_driver"),
                        "/launch/",
                        "startup.launch.py",
                    ]
                ),
                launch_arguments={
                    "namespace": "test1",
                    "controller_config": f"{test_config_dir + 'test1_ros2_controller_config.yaml'}",  # noqa: E501
                    "jtc_config": f"{test_config_dir + 'test1_joint_trajectory_controller_config.yaml'}",  # noqa: E501
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        get_package_share_directory("kuka_kss_rsi_driver"),
                        "/launch/",
                        "startup.launch.py",
                    ]
                ),
                launch_arguments={
                    "namespace": "test2",
                    "controller_config": f"{test_config_dir + 'test2_ros2_controller_config.yaml'}",  # noqa: E501
                    "jtc_config": f"{test_config_dir + 'test2_joint_trajectory_controller_config.yaml'}",  # noqa: E501
                    "x": "2",
                }.items(),
            ),
            launch_testing.actions.ReadyToTest(),
        ]
    )


class TestMultiStartup(unittest.TestCase):
    def test_read_stdout(self, proc_output):
        # Check for successful initialization
        proc_output.assertWaitFor("got segment test1_base", timeout=20)
        proc_output.assertWaitFor("got segment test2_base", timeout=20)
        proc_output.assertWaitFor(
            "Successful initialization of hardware 'test1_kr6_r700_sixx'", timeout=20
        )
        proc_output.assertWaitFor(
            "Successful initialization of hardware 'test2_kr6_r700_sixx'", timeout=20
        )
        # Check whether disabling automatic activation was successful
        proc_output.assertWaitFor(
            "Setting component 'test1_kr6_r700_sixx' to 'unconfigured' state.", timeout=20
        )
        proc_output.assertWaitFor(
            "Setting component 'test2_kr6_r700_sixx' to 'unconfigured' state.", timeout=20
        )
