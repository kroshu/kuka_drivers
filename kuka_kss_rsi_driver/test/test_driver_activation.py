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
from ament_index_python.packages import get_package_share_directory
from launch.actions.include_launch_description import IncludeLaunchDescription
from launch.launch_description_sources.python_launch_description_source import (
    PythonLaunchDescriptionSource,
)


# Launch driver startup
@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    return launch.LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        get_package_share_directory("kuka_kss_rsi_driver"),
                        "/launch/",
                        "startup.launch.py",
                    ]
                )
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        get_package_share_directory("kuka_rsi_simulator"),
                        "/launch/",
                        "kuka_rsi_simulator.launch.py",
                    ]
                )
            ),
            launch.actions.TimerAction(
                period=10.0,
                actions=[
                    launch.actions.ExecuteProcess(
                        cmd=["ros2", "lifecycle", "set", "robot_manager", "configure"],
                        output="screen",
                    ),
                ],
            ),
            launch.actions.TimerAction(
                period=15.0,
                actions=[
                    launch.actions.ExecuteProcess(
                        cmd=["ros2", "lifecycle", "set", "robot_manager", "activate"],
                        output="screen",
                    ),
                ],
            ),
            launch_testing.actions.ReadyToTest(),
        ]
    )


class TestDriverActivation(unittest.TestCase):
    def test_read_stdout(self, proc_output):
        return  # This test would require an EKI server, which we don't have
        # Check for successful initialization
        proc_output.assertWaitFor("Robot initialized", timeout=5)
        proc_output.assertWaitFor(
            "Successful initialization of hardware 'kr6_r700_sixx'", timeout=5
        )
        # Check whether disabling automatic activation was successful
        proc_output.assertWaitFor(
            "Setting component 'kr6_r700_sixx' to 'unconfigured' state.", timeout=5
        )
        # Check for successful configuration and activation
        proc_output.assertWaitFor("Successful 'configure' of hardware 'kr6_r700_sixx'", timeout=15)
        proc_output.assertWaitFor("Successful 'activate' of hardware 'kr6_r700_sixx'", timeout=20)
