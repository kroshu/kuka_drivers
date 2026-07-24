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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution

from launch_ros.actions import Node


def generate_launch_description():
    rsi_ip_address = DeclareLaunchArgument(
        "rsi_ip_address", default_value=TextSubstitution(text="127.0.0.1")
    )
    rsi_port = DeclareLaunchArgument("rsi_port", default_value=TextSubstitution(text="59152"))
    rsi_xml_config_file = DeclareLaunchArgument(
        "rsi_xml_config_file", default_value=TextSubstitution(text="")
    )
    ignore_ipoc = DeclareLaunchArgument(
        "ignore_ipoc", default_value=TextSubstitution(text="false")
    )

    return LaunchDescription(
        [
            rsi_ip_address,
            rsi_port,
            rsi_xml_config_file,
            ignore_ipoc,
            Node(
                package="kuka_rsi_simulator",
                executable="rsi_simulator",
                name="kuka_rsi_simulator",
                parameters=[
                    {
                        "rsi_ip_address": LaunchConfiguration("rsi_ip_address"),
                        "rsi_port": LaunchConfiguration("rsi_port"),
                        "rsi_xml_config_file": LaunchConfiguration("rsi_xml_config_file"),
                        "ignore_ipoc": LaunchConfiguration("ignore_ipoc"),
                    }
                ],
            ),
        ]
    )
