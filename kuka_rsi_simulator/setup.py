# Copyright 2022 Márton Antal
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
# limitations under the License.from launch import LaunchDescription

from setuptools import setup
import os
from glob import glob

package_name = "kuka_rsi_simulator"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "launch"),
            glob(os.path.join("launch", "*.launch.py")),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Marton Antal",
    maintainer_email="antal.marci@gmail.com",
    description="Simple package for simulating the KUKA RSI interface.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": ["rsi_simulator = kuka_rsi_simulator.rsi_simulator:main"],
    },
)
