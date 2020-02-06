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

from setuptools import find_packages, setup


setup(
    name='kuka_sunrise',
    version='0.0.1',
    author='Zoltán Rési',
    author_email='resizoltan@gmail.com',
    description='Copyright entry point for ament_copyright linter',
    classifiers=[
        'Programming Language :: Python :: 3',
        'License :: OSI Approved :: Apache Software License'
    ],
    entry_points={
        'ament_copyright.copyright_name': [
            'resizoltan = scripts.copyright.copyright:resizoltan',
        ],
    },
    packages=find_packages(),
)
