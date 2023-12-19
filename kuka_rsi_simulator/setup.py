from setuptools import setup
import os
from glob import glob

package_name = 'kuka_rsi_simulator'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*.launch.py'))),
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*_launch.py'))),
        (os.path.join('share', package_name, 'config'),
            glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Marton Antal',
    maintainer_email='antal.marci@gmail.com',
    description='Simple package for simulating the KUKA RSI interface.',
    license='BSD',
    entry_points={
        'console_scripts': [
            'rsi_simulator = kuka_rsi_simulator.rsi_simulator:main'
        ],
    },
)
