# ROS2 KUKA Drivers

Experimental ROS2 driver for KUKA robots. Recommended distribution is [ROS 2 Humble Hawksbill:](https://docs.ros.org/en/humble/Installation.html).

> ROS2 Iron Irwini has breaking changes, thus it is not yet supported.

Github CI | SonarCloud
------------| ---------------
[![Build Status](https://github.com/kroshu//kuka_drivers/workflows/CI/badge.svg?branch=master)](https://github.com/kroshu/ros2_kuka_sunrise_fri_driver/actions) | [![Quality Gate Status](https://sonarcloud.io/api/project_badges/measure?project=kroshu_kuka_drivers&metric=alert_status)](https://sonarcloud.io/dashboard?id=kroshu_kuka_drivers)

# Installation

Create ROS2 workspace (if already not created).
```bash
mkdir -p ~/ros2_ws/src
```

Clone KUKA ROS2 repositories.
```bash
cd ~/ros2_ws/src
git clone https://github.com/kroshu/kuka_drivers.git
vcs import < kuka_drivers/upstream.repos
```

Install dependencies using `rosdep`.
```bash
cd ~/ros2_ws
sudo apt install python3-rosdep
sudo rosdep init
rosdep update
sudo apt upgrade
rosdep install --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y
```

Build KUKA packages.
```bash
cd ~/ros2_ws
colcon build
```

Source built KUKA packages.
```bash
# Replace ".bash" with your shell if you're not using bash
# Possible values are: setup.bash, setup.sh, setup.zsh
source ~/ros2_ws/install/setup.bash
```

# Get Started

Documentation of this project can be found on the repository's [Wiki](https://github.com/kroshu/kuka_drivers/wiki) page.