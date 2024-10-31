# ROS2 KUKA Drivers

This repository contains ROS2 drivers for all KUKA operating systems.

ROS2 Distro | Branch | Github CI | SonarCloud
------------ | -------------- | -------------- | --------------
**Jazzy** | [`master`](https://github.com/kroshu/kuka_drivers/tree/master) | [![Build Status](https://github.com/kroshu//kuka_drivers/actions/workflows/industrial_ci_jazzy.yml/badge.svg?branch=master)](https://github.com/kroshu/kuka_drivers/actions/workflows/industrial_ci_jazzy.yml?branch=master) | [![Quality Gate Status](https://sonarcloud.io/api/project_badges/measure?project=kroshu_kuka_drivers&metric=alert_status)](https://sonarcloud.io/dashboard?id=kroshu_kuka_drivers)
**Humble** | [`humble`](https://github.com/kroshu/kuka_drivers/tree/humble) | [![Build Status](https://github.com/kroshu//kuka_drivers/actions/workflows/industrial_ci_humble.yml/badge.svg)](https://github.com/kroshu/kuka_drivers/actions/workflows/industrial_ci_humble.yml) | [![Quality Gate Status](https://sonarcloud.io/api/project_badges/measure?project=kroshu_kuka_drivers&metric=alert_status&branch=humble)](https://sonarcloud.io/dashboard?id=kroshu_kuka_drivers)

# Requirements
The drivers require a system with ROS installed. It is recommended to use Ubuntu 22.04 with ROS Humble.

It is also recommended to use a client machine with a real-time kernel, as all three drivers require cyclic, real-time communication. Due to the real-time requirement, Windows systems are not recommended and covered in the documentation.


# Installation
The driver is not available as a binary package, building from source is necessary.

Create ROS2 workspace (if not already created).
```bash
mkdir -p ~/ros2_ws/src
```

Clone KUKA ROS2 repositories.
```bash
cd ~/ros2_ws/src
git clone -b humble https://github.com/kroshu/kuka_drivers.git
vcs import < kuka_drivers/upstream.repos
```

Install and initialize rosdep (if not already done)
```bash
sudo apt install python3-rosdep
sudo rosdep init
```

Install dependencies using `rosdep`.
```bash
cd ~/ros2_ws
rosdep update
sudo apt upgrade
rosdep install --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y
```

Build all packages in workspace.
```bash
cd ~/ros2_ws
colcon build
```

Source workspace.
```bash
# Replace ".bash" with your shell if you're not using bash
# Possible values are: setup.bash, setup.sh, setup.zsh
source ~/ros2_ws/install/setup.bash
```

# Getting Started
Documentation of this project can be found on the repository's [Wiki](https://github.com/kroshu/kuka_drivers/wiki) page.

If you find something confusing, not working, or would like to contribute, please read our [contributing guide](CONTRIBUTING.md) before opening an issue or creating a pull request.
