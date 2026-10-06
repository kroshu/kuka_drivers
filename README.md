# ROS2 KUKA Drivers

This repository combines community-supported code for legacy KUKA operating systems with officially supported, open-source driver packages from the [`kuka-ros/kuka_drivers`](https://github.com/kuka-ros/kuka_drivers) repository, included as the `upstream/kuka_drivers` Git submodule. The submodule provides the shared driver core, interfaces, RSI packages, and controllers; this repository contains drivers and controllers for legacy systems, including iiQKA EAC and Sunrise FRI.

KSS and iiQKA.OS2 use the same RSI-based ROS 2 driver from the upstream repository, only the controller-side setup differs between the two systems.

ROS2 Distro | Branch | Github CI | SonarCloud
------------ | -------------- | -------------- | --------------
**Jazzy** | [`master`](https://github.com/kuka-ros/kuka_drivers/tree/master) | [![Build Status](https://github.com/kuka-ros/kuka_drivers/actions/workflows/industrial_ci_jazzy.yml/badge.svg?branch=master)](https://github.com/kuka-ros/kuka_drivers/actions/workflows/industrial_ci_jazzy.yml?branch=master) | [![Quality Gate Status](https://sonarcloud.io/api/project_badges/measure?project=kuka-ros_kuka_drivers&metric=alert_status)](https://sonarcloud.io/dashboard?id=kuka-ros_kuka_drivers)
**Humble** | [`humble`](https://github.com/kuka-ros/kuka_drivers/tree/humble) | [![Build Status](https://github.com/kuka-ros/kuka_drivers/actions/workflows/industrial_ci_humble.yml/badge.svg?branch=humble)](https://github.com/kuka-ros/kuka_drivers/actions/workflows/industrial_ci_humble.yml?branch=humble) | [![Quality Gate Status](https://sonarcloud.io/api/project_badges/measure?project=kuka-ros_kuka_drivers&metric=alert_status&branch=humble)](https://dashboard.sonarcloud.io?id=kuka-ros_kuka_drivers)

## Requirements

The drivers require a system with ROS installed. It is recommended to use Ubuntu 22.04 with ROS Humble.

It is also recommended to use a client machine with a real-time kernel, as all three drivers require cyclic, real-time communication. Due to the real-time requirement, Windows systems are not recommended and covered in the documentation.

## Installation

### Installation as binary package

The driver is also available as a binary package. Installing the `kuka_drivers` metapackage will only install the packages strictly necessary for using the drivers. To install all available robot models, the `kuka_robot_descriptions` package should be also installed.

```bash
sudo apt install ros-jazzy-kuka-drivers
sudo apt install ros-jazzy-kuka-robot-descriptions
```

If due to lack of resources this is not intended, it is also possible to install support packages only for a single robot family.
```bash
sudo apt install ros-jazzy-kuka-drivers
sudo apt install ros-jazzy-kuka-agilus-support
```

> [!NOTE]
> As the ROS2 packages are not immediately available via apt after the release, it is possible that the installed version lacks some features already available on the development branch.

### Installation from source

The driver can be also built from source. The main advantage of this is to get features before they are released and available for installation. All configuration options should also be available if using the released binary packages.

Create ROS2 workspace (if not already created).

```bash
mkdir -p ~/ros2_ws/src
```

Clone KUKA ROS2 repositories.

```bash
cd ~/ros2_ws/src
git clone --recurse-submodules -b humble https://github.com/kuka-ros/kuka_drivers.git
vcs import --recursive < kuka_drivers/upstream.repos
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

## Getting Started

Documentation of this project can be found in the [doc/wiki](doc/wiki/Home.md) folder.

If you find something confusing, not working, or would like to contribute, please read our [contributing guide](CONTRIBUTING.md) before opening an issue or creating a pull request.
