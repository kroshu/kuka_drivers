# Welcome to the ROS2 KUKA Sunrise driver project!

## Project description

This project centers on the development of a comprehensive ROS2 driver for external control of KUKA robots running on Sunrise OS through the real-time Fast Robot Interface (FRI). The following aspects are given particular consideration:

1. Application lifecycle controlled completely from ROS2, e.g.
   
   - Parameter configuration
   
   - Start/stop of external monitoring and commanding
   
   - Reacting to application errors

2. Exposing all functionality of the FRI towards the ROS2 system, e.g.
   
   - All command and control modes of joints
   
   - Field bus I/O handling

3. Exploiting new features of ROS2 w.r.t. ROS1, e.g.
   
   - Lifecycle management
   
   - Node-based ROS parameters
   
   - Real-time inter-node communication

The ROS2 KUKA Sunrise driver does not include a controller; its purpose is to expose a joint-level interface of robots running on Sunrise to the ROS2 system. The control loop is then expected to be closed by a connected joint controller node. This approach is contrary to how most real-time robot interfaces to ROS and ROS2 are realized, which would be to use the ros2\_control stack. The document [3. Relationship to ros2_control](3.-Relationship-to-ros2_control.md) provides more information on this topic.

## Current state of project

This project is currently experimental. The functionalities are not fully tested and the API and the internal architecture of the driver should be expected to change. This wiki is targeted at developers who would like to understand and potentially contribute to the project. Usage of this driver is discouraged except for the testing of the driver, only in controlled environment and with the proper safety configuration applied.

## Features

The following features of the FRI are exposed to ROS2:

#### Real-time interface

- [ ] Monitor joint states
  - [x] Actual position
  - [ ] Actual torque
  - [ ] Setpoint position
  - [ ] Setpoint torque
  - [x] External torque
- [ ] Joint commands
  - [x] Position
  - [x] Torque
- [ ] Field bus
  - [ ] Inputs
  - [ ] Outputs
- [ ] Connection quality
- [ ] Safety state
- [x] Tracking performance

#### Manager interface

- [x] Session state
- [x] Control mode
- [x] Client command mode
- [ ] Operation mode
- [ ] Number of axes

## Contact

If you have questions, suggestions or want to contribute, feel free to open an [issue](https://github.com/kroshu/ros2_kuka_sunrise/issues), start a [discussion](https://github.com/kroshu/ros2_kuka_sunrise/discussions) or drop an email to:

**Zoltán Rési** 

GitHub: https://github.com/resizoltan

Email: resizoltan@gmail.com
