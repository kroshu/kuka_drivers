# Welcome to the ROS2 KUKA drivers project!

## KUKA Sunrise driver (FRI)

### Project description

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

### Current state of project

This project is currently experimental. The functionalities are not fully tested and the API and the internal architecture of the driver should be expected to change. This wiki is targeted at developers who would like to understand and potentially contribute to the project. Usage of this driver is discouraged except for the testing of the driver, only in controlled environment and with the proper safety configuration applied.

### Features

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

## KUKA KSS driver (RSI)

Another project in the repo centers on the development of a ROS2 driver for KSS robots through Robot Sensor Interface (RSI). It is in an experimental state, with only joint angle states and commands available. The structure of the driver is similiar to that of the Sunrise driver and the same interfaces are opened, so the same joint controller can be used to move robots. Howewer this controller should be improved, as it allows big torques that stop the robot for machine protection reasons.

## KUKA RoX driver (ECI)

The third project in the repo is the driver for iiQKA robots. 

### Architecture

The driver uses the ros2_control framework, so a variaty of controllers are supported and it can be easily integrated into moveit. It consists of a realtime component for controlling the robot via UDP (ROS2 hardware interface) and a non-realtime one for lifecycle management of the controllers and the hardware interface. The driver supports a control cycle time of 4 milliseconds, but the round-trip time of one cycle should not exceed 3 milliseconds, as above that the packets are considered lost. Therefore it is advised to run the driver on a realtime-capable Linux machine (with the PRREMPT_RT patch applied). After a few lost packets the connection is considered not stable enough and external control is ended.

The driver depends on some KUKA-specific packages, which are only available with the real robot, but setting the MOCK_HW_ONLY flag in the hardware_interface enables the usage of the driver in a simulated way, so that motion planning problems can be tried out with the same components running.
Two additional packages (not listed in package.xml) must be installed with apt:
- libnanopb-dev
- libgrpc++-dev

### Setup

By default, the mock libraries are used, this can be changed in the cmake file by setting MOCK_KUKA_LIBS to FALSE before building.

The IP addresses of the client machine and controller must be given in the *config/eci_config.yaml* configuration file. A rebuild is not needed after the changes, but the file has to be modified before starting the nodes. The control mode of the robot can also be modified in the same configuration file: you can choose either 1 (POSITION_CONTROL), 3 (JOINT_IMPEDANCE_CONTROL) or 5 (TORQUE_CONTROL). This also sets the control_mode parameter of the robot manager node, which can be only modified at startup, control mode changes are not supported in runtime at the current state.

Besides, the setting of scheduling priorities must be allowed for your user (extend /etc/security/limits.conf with "username	 -	 rtprio		 98" and restart) to enable real-time performance.

### Usage

The usage of the driver is quite simple, one has to start the launch file in the package to start all required nodes (it also starts an rviz node for visualisation, which can be commented out if not needed):

**`ros2 launch kuka_rox_hw_interface startup.launch.py`**

After all components have started successfully, the system needs to be configured and activated to start external control:

**`ros2 lifecycle set robot_manager configure`**


**`ros2 lifecycle set robot_manager activate`**

On successful activation the robot controller and the driver start communication with a 4 ms cycle time, and it is possible to move the robot through the joint trajectory controller. The easiest way to achieve this is to start an rqt_joint_trajcectory controller and move the joints with cursors or one can also execute trajectories planned with moveit - an example of this can be found in kuka_sunrise_control/eci_demo package.

To stop external control, the components have to be deactivated with **`ros2 lifecycle set robot_manager deactivate`**

BEWARE, that this is a non-realtime process including lifecycle management, so the connection is not terminated immediately, in cases where an abrupt stop is needed, the safety stop of the SmartPad should be used!

It is also possible to use different controllers with some modifications in the launch and yaml files (for example ForwardCommandController, which forwards the commands send to a ROS2 topic towards the robot). In these cases, one has to make sure, that the commands sent to the robot are close to the current position, otherwise the machine protection will stop the robot movement.

### Issues

The driver is in an experimental state, with only joint position commands supported. We have encountered the following isses:
- If there is an error after starting the launch file, the process must be stopped and started again. This error is related to spawning the joint_trajectory controller and comes forth sporadically.
The logs are the following:

    [spawner-4] Node not found

    [ERROR] [spawner-4]: process has died [pid ..., exit code 1, cmd '/opt/ros/humble/lib/controller_manager/spawner joint_trajectory_controller -c /controller_manager -p .../kuka_rox_hw_interface/share/kuka_rox_hw_interface/config/joint_trajectory_controller_config.yaml --inactive --ros-args']

- If you activate and deactivate the robot manager node, the reactivation will fail, you must restart the whole process if you want to control the robot again



## Contact

If you have questions, suggestions or want to contribute, feel free to open an [issue](https://github.com/kroshu/ros2_kuka_sunrise/issues) or start a [discussion](https://github.com/kroshu/ros2_kuka_sunrise/discussions).

