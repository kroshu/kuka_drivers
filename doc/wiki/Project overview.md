# Welcome to the ROS2 KUKA drivers project!

## Goals of the project

This project aims to provide reliable real-time capable drivers for all KUKA robots. Currently KUKA robots are available with 3 different operating systems with real-time control API-s:
- KSS supporting industrial robots, with Robot Sensor Interface (RSI)
- Sunrise supporting cobots (LBR iiwa-s), with Fast Robot Interface (FRI)
- iiQKA supporting cobots (LBR iisy-s), with ExternalAPI.Control (EAC)

It is also the goal of this project to provide the same API for all three OS-s, hiding the underlying startup procedure and communication technology, thus enabling changing seamlessly to a different KUKA OS.

Additionally the aim was to write high quality, maintainable code with standardized interfaces, that conforms with the standards defined by the [ROS-Industrial project](https://www.rosin-project.eu/).

## Common interface

Two different interfaces should be defined for all drivers supporting real-time control:
- real-time interface, defining how to handle the cyclic dataflow
- non-real-time interface, defining the startup procedure with optional configuration

#### Real-time interface
The choice for the real-time interface was straightforward, as a standardized control framework exists for ROS2, called `ros2_control`, also supported by ROS-Industrial. The drivers are built using this framework, therefore it is recommended the read through its [documentation](https://control.ros.org/master/doc/ros2_control/doc/index.html), as this documentation builds on the knowledge of the framework.

All 3 of the KUKA real-time interfaces handle the timing on the controller side, so external control is always synchronized with the internal control cycle. This means, that callig the `read` method of the `controller_manager` cannot return immediately, but has to wait until the controller sends an update, which is triggered by the internal clock. Because of this blocking read, the deafult `ros2_control_node` cannot be used, as there it is expected that the `controller_manager` handles the timing according to the configured control frequency. Therefore a [custom control node](https://github.com/kroshu/kuka_drivers/blob/master/kuka_drivers_core/src/control_node.cpp) was implemented that uses the `controller_manager` and all other tools of `ros2_control`, but leaves the time management to the robot controller.

This change does not influence the API of the `ros2_control` framework, the real-time dataflow can be accessed by any controller.

#### Non-real-time interface
The startup procedure for any system in ROS can be defined using a launch file, that can start multiple processes. By default, starting the control node with a hardware interface and controllers configured immediately starts external control. This behaviour has a few drawbacks:
- The user cannot configure some parameters during runtime, that cannot be changed during external control
- The user cannot easily synchronize the start of external control with other components of the system
- It can cause unexpected behaviour, which can be potentially dangerous to the hardware or surroundings.
    - In torque control mode, the robot can start to move, if the torque sensors are not perfectly calibrated (and as that is hard to achieve, this can happen in most cases). This could be mitigated with a simple torque controller that tries to hold the position, but there is no guarantee that loading and activating the controller was successful, external control would start even if it failed. This could result in a scenario, where the robot starts to move unexpectedly.

The last issue should be certainly prevented from happening, therefore it was decided to extend the default startup procedure with a [lifecycle interface](https://design.ros2.org/articles/node_lifecycle.html), that synchronizes all components of the driver. The harware interfaces and controllers already have a lifecycle interface, but by default they are loaded and activated at startup. This configuration was modified to only load the hardware interfaces and controllers, configuration and activation is handled by a custom a lifecycle node, called `robot_manager`. The 3 states of the `robot_manager` node have the following meaning:

- `unconfigured`: all necessary components are started, but no connection is needed to the robot
- `configured`: The driver has configured the parameters necessary to start external control. Connection to the robot might be needed. (All of the parameters have default values in the driver, which are set on the robot controller during configuration.) A few [configuration controllers](https://github.com/kroshu/kuka_controllers?tab=readme-ov-file#configuration-controllers) might be active, that handle the runtime parameters of the hardware interface.
- `active`: external control is running with cyclic real-time communication, controllers are active

To achieve these synchronized states, the state transitions of the system do the following steps (implemented by the launch file and the `robot_manager` node):
- startup: all components of the system are started: `control_node`, `robot_manager` node, `robot_state_publisher` (optionally `rviz`) and all of the necessary controllers are loaded and configured
- `configure`: activate configuration controllers, configure hardware interface
- `activate`: activate real-time controllers, activate hardware interface
- `deactivate`: deactivate hardware interface, deactivate real-time controllers
- `cleanup`: clean up hardware interface, deactivate configuration controllers

Note: the lifecycle interface of the controllers are a little bit different, as they do not have a `cleanup` transition. To have a consequent `unconfigured` state, the configuration of the controllers are not handled in the `configure` transition, but at startup.

Including the controller state handling in the system state makes the implementation more complex, as controllers must be deactivated and activated at control mode changes, but it has two advantages:
 - minor performance increase: unused controllers are not active and therefore do not consume resources
 - unexpected behaviour is not possible: external control will not start on the robot, unless all necessary controllers are successfully activated, while control mode changes (on the robot) are only possible after the controllers for the new control mode are activated.

The consequence of the lifecycle interface is, that 3 commands are necessary to start external control for all robots:
 - start the appropriate launch file for your robot with your robot model as parameter (details can be found [here](#detailed-setup-and-startup-instructions))
 - `ros2 lifecycle set robot_manager configure`
 - `ros2 lifecycle set robot_manager activate`

#### Control mode definitions

The control mode specifications are also part of the common API. They are defined as an enum in the [`kuka_drivers_core`](https://github.com/kroshu/kuka_drivers/blob/master/kuka_drivers_core/include/kuka_drivers_core/control_mode.hpp) package, and have the following meaning:

- joint position control: the driver streams cyclic position updates for every joint. 
    - Needed command interface(s): `position`
- joint impedance control: the driver streams cyclic position updates for every joint and additionally stiffness [Nm/rad] and normalized damping [-] attributes, which define how the joint reacts to external effects (around the setpoint position). The effect of gravity is compensated internally.
    - Needed command interface(s): `position`, `stiffness`, `damping`
- joint velocity control: the driver streams cyclic velocity updates for every joint.
    - Needed command interface(s): `velocity`
- joint torque control: the driver streams cyclic torque updates for every joint, which define the torque overlay to be superimposed over gravity compensation. (An input of 0 means, that the joint should remain in gravity compensatin and should not move.)
    - Needed command interface(s): `effort`
- cartesian position control: the driver streams cyclic pose updates for every degree of freedom. The orientation representation is the KUKA ABC convention. It is the responsibility of the user to stream poses, for which a valid IK solution exists.
    - Needed command interface(s): `cart_position`
- cartesian impedance control: the driver streams cyclic pose updates for every degree of freedom. Additional stiffness [N/m or Nm/rad] and normalized damping [-] attributes define the behaviour of each degree of freedom to external forces. The nullspace stiffness and damping values define the behaviour of the redundant degree(s) of freedom.
    - Needed command interface(s): `cart_position`, `cart_stiffness`, `cart_damping`, (`nullspace_stiffness`, `nullspace_damping`)
- cartesian velocity control: the driver streams cyclic cartesian velocity (twist) updates for every degree of freedom. It is the responsibility of the user to stream velocities, for which a valid IK solution exists.
    - Needed command interface(s): `cart_velocity`
- wrench control: the driver streams cyclic wrench updates, which define the forces and torques, that the robot end effector should exert on the environment. The effect of gravity is internally compensated. (If the environment does not have a counterforce, the robot will start to move)
    - Needed command interface(s): `wrench`


#### Supported features

The following table shows the supported features and control modes of each driver. (`✓` means supported, `✗` means not supported by the KUKA interface, empty means supported by the KUKA interface, but not yet supported by the driver)

|OS | Joint position control | Joint impedance control | Joint velocity control | Joint torque control | Cartesian position control | Cartesian impedance control | Cartesian velocity control | Wrench control| I/O control|
|---|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|
|KSS| ✓ | ✗ | ✗ | ✗ | | ✗ | ✗ | ✗ | |
|Sunrise| ✓ | ✓ | ✗ | ✓ | | | ✗ | | |
|iiQKA| ✓ | ✓ | ✗ | ✓ | ✗ | ✗ | ✗ | ✗ | ✗ |


## Additional packages

The repository contains a few other packages aside from the 3 drivers:
- `kuka_driver_interfaces`: this package contains the custom message definition necessary for KUKA robots.
- `kuka_drivers_core`: this package contains core functionalities used by more drivers, including the `control_node`, base classes for nodes with improved parameter handling, enum and constant definitions and a class for managing the controller activation and deactivation at control mode changes. Details about these features can be found in the package [documentation](https://github.com/kroshu/kuka_drivers/blob/master/kuka_drivers_core/README.md)
- `kuka_rsi_simulator`: this package contains a simple simulator of RSI, that implements a UDP server accepting the same xml format as RSI and returning the commanded values as the current state, without any checks.
- `iiqka_moveit_example`: this package contains basic examples of using moveit with the driver, more information in the [next section](#moveit-integration). Additionally it contains a [launch file](../../examples/iiqka_moveit_example/launch/launch_trajectory_publisher.launch.py) that commands 4 goal positions near the home position cyclically (the points and parameters can be modified in [this](../../examples/iiqka_moveit_example/config/dummy_publisher.yaml) configuration file). This can be used to test moving any robot with the driver, and is the recommended way instead of the `rqt_joint_trajectory_controller`, which commands very jerky trajectories due to batching.

## Moveit integration

## Detailed setup and startup instructions

[Instructions for industrial robots using KSS](KSS_RSI.md)

[Instructions for cobots using Sunrise](Sunrise_FRI.md)

[Instructions for cobots using iiQKA](iiQKA_EAC.md)

## KUKA Sunrise driver (FRI)


## KUKA KSS driver (RSI)

Another project in the repo centers on the development of a ROS2 driver for KSS robots through Robot Sensor Interface (RSI). It is in an experimental state, with only joint angle states and commands available. The guide to set up this driver on a real robot can be found in kuka_kss_rsi_driver\krl for both KCR4 and KRC5 controllers.

### Simulation

To try out the driver with an open-loop simulation the driver and the kuka_rsi_simulator must be started, (at first only a "collapsed" robot will be visible in rviz):

**`ros2 launch kuka_kss_rsi_driver startup_with_rviz.launch.py`**

**`ros2 launch kuka_rsi_simulator kuka_rsi_simulator_launch.py`**

After all components have started successfully, the system needs to be configured and activated to start the simulation, the robot will be visible in rviz after activation:

**`ros2 lifecycle set robot_manager configure`**

**`ros2 lifecycle set robot_manager activate`**

## iiQKA driver (ECI)

The third project in the repo is the driver for iiQKA robots. 

### Architecture

The driver uses the ros2_control framework, so a variaty of controllers are supported and it can be easily integrated into moveit. It consists of a realtime component for controlling the robot via UDP (ROS2 hardware interface) and a non-realtime one for lifecycle management of the controllers and the hardware interface. The driver supports a control cycle time of 4 milliseconds, but the round-trip time of one cycle should not exceed 3 milliseconds, as above that the packets are considered lost. Therefore it is advised to run the driver on a realtime-capable Linux machine (with the PRREMPT_RT patch applied). After a few lost packets the connection is considered not stable enough and external control is ended.

The driver depends on some KUKA-specific packages, which are only available with the real robot, but setting the MOCK_HW_ONLY flag in the hardware_interface enables the usage of the driver in a simulated way, so that motion planning problems can be tried out with the same components running.

### Setup

By default, the mock libraries are used, this can be changed in the cmake file by setting MOCK_KUKA_LIBS to FALSE before building.

The IP addresses of the client machine and controller must be given in the *config/driver_config.yaml* configuration file. A rebuild is not needed after the changes, but the file has to be modified before starting the nodes. The control mode of the robot can also be modified in the same configuration file: you can choose either 1 (POSITION_CONTROL), 3 (JOINT_IMPEDANCE_CONTROL) or 5 (TORQUE_CONTROL). This also sets the control_mode parameter of the robot manager node, which can be only modified at startup, control mode changes are not supported in runtime at the current state.

Besides, the setting of scheduling priorities must be allowed for your user (extend /etc/security/limits.conf with "username	 -	 rtprio		 98" and restart) to enable real-time performance.

### Usage

The usage of the driver is quite simple, one has to start the launch file in the package to start all required nodes (it also starts an rviz node for visualisation, which can be commented out if not needed):

**`ros2 launch kuka_iiqka_eac_driver startup.launch.py`**

After all components have started successfully, the system needs to be configured and activated to start external control:

**`ros2 lifecycle set robot_manager configure`**


**`ros2 lifecycle set robot_manager activate`**

On successful activation the robot controller and the driver start communication with a 4 ms cycle time, and it is possible to move the robot through the joint trajectory controller. The easiest way to achieve this is to start an rqt_joint_trajcectory controller and move the joints with cursors or one can also execute trajectories planned with moveit - an example of this can be found in kuka_sunrise_fri_driver_control/iiqka_moveit_example package.

To stop external control, the components have to be deactivated with **`ros2 lifecycle set robot_manager deactivate`**

BEWARE, that this is a non-realtime process including lifecycle management, so the connection is not terminated immediately, in cases where an abrupt stop is needed, the safety stop of the SmartPad should be used!

It is also possible to use different controllers with some modifications in the launch and yaml files (for example ForwardCommandController, which forwards the commands send to a ROS2 topic towards the robot). In these cases, one has to make sure, that the commands sent to the robot are close to the current position, otherwise the machine protection will stop the robot movement.




