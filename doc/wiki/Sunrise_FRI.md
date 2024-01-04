## Sunrise driver (FRI)

### Setup

#### Client side
- It is recommended to use the driver on a real-time capable client machine (further information about setting up the PREEMPT_RT patch can be found [here](Realtime.md)).
- Set a fixed IP in the subnet of the controller for the real-time machine.

#### Controller side

- Upload the robot application under `robot_application/src` to the controller using the Sunrise Workbench

### Configuration

#### Startup configuration

The following configuration files are available in the `config` directory of the package:
- `driver_config.yaml`: contains IP addresses and runtime parameters
- `ros2_controller_config.yaml`: contains the controller types for every controller name. Should be only modified if a different controller is to be used. The `configure_components_on_start` parameter should never be modified, which ensures that the hardware interface is not activated at startup.
- configuration files for specific controllers, for further information, see the documentation of the given controller
- `gpio_config.xacro`: contains the I/O setup of the system, but this was not tested yet

##### IP configuration
The following parameter must be set in the driver configuration file:
- `controller_ip`: IP address of the controller

#### Runtime parameters
The parameters in the driver configuration file can be also changed during runtime using the parameter interface of the `robot_manager` node:
- `send_period_ms` (integer): this parameter defines the send rate in milliseconds (with which the controller sends robot state updates). It must be between 1 and 10 for control and can be only changed in `inactive` and `configuring` states.
- `receive_multiplier` (integer): this parameter defines the answer rate factor (the client should sends commands in every `receive_multiplier`*`send_period_ms` milliseconds). It must be at least 1 and can be only changed in `inactive` and `configuring` states.
- `control_mode`, `command_mode` (strings): control mode related parameters, which will be combined to support the defined enums. They cannot be changed in active state.
- `joint_damping`, `joint_stiffness` (double vectors): these parameters change the stiffness and damping attributes of joint impedance control mode. They will be removed after changing to using the `joint_group_impedance_controller` to adapt to conventions.

### Usage

#### Starting the driver

1. On the controller, start the uploaded robot application (ROS2_Control).
2. To start the driver, two launch file are available, with and without `rviz`. To launch (without `rviz`), run 
    ```
    ros2 launch kuka_iiqka_eac_driver startup.launch.py`
    ```
    - This starts the 3 core components of every driver (described in the *Non-real-time interface* section of the [project overview](Project%20overview.md)) and the following controllers:
      - `joint_state_broadcaster` (no configuration file, all state interfaces are published)
      - `joint_trajectory_controller` ([configuration file](../../kuka_iiqka_eac_driver/config/joint_trajectory_controller_config.yaml))
      - [`fri_configuration_controller`](https://github.com/kroshu/kuka_controllers?tab=readme-ov-file#fri_configuration_controller) (no configuration file)
      - [`fri_state_broadcaster`](https://github.com/kroshu/kuka_controllers?tab=readme-ov-file#fri_state_broadcaster) (no configuration file)

3. After successful startup, the `robot_manager` node has to be activated to start the cyclic communication with the robot controller (before this only a collapsed robot is visible in `rviz`):
    ```
    ros2 lifecycle set robot_manager configure
    ros2 lifecycle set robot_manager activate
    ```
On successful activation the brakes of the robot will be released and external control is started using the requested control mode. To test moving the robot, the `rqt_joint_trajectory_controller` is not recommended, use the launch file in the `iiqka_moveit_example` package instead (usage is described in the *Additional packages* section of the [project overview](Project%20overview.md)).


##### Launch arguments

Both launch files support the following argument:
- `robot_model`: defines which LBR iiwa robot to use. Available options: `lbr_iiwa14_r820` (default)
- `use_fake_hardware`: if true, the `mock_components/GenericSystem` will be used instead of the `KukaRSIHardwareInterface`. This enables trying out the driver without actual hardware.

#### Stopping external control

To stop external control, all components have to be deactivated with `ros2 lifecycle set robot_manager deactivate`

BEWARE, that this is a non-realtime process including lifecycle management, so the connection is not terminated immediately, in cases where an abrupt stop is needed, the safety stop of the Teach Pendant should be used! (This will also deactivate all components to allow reactivation without a restart.)


### Known issues and limitations

- Not all hardware-related communication is implemented in the hardware interface, therefore the mock hardware option is not working properly
- The control mode handling for the driver is not the one defined in the `kuka_drivers_core` package
  - enum definition and controller switching logic is not used
  - joint impedance control is not implemented properly using command interfaces
- I/O control was not tested at all
- Cartesian modes are not yet supported