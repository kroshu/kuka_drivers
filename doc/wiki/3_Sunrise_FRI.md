## Sunrise driver (FRI)

### Setup

#### Client side
- It is recommended to use the driver on a real-time capable client machine (further information about setting up the PREEMPT_RT patch can be found [here](https://github.com/kroshu/kuka_drivers/wiki/5_Realtime)).
- Set a fixed IP in the subnet of the controller for the real-time machine.

#### Controller side

- Upload the robot application under `robot_application/src` to the controller using Sunrise Workbench

### Configuration

#### Startup configuration

The following configuration files are available in the `config` directory of the package:
- `driver_config.yaml`: : contains runtime parameters of the `robot_manager` node
- `ros2_controller_config.yaml`: contains the controller types for every controller name. Should be only modified if a different controller is to be used.
- configuration files for specific controllers, for further information, see the documentation of the given controller
- `gpio_config.xacro`: contains the I/O setup of the system, but this was not tested yet

##### IP configuration
The IP address of robot controller must be provided as a launch argument. For further information see section [launch arguments](#launch-arguments).

#### Runtime parameters
The parameters in the driver configuration file can be also changed during runtime using the parameter interface of the `robot_manager` node:
- `send_period_ms` (integer): this parameter defines the send rate in milliseconds (with which the controller sends robot state updates). It must be between 1 and 10 for control and can be only changed in `inactive` and `configuring` states.
- `receive_multiplier` (integer): this parameter defines the answer rate factor (the client should sends commands in every `receive_multiplier`*`send_period_ms` milliseconds). It must be at least 1 and can be only changed in `inactive` and `configuring` states.
- `control_mode`: The enum value of the control mode should be given, which updates the `ControlMode` and `ClientCommandMode` parameters of FRI. It cannot be changed in active state.
- `joint_damping`, `joint_stiffness` (double vectors): these parameters change the stiffness and damping attributes of joint impedance control mode. The updated values are sent to the hardware interface using the `joint_group_impedance_controller` to adapt to conventions, but it is not possible to change them in active state due to the constraints of FRI. (Therefore the `joint_group_impedance_controller` is deactivated at driver activation.)
- `position_controller_name`: The name of the controller (string) that controls the `position` interface of the robot. It can't be changed in active state.
- `torque_controller_name`: The name of the controller (string) that controls the `effort` interface of the robot. It can't be changed in active state.

### Usage

#### Starting the driver

1. On the controller, start the uploaded robot application (ROS2_Control).
2. To start the driver, two launch file are available, with and without `rviz`. To launch (without `rviz`), run
```
ros2 launch kuka_sunrise_fri_driver startup.launch.py controller_ip:=0.0.0.0 client_ip:=0.0.0.0
```
This starts the 3 core components of every driver (described in the [Non-real-time interface](https://github.com/kroshu/kuka_drivers/wiki#non-real-time-interface) section of the project overview) and the following controllers:
- `joint_state_broadcaster` (no configuration file, all state interfaces are published)
- `joint_trajectory_controller` ([configuration file](https://github.com/kroshu/kuka_drivers/tree/master/kuka_sunrise_fri_driver/config/joint_trajectory_controller_config.yaml))
- [`fri_configuration_controller`](https://github.com/kroshu/kuka_controllers?tab=readme-ov-file#fri_configuration_controller) (no configuration file)
- [`fri_state_broadcaster`](https://github.com/kroshu/kuka_controllers?tab=readme-ov-file#fri_state_broadcaster) (no configuration file)
- `joint_group_impedance_controller` ([configuration file](https://github.com/kroshu/kuka_drivers/tree/master/kuka_sunrise_fri_driver/config/joint_impedance_controller_config.yaml))
- `effort_controller` (of type `JointGroupEffortController`, [configuration file](https://github.com/kroshu/kuka_drivers/tree/master/kuka_sunrise_fri_driver/config/effort_controller_config.yaml))
- [`control_mode_handler`](https://github.com/kroshu/kuka_controllers?tab=readme-ov-file#control_mode_handler) (no configuration file)

3. After successful startup, the `robot_manager` node has to be activated to start the cyclic communication with the robot controller (before this only a collapsed robot is visible in `rviz`):
    ```
    ros2 lifecycle set robot_manager configure
    ros2 lifecycle set robot_manager activate
    ```
On successful activation the brakes of the robot will be released and external control is started using the requested control mode. To test moving the robot, the `rqt_joint_trajectory_controller` is not recommended, use the launch file in the `iiqka_moveit_example` package instead (usage is described in the [Additional packages](https://github.com/kroshu/kuka_drivers/wiki#additional-packages) section of the project overview).


##### Launch arguments

Both launch files support the following argument:
- `controller_ip`: IP address of the robot controller
- `client_ip`: IP address of the client PC
- `client_port`: port of the client machine (default: 30200)
- `robot_model`: defines which LBR iiwa robot to use. Available options: `lbr_iiwa14_r820` (default)
- `use_fake_hardware`: if true, the `KukaMockHardwareInterface` will be used instead of the `KukaFRIHardwareInterface`. This enables trying out the driver without actual hardware.
- `namespace`: adds a namespace to all nodes and controllers of the driver, and modifies the `prefix` argument of the robot description macro to `namespace_`
- `x`, `y`, `z`: define the position of `base_link` relative to the `world` frame in meters (default: [0, 0, 0])
- `roll`, `pitch`, `yaw`: define the orientation of `base_link` relative to the `world` frame in radians (default: [0, 0, 0])
- `roundtrip_time`: The roundtrip time (in microseconds) to be enforced by the [KUKA mock hardware interface](https://github.com/kroshu/kuka_robot_descriptions?tab=readme-ov-file#custom-mock-hardware), (defaults to 5000 us, only used if `use_fake_hardware` is true)
- `controller_config`: the location of the `ros2_control` configuration file (defaults to `kuka_sunrise_fri_driver/config/ros2_controller_config.yaml`)
- `jtc_config`: the location of the configuration file for the `joint_trajectory_controller` (defaults to `kuka_sunrise_fri_driver/config/joint_trajectory_controller_config.yaml`)
- `jic_config`: the location of the configuration file for the `joint_impedance_controller` (defaults to `kuka_sunrise_fri_driver/config/joint_impedance_controller_config.yaml`)
- `ec_config`: the location of the configuration file for the `effort_controller` (defaults to `kuka_sunrise_fri_driver/config/effort_controller_config.yaml`)


The `startup_with_rviz.launch.py` additionally contains one argument:
- `rviz_config`: the location of the `rviz` configuration file (defaults to `kuka_resources/config/view_6_axis_urdf.rviz`)


#### Stopping external control

To stop external control, all components have to be deactivated with `ros2 lifecycle set robot_manager deactivate`

BEWARE, that this is a non-realtime process including lifecycle management, so the connection is not terminated immediately, in cases where an abrupt stop is needed, the safety stop of the Teach Pendant should be used! (This will also deactivate all components to allow reactivation without a restart.)


### Known issues and limitations

- I/O control was not tested
- Cartesian modes are not yet supported
