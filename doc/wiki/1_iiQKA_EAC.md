## iiQKA driver (EAC)

### Setup

#### Client side
- It is recommended to use the driver on a real-time capable client machine (further information about setting up the PREEMPT_RT patch can be found [here](https://github.com/kroshu/kuka_drivers/wiki/5_Realtime)).
- Set a fixed IP in the subnet of the KONI interface for the real-time machine.

#### Controller side

- Install ExternalAPI.Control toolbox (requires iiQKA 1.2)
  - The toolbox is not available by default and should be requested from KUKA Customer Support
- Set KONI IP (System Settings -> Network -> KONI) + restart
- Connect external client to KONI port (XF7)
- Release SPOC on the Teach Pendant

### Configuration

#### Configuration files

The following configuration files are available in the `config` directory of the package:
- `driver_config.yaml`: contains runtime parameters of the `robot_manager` node
- `qos_config.yaml`: contains the configuration options for the QoS profile defining the connection quality (description later)
- `ros2_controller_config.yaml`: contains the controller types for every controller name. Should be only modified if a different controller is to be used.
- configuration files for specific controllers (for further information, see the documentation of the given controller)

##### QoS profile configuration
It is possible to configure the required connection quality of external control using a QoS profile, which defines under which conditions should external control be terminated in case of packet losses. A packet is considered lost if the controller does not receive a command in 2.5 milliseconds after publishing the state. In case of a packet loss, extrapolation is used to optimize the robot behaviour, unless the limits defined in the profile are violated. Two limits can be defined:
- Consequent packet losses allowed, defined by `consequent_lost_packets` parameter, maximum value is 5
- Allowed packet losses in given timeframe: defined by `lost_packets_in_timeframe` (maximum value is 25) and `timeframe_ms` parameters, maximum ratio is 25/sec

#### Runtime parameters
The parameters in the driver configuration file can be also changed during runtime using the parameter interface of the `robot_manager` node:
- `control_mode`: The enum value of the control mode should be given. It can be changed in all primary states, but in active state the brakes are always closed before control is started in a new mode.
- `position_controller_name`: The name of the controller (string) that controls the `position` interface of the robot. It can't be changed in active state.
- `impedance_controller_name`: The name of the controller (string) that controls the `stiffness` and `damping` interfaces of the robot. It can't be changed in active state.
- `torque_controller_name`: The name of the controller (string) that controls the `effort` interface of the robot. It can't be changed in active state.

#### IP Configuration
The IP address of the client machine and robot controller must be provided as a launch argument. For further information see section [launch arguments](#launch-arguments).

### Usage

#### Starting the driver

To start the driver, two launch file are available, with and without `rviz`. To launch (without `rviz`), run:

```
ros2 launch kuka_iiqka_eac_driver startup.launch.py client_ip:=0.0.0.0 controller_ip:=0.0.0.0
```

This starts the 3 core components of every driver (described in the [Non-real-time interface](https://github.com/kroshu/kuka_drivers/wiki#non-real-time-interface) section of the project overview) and the following controllers:
- `joint_state_broadcaster` (no configuration file, all state interfaces are published)
- `joint_trajectory_controller` ([configuration file](https://github.com/kroshu/kuka_drivers/tree/master/kuka_iiqka_eac_driver/config/joint_trajectory_controller_config.yaml))
- `joint_group_impedance_controller` ([configuration file](https://github.com/kroshu/kuka_drivers/tree/master/kuka_iiqka_eac_driver/config/joint_impedance_controller_config.yaml))
- `effort_controller` (of type `JointGroupPositionController`, [configuration file](https://github.com/kroshu/kuka_drivers/tree/master/kuka_iiqka_eac_driver/config/effort_controller_config.yaml))
- [`kuka_control_mode_handler`](https://github.com/kroshu/kuka_drivers/wiki/4_Controllers#kuka_control_mode_handler) (no configuration file)

After successful startup, the `robot_manager` node has to be activated to start the cyclic communication with the robot controller (before this only a collapsed robot is visible in `rviz`):
  ```
  ros2 lifecycle set robot_manager configure
  ros2 lifecycle set robot_manager activate
  ```

On successful activation the brakes of the robot will be released and external control is started using the requested control mode. To test moving the robot, the `rqt_joint_trajectory_controller` is not recommended, use the launch file in the `iiqka_moveit_example` package instead (usage is described in the [Additional packages](https://github.com/kroshu/kuka_drivers/wiki#additional-packages) section of the project overview).

It is important to note, that the commanded and measures torques have a different meaning: the `effort` command interface accepts values that should be superimposed on internal gravity compensation, while the state interface provides the actually measured torques (sum on internal and external effects).


##### Launch arguments

Both launch files support the following arguments:
- `client_ip`: IP address of the client machine
- `controller_ip`: KONI IP of the controller
- `robot_model`: defines which LBR iisy robot to use. Available options: `lbr_iisy3_r760` (default), `lbr_iisy11_r1300`, `lbr_iisy15_r930`
- `use_fake_hardware`: if true, the `KukaMockHardwareInterface` will be used instead of the `KukaEACHardwareInterface`. This enables trying out the driver without actual hardware.
- `namespace`: adds a namespace to all nodes and controllers of the driver, and modifies the `prefix` argument of the robot description macro to `namespace_`
- `x`, `y`, `z`: define the position of `base_link` relative to the `world` frame in meters (default: [0, 0, 0])
- `roll`, `pitch`, `yaw`: define the orientation of `base_link` relative to the `world` frame in radians (default: [0, 0, 0])
- `roundtrip_time`: The roundtrip time (in microseconds) to be enforced by the [KUKA mock hardware interface](https://github.com/kroshu/kuka_robot_descriptions?tab=readme-ov-file#custom-mock-hardware), (defaults to 2500 us, only used if `use_fake_hardware` is true)
- `qos_config`: the location of the QoS configuration file (defaults to `kuka_iiqka_eac_driver/config/qos_config.yaml`)
- `controller_config`: the location of the `ros2_control` configuration file (defaults to `kuka_iiqka_eac_driver/config/ros2_controller_config.yaml`)
- `jtc_config`: the location of the configuration file for the `joint_trajectory_controller` (defaults to `kuka_iiqka_eac_driver/config/joint_trajectory_controller_config.yaml`)
- `jic_config`: the location of the configuration file for the `joint_impedance_controller` (defaults to `kuka_iiqka_eac_driver/config/joint_impedance_controller_config.yaml`)
- `ec_config`: the location of the configuration file for the `effort_controller` (defaults to `kuka_iiqka_eac_driver/config/effort_controller_config.yaml`)


The `startup_with_rviz.launch.py` additionally contains one argument:
- `rviz_config`: the location of the `rviz` configuration file (defaults to `kuka_resources/config/view_6_axis_urdf.rviz`)

#### Stopping external control

To stop external control, all components have to be deactivated with `ros2 lifecycle set robot_manager deactivate`

BEWARE, that this is a non-realtime process including lifecycle management, so the connection is not terminated immediately, in cases where an abrupt stop is needed, the safety stop of the Teach Pendant should be used! (This will also deactivate all components to allow reactivation without a restart.)
