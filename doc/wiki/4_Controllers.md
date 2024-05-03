## KUKA-specific controllers

The controllers needed for KUKA drivers can be divided into three categories.

### Traditional controllers

These controllers update the command interfaces of a hardware cyclically.

#### `joint_group_impedance_controller`
The joint impedance controller listens on the `~/command` topic and updates the `stiffness` and `damping` interfaces of the hardware accordingly.
The command must be of `std_msgs::Float64MultiArray` type and must contain the values for all configured joints. The order of the values should match the `stifness_1, damping_1, stiffness_2, ...` pattern. The controller only processes the commands received on this topic, when it is in active state.

Example cli command to set damping to 0.7 and stiffness to 100 for all joints of a 6 DOF robot:

```
ros2 topic pub /joint_group_impedance_controller/commands std_msgs/msg/Float64MultiArray "{data: [100, 0.7, 100, 0.7, 100, 0.7, 100, 0.7, 100, 0.7, 100, 0.7]}" --once
```

__Required parameters__:
- `joints` [string_array]: Names of joints used by the controller

### Broadcasters

Broadcasters receive the state interfaces of a hardware and publish it to a ROS2 topic.
#### `fri_state_broadcaster`

The `FRIStateBroadcaster` publishes the actual state of FRI to the `~/fri_state` topic, using the custom [FRIState](https://github.com/kroshu/kuka_drivers/blob/master/kuka_driver_interfaces/msg/FRIState.msg) message.

__Required parameters__: None


#### `kuka_event_broadcaster`

The `EventBroadcaster` publishes server state change events as integers (enum values) on the `~/hardware_event` topic. The enum values are equivalent with the following events:
- 2: Start command was accepted by the robot controller
- 3: External control started
- 4: External control stopped (by user)
- 5: Control mode switch was successful (only relevant for drivers, where control mode can be changed in active state)
- 6: External control stopped by an error (Error message is only available in the hardware interface)

__Required parameters__: None

### Configuration controllers

Hardware interfaces do not support parameters that can be changed in runtime. To provide this behaviour, configuration controllers can be used, which update specific command interfaces of a hardware, that are exported as a workaround instead of parameters.

#### `kuka_control_mode_handler`

The `ControlModeHandler` can update the `control_mode` command interface of a hardware. It listens on the `~/control_mode` topic and makes control mode changes possible without having to reactivate the driver.
The control mode is [defined as an enum](https://github.com/kroshu/kuka_drivers/blob/master/kuka_drivers_core/include/kuka_drivers_core/control_mode.hpp) in the `kuka_drivers_core` package, the subscription therefore is of an unsigned integer type.

__Required parameters__: None

#### `fri_configuration_controller`

The `SendPeriodMilliSec` parameter of FRI defines the period with which the controller sends state updates, while the `ReceiveMultiplier` defines the answer rate factor (ratio of receiving states and sending commands). These are parameters of the hardware interface, which can be modified in connected state, when control is not active. To support changing these parameters after startup, the `FRIConfigurationController` advertises the topic `~/set_fri_config`. Sending a message containing the desired integer values of `send_period_ms` and `receive_multiplier` updates the parameters of the hardware interface.

__Required parameters__: None
