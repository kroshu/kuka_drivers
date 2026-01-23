# KUKA-specific controllers

To simplify the use of KUKA drivers, a set of custom ROS 2 controllers has been developed. These controllers are launched using the provided ROS 2 launch files. They follow the ROS 2 lifecycle system, and their states and transitions are handled by special robot manager nodes designed for each KUKA robot operating system. Currently, four types of controllers are available:

1. Traditional Controllers
2. Broadcasters
3. Configuration Controllers
4. Hybrid Controllers

## 1. Traditional Controllers

These controllers update the command interfaces of a hardware cyclically.

### 1.1. `joint_group_impedance_controller`

The joint impedance controller listens on the `~/command` topic and updates the `stiffness` and `damping` interfaces of the hardware accordingly.
The command must be of `std_msgs::Float64MultiArray` type and must contain the values for all configured joints. The order of the values should match the `stiffness_1, damping_1, stiffness_2, ...` pattern. The controller only processes the commands received on this topic, when it is in active state.

Example cli command to set damping to 0.7 and stiffness to 100 for all joints of a 6 DOF robot:

```shell
ros2 topic pub /joint_group_impedance_controller/commands std_msgs/msg/Float64MultiArray "{data: [100, 0.7, 100, 0.7, 100, 0.7, 100, 0.7, 100, 0.7, 100, 0.7]}" --once
```

The controller also publishes the position values commanded at the joint position interface to the `~/commanded_positions` topic, which can be used instead of the measured joint positions to allow seamless execution of consequent trajectories in joint impedance mode.

__Required Parameters__:

- `joints` [string_array]: Names of joints used by the controller

## 2. Broadcasters

Broadcasters receive the state interfaces of a hardware and publish it to a ROS2 topic.

### 2.1. `fri_state_broadcaster`

The `FRIStateBroadcaster` publishes the actual state of FRI to the `~/fri_state` topic, using the custom [FRIState](https://github.com/kroshu/kuka_drivers/blob/master/kuka_driver_interfaces/msg/FRIState.msg) message.

__Required Parameters__: None

### 2.2. `kuka_event_broadcaster`

The `EventBroadcaster` publishes server state change events as integers (enum values) on the `~/hardware_event` topic. The enum values are equivalent with the following events:

- 2: Start command was accepted by the robot controller
- 3: External control started
- 4: External control stopped (by user)
- 5: Control mode switch was successful (only relevant for drivers, where control mode can be changed in active state)
- 6: External control stopped by an error (Error message is only available in the hardware interface)

__Required Parameters__: None

## 3. Configuration Controllers

Hardware interfaces do not support parameters that can be changed in runtime. To provide this behaviour, configuration controllers can be used, which update specific command interfaces of a hardware, that are exported as a workaround instead of parameters.

### 3.1. `kuka_control_mode_handler`

The `ControlModeHandler` can update the `control_mode` command interface of a hardware. It listens on the `~/control_mode` topic and makes control mode changes possible without having to reactivate the driver.
The control mode is [defined as an enum](https://github.com/kroshu/kuka_drivers/blob/master/kuka_drivers_core/include/kuka_drivers_core/control_mode.hpp) in the `kuka_drivers_core` package, the subscription therefore is of an unsigned integer type.

__Required Parameters__: None

### 3.2. `fri_configuration_controller`

The `SendPeriodMilliSec` parameter of FRI defines the period with which the controller sends state updates, while the `ReceiveMultiplier` defines the answer rate factor (ratio of receiving states and sending commands). These are parameters of the hardware interface, which can be modified in connected state, when control is not active. To support changing these parameters after startup, the `FRIConfigurationController` advertises the topic `~/set_fri_config`. Sending a message containing the desired integer values of `send_period_ms` and `receive_multiplier` updates the parameters of the hardware interface.

__Required Parameters__: None

## 4. Hybrid Controllers

Hybrid Controllers group together related functionalities that are intended to be used in combination. Currently, there is a single Hybrid Controller that handles non-real-time messages. It effectively combines the roles of a Broadcaster and a Configuration Controller.

### 4.1. `kuka_kss_message_handler`

The `kuka_kss_message_handler` controller only works for the EKI + RSI driver. It provides several non-real-time capabilities:

- __Toggle Robot Drives__

  Publish a `std_msgs::msg::Bool` message to `~/drive_state` to control the drive state:

  ```shell
  ros2 topic pub /kss_message_handler/drive_state std_msgs/msg/Bool "{data: false}" --once
  ```

- __Set RSI Cycle Time__

  Publish a `std_msgs::msg::UInt8` message to `~/cycle_time` to set the cycle time:
  - 1 &rarr; 4 ms
  - 2 &rarr; 12 ms

  ```shell
  ros2 topic pub /kss_message_handler/cycle_time std_msgs/msg/UInt8 "{data: 1}" --once
  ```

- __Monitor Robot Status__

  Subscribe to `~/status` to receive updates via `kuka_driver_interfaces::msg::KssStatus`, which includes:

  Field             | Possible values                          | Description
  ------------------|------------------------------------------|--------------------------------------------------------------------
  `control_mode`    | `1` (joint position)                     | Specifies the robot's control mode
  `cycle_time`      | `1` (4 ms), `2` (12 ms)                  | Defines the selected RSI communication cycle time
  `drives_powered`  | `true`/`false`                           | Shows whether the robot's drives are powered
  `emergency_stop`  | `true`/`false`                           | Reflects whether the emergency stop is active
  `guard_stop`      | `true`/`false`                           | Indicates whether the guard stop is active
  `in_motion`       | `true`/`false`                           | Shows whether the robot is moving
  `motion_possible` | `true`/`false`                           | Indicates whether motion is possible
  `operation_mode`  | `1` (T1), `2` (T2), `3` (AUT), `4` (EXT) | Represents the robot's current operation mode
  `robot_stopped`   | `true`/`false`                           | Signals whether the robot stopped

  ```shell
  ros2 topic echo /kss_message_handler/status
  ```

__Note:__ These features are available only when the driver is in the __configured__ state. However, status updates are still published in the __active__ state. These updates are only sent if the EKI driver has an idle cycle, meaning no other messages are being transmitted at that moment; this applies to both the configured and the active states.

__Required Parameters__: None
