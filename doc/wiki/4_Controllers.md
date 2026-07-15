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

The `FRIStateBroadcaster` publishes the actual state of FRI to the `~/fri_state` topic, using the custom [FRIStateArray](https://github.com/kroshu/kuka_drivers/blob/master/kuka_driver_interfaces/msg/FRIStateArray.msg) message, which contains an array of [FRIState](https://github.com/kroshu/kuka_drivers/blob/master/kuka_driver_interfaces/msg/FRIState.msg) messages.

__Required Parameters__: None

__Optional Parameters__:

- `robot_prefixes` (`string[]`, default `['']`):
  - Empty string entry (`''`) maps to unprefixed interfaces for single-robot compatibility (`fri_state/session_state`, etc.).
  - Non-empty entries map to prefixed interfaces (`<robot_prefix>_fri_state/session_state`, etc.).
  - One publisher instance broadcasts the state of all robots as a single `FRIStateArray` message.

### 2.2. `kuka_event_broadcaster`

The `EventBroadcaster` publishes server state change events as a map-like message on
`~/hardware_event` using `kuka_driver_interfaces::msg::HardwareEvent`.

- Single robot (default): uses `state/server_state` interface and publishes one message per event.
- Multi robot (with `robot_prefixes`): uses state interfaces `<robot_prefix>_state/server_state` and publishes one message per changed robot event.

The enum values are equivalent with the following events:

- 2: Start command was accepted by the robot controller
- 3: External control started
- 4: External control stopped (by user)
- 5: Control mode switch was successful (only relevant for drivers, where control mode can be changed in active state)
- 6: External control stopped by an error (Error message is only available in the hardware interface)

__Required Parameters__: None

__Optional Parameters__:

- `robot_prefixes` (`string[]`, default `['']`):
  - Empty string entry (`''`) maps to the unprefixed `state/server_state` interface.
  - Non-empty entries map to prefixed interfaces (`<robot_prefix>_state/server_state`).

## 3. Configuration Controllers

Hardware interfaces do not support parameters that can be changed in runtime. To provide this behaviour, configuration controllers can be used, which update specific command interfaces of a hardware, that are exported as a workaround instead of parameters.

### 3.1. `kuka_control_mode_handler`

The `ControlModeHandler` can update the `control_mode` command interface of a hardware. It listens on the `~/control_mode` topic and makes control mode changes possible without having to reactivate the driver.
The control mode is [defined as an enum](https://github.com/kroshu/kuka_drivers/blob/master/kuka_drivers_core/include/kuka_drivers_core/control_mode.hpp) in the `kuka_drivers_core` package, the subscription therefore is of an unsigned integer type.

In multi-robot mode, one `ControlModeHandler` instance can update multiple prefixed
`control_mode` command interfaces, but it still accepts only a single shared `~/control_mode`
input topic. Therefore the same control mode is applied to all configured robots.

__Required Parameters__: None

__Optional Parameters__:

- `robot_prefixes` (`string[]`, default `['']`):
  - Empty string entry (`''`) maps to the unprefixed `runtime_config/control_mode` interface.
  - Non-empty entries map to prefixed interfaces (`<robot_prefix>_runtime_config/control_mode`).

### 3.2. `fri_configuration_controller`

The `SendPeriodMilliSec` parameter of FRI defines the period with which the controller sends state updates, while the `ReceiveMultiplier` defines the answer rate factor (ratio of receiving states and sending commands). These are parameters of the hardware interface, which can be modified in connected state, when control is not active. To support changing these parameters after startup, the `FRIConfigurationController` subscribes to the `~/set_fri_config` topic. Sending a message containing the desired integer values of `send_period_ms` (cycle time) and `receive_multiplier` updates the parameters of the hardware interface.

In multi-robot mode, one `FRIConfigurationController` instance can update multiple prefixed command interfaces, but it still accepts only a single shared `~/set_fri_config` input topic. Therefore the same FRI configuration is applied to all configured robots.

__Required Parameters__: None

__Optional Parameters__:

- `robot_prefixes` (`string[]`, default `['']`):
  - Empty string entry (`''`) maps to unprefixed command interfaces (`runtime_config/receive_multiplier`, `runtime_config/send_period`).
  - Non-empty entries map to prefixed interfaces (`<robot_prefix>_runtime_config/receive_multiplier`, `<robot_prefix>_runtime_config/send_period`).

## 4. Hybrid Controllers

Hybrid Controllers group together related functionalities that are intended to be used in combination. Currently, there is a single Hybrid Controller that handles non-real-time messages. It effectively combines the roles of a Broadcaster and a Configuration Controller.

### 4.1. `kuka_kss_message_handler`

The `kuka_kss_message_handler` controller only works for the EKI + RSI driver. It provides two non-real-time capabilities:


- __Set RSI Cycle Time__

  Publish a `std_msgs::msg::UInt8` message to `~/cycle_time` to set the cycle time:
  - 1 &rarr; 4 ms
  - 2 &rarr; 12 ms

  In multi-robot mode, this topic is shared across all configured robots, therefore the selected
  cycle time is applied to all of them.

  ```shell
  ros2 topic pub /kss_message_handler/cycle_time std_msgs/msg/UInt8 "{data: 1}" --once
  ```

- __Monitor Robot Status__

  Subscribe to `~/status` to receive updates via
  `kuka_driver_interfaces::msg::KssStatusArray` (`robot_names[]` + `statuses[]`).
  The `statuses` entries are `kuka_driver_interfaces::msg::KssStatus` values and each index
  corresponds to the same index in `robot_names`.

  Status messages are published at 1 Hz regardless of whether values changed.

  Each `kuka_driver_interfaces::msg::KssStatus` includes:

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

__Optional Parameters__:

- `robot_prefixes` (`string[]`, default `['']`):
  - Empty string entry (`''`) maps to unprefixed state interfaces (`state/control_mode`, `state/cycle_time`, etc.) and unprefixed command interface (`runtime_config/cycle_time`).
  - Non-empty entries map to prefixed interfaces (`<robot_prefix>_state/control_mode`, `<robot_prefix>_runtime_config/cycle_time`, etc.).
  - The `~/cycle_time` topic is shared across all configured robots, allowing centralized control of all robot instances.
