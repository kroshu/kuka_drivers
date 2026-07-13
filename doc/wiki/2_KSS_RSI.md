# KSS and iiQKA.OS2 drivers (RSI)

This guide provides instructions for setting up and using the RSI-based ROS 2 driver for KUKA robots running on **KUKA System Software (KSS)** and **iiQKA.OS2**.

The driver supports three configurations on both KSS and iiQKA.OS2:

- `rsi_only`: Uses only the RSI channel.
- `eki_rsi`: Uses EKI for non-real-time startup/status handling and RSI for cyclic control.
- `mxa_rsi`: Uses mxAutomation for non-real-time startup/status handling and RSI for cyclic control.

The integration of EKI and mxA not only helps the initiation of external control but also unlocks additional capabilities via ROS 2 controllers.

It is recommended to run the driver on a real-time capable client machine. Detailed instructions for setting up the `PREEMPT_RT` path are available on the [Realtime](https://github.com/kroshu/kuka_drivers/wiki/5_Realtime) wiki page.

## Test setups

### KSS tested configurations

| Controller | Robot                | KSS Version | EthernetKRL Version | RSI Version |
|------------|----------------------|-------------|---------------------|-------------|
| KR C4 OPS  | &ndash;              | 8.6.11      | 3.1.4               | 4.1.3       |
| KR C5 OPS  | &ndash;              | 8.7.5       | 3.2.5               | 5.0.2       |
| KR C5      | KR 120 R2700-2 Dummy | 8.7.5       | 3.2.5               | 5.0.2       |
| KR C5      | KR 6 R900-2          | 8.7.5       | 3.2.5               | 5.0.2       |

### iiQKA.OS2 tested configurations

| Controller | Robot         | iiQKA.OS2 Version | RSI Version |
|------------|---------------|-------------------|-------------|
| KR C5 OPS  | &ndash;       | 9.1.0             | 6.1.2       |
| KR C5      | KR 16 R1610-2 | 9.1.0             | 6.1.2       |


## Client-side setup

It is recommended to run the driver on a real-time capable client machine. Detailed instructions for setting up the `PREEMPT_RT` path are available on the [Realtime](https://github.com/kroshu/kuka_drivers/wiki/6_Realtime) wiki page.

To be able to connect to RSI running on the controller, a fixed IP in the subnet of the RSI interface is required on the Linux machine.

## Controller-side setup (OS-specific)

To set up the controller with WorkVisual/iiQWorks.Sim, a Windows machine is also required with a fixed IP in the subnet of the KLI interface for transferring the project.

### KSS setup

Use the SDK setup guide for all KSS controller-side steps (network, file deployment, wrappers):

- [External Control Setup for KSS](https://github.com/kroshu/kuka-external-control-sdk/blob/master/kuka_external_control_sdk/doc/kss_setup.md)

Setups for all three versions (`rsi_only`, `eki_rsi`, `mxa_rsi`) are available in this file.

### iiQKA.OS2 setup (RSI 6.x)

Use the SDK setup guide for all iiQKA.OS2 controller-side steps (network, file deployment, wrappers):

- [External Control Setup for iiQKA.OS2](https://github.com/kroshu/kuka-external-control-sdk/blob/master/kuka_external_control_sdk/doc/iiqka_os2_setup.md)

Setups for all three versions (`rsi_only`, `eki_rsi`, `mxa_rsi`) are available in this file.

## Driver configuration

### Startup configuration

The following configuration files are available in the `config` directory of the package:

- `driver_config.yaml`: contains runtime parameters of the `robot_manager` node
- `ros2_controller_config.yaml`: contains the controller types for every controller name. Should be only modified if a different controller is to be used.
- configuration files for specific controllers, for further information, see the documentation of the given controller


### Runtime parameters

The parameters in the driver configuration file can be changed during runtime using the parameter interface of the `robot_manager` node:

- `position_controller_name`: The name of the controller (string) that controls the `position` interface of the robot. It can't be changed in active state.
- `cycle_time`: The cycle time of RSI communication either 1 (4ms) or 2 (12ms). It can't be changed in active state.

### I/O configuration

KSS and iiQKA.OS2 support inputs and outputs for real-time usage through RSI. The I/Os are defined from the robot controller's point of view: an `input` can only have state interfaces in ROS Control, while an `output` can have both state and command interfaces.

RSI groups the I/Os into three categories:

- `BOOL`: The I/Os act as two-state signals and can be set to `true` or `false`.
- `DOUBLE`: The I/Os can store decimal numbers represented in floating-point format.
- `LONG`: The I/Os can store whole numbers using a 64-bit integer representation.

Generally, only a few constraints are imposed on naming the I/Os:

- The names of the I/Os must be unique keys.
- The names must be unique across both the state and command interfaces.
- Since `outputs` can have both state and command interfaces, if these interfaces are configured with the same name, they are considered connected. In this case, the system will handle them by first reading the state of the `output`, then writing to it via the command interface.


#### Client side I/O configuration

To configure the client side, two configuration files need to be completed:

1. The `kuka_rsi_driver/config/gpio_config.xacro` file is an extension to the robot's URDF and contains a `<gpio>` tag as part of the ROS Control parameters.
   - The GPIO object must be called `gpio`.
   - The state and command interfaces must be configured according to the example provided in the file.
   - For each interface, several additional parameters are available:
     - `name`: The previously mentioned unique key.
     - `data_type`: The data type of the interface. Must be one of the three supported by RSI: `BOOL`, `DOUBLE`, or `LONG`.
     - `limits`: Enables additional limit checking (defaults to `true`).
     - `min`: Minimum value for limit checking. (If not used or used incorrectly, `limits` is set to `false`.)
     - `max`: Maximum value for limit checking. (If not used or used incorrectly, `limits` is set to `false`.)
     - `initial_value`: Initial value of the interface. The value must be in a number format for every data type. Mostly useful for outputs without a state interface, as this value is otherwise overridden during the first cycle.
  - To set up the provided example, uncomment the constructed interfaces in the `gpio_config.xacro` config file.
   - An example with both state and command interfaces with all parameters:

      ```xml
      <command_interface name="OUTPUT_01" data_type="DOUBLE">
        <limits enable="true"/>
        <param name="min">0.0</param>
        <param name="max">100.0</param>
      </command_interface>
      <state_interface name="OUTPUT_01" data_type="DOUBLE">
        <limits enable="true"/>
        <param name="min">0.0</param>
        <param name="max">100.0</param>
        <param name="initial_value">50.0</param>
      </state_interface>
      ```

2. The `kuka_rsi_driver/config/gpio_controller_config.yaml` file defines the configuration for the I/O controller.
   - For I/O control, the [GpioCommandController](https://control.ros.org/master/doc/ros2_controllers/gpio_controllers/doc/userdoc.html) from ROS Control is used.
   - The controller requires a configuration file that describes the available state and command interfaces.
   - The `gpios` field contains a list of available GPIO interface groups. Currently, only one group is supported, and it **must** be named `gpio`.
   - Additionally, the file contains lists of available state and command interfaces:
     - These must be listed in groups, as explained in the linked controller documentation.
     - Ensure that the interface names match those defined earlier.

## Usage

### Starting the driver

1. To start the driver, two launch files are available, with and without `rviz`. To launch (without `rviz`), run:

    ```bash
    ros2 launch kuka_rsi_driver startup.launch.py
    ```

    - This starts the 3 core components of every driver (described in the [Non-real-time interface](https://github.com/kroshu/kuka_drivers/wiki#non-real-time-interface) section of the project overview) and the following controllers:
      - `joint_state_broadcaster` (no configuration file, all state interfaces are published)
      - `joint_trajectory_controller` ([configuration file](https://github.com/kroshu/kuka_drivers/tree/master/kuka_rsi_driver/config/joint_trajectory_controller_config.yaml))
    - There is no need to set the Client IP, since the driver automatically listens on the `0.0.0.0` address.
    - After successful startup, the `robot_manager` node has to be activated to start the cyclic communication with the robot controller, see further steps (before this only a collapsed robot is visible in `rviz`):

2. Configure and activate all components of the driver:

    ```bash
    ros2 lifecycle set robot_manager configure
    ros2 lifecycle set robot_manager activate
    ```

    - The hardware interface is now waiting for the robot controller to connect, the timeout for this is currently 10 seconds

3. Start external control according to your `driver_version`:
  - `rsi_only`: start the RSI program manually on the controller and execute `RSI_MOVECORR()`.
    - in T1, a warning (*!!! Attention - Sensor correction goes active !!!*) should be visible after reaching `RSI_MOVECORR()`, which should be confirmed to start this step
  - `eki_rsi` or `mxa_rsi`: RSI program is automatically selected and started
  

On successful activation the brakes of the robot will be released and external control is started. To test moving the robot, the `rqt_joint_trajectory_controller` is not recommended, use the launch file in the `iiqka_moveit_example` package instead (found in examples repo, usage is described in the [Additional packages](https://github.com/kroshu/kuka_drivers/wiki#moveit-integration) section of the project overview).

### Launch arguments

Both launch files support the following arguments:

- `client_port`: port of the client machine (default: 59152)
- `controller_ip`: The IP address of the KUKA Line Interface (KLI) - not used for `rsi_only` setup
- `mxa_client_port`: port of the client machine where mxAutomation packets are received (default: 1337)
- `robot_model` and `robot_family`: defines which robot to use. The available options for the valid model and family combinations can be found in the [readme](https://github.com/kroshu/kuka_robot_descriptions?tab=readme-ov-file#what-data-is-verified) of the `kuka_robot_descriptions` repository.
- `mode`: if set to 'mock', the `KukaMockHardwareInterface` will be used instead of the `KukaRSIHardwareInterface`. This enables trying out the driver without actual hardware.
- `use_gpio`: if set to `false` the usage of I/Os are disabled (defaults to `true`).
- `namespace`: adds a namespace to all nodes and controllers of the driver, and modifies the `prefix` argument of the robot description macro to `namespace_`
- `x`, `y`, `z`: define the position of `base_link` relative to the `world` frame in meters (default: [0, 0, 0])
- `roll`, `pitch`, `yaw`: define the orientation of `base_link` relative to the `world` frame in radians (default: [0, 0, 0])
- `roundtrip_time`: The roundtrip time (in microseconds) to be enforced by the [KUKA mock hardware interface](https://github.com/kroshu/kuka_robot_descriptions?tab=readme-ov-file#custom-mock-hardware), (defaults to 4000 us, only used if `mode` is set to 'mock')
- `controller_config_dir`: the directory that contains all controller configuration files (defaults to `kuka_rsi_driver/config`). The driver expects the following file names in this directory:
  - `ros2_controller_config_rsi_only.yaml` (used when `driver_version:=rsi_only`)
  - `ros2_controller_config_extended.yaml` (used when `driver_version:=eki_rsi` or `driver_version:=mxa_rsi`)
  - `joint_trajectory_controller_config.yaml`
  - `kuka_event_broadcaster_config.yaml`
  - `gpio_controller_config.yaml` (used only if `use_gpio:=true`)
  - `kuka_control_mode_handler_config.yaml` (used only if `driver_version:=eki_rsi` or `mxa_rsi`)
  - `kuka_kss_message_handler_config.yaml` (used only if `driver_version:=eki_rsi` or `mxa_rsi`)
- `driver_version`: configures which driver to use. Possible values are `rsi_only`, `eki_rsi` and `mxa_rsi` (defaults to `rsi_only`)
- `verify_robot_model`: If set to `true` and `driver_version` is set to `eki_rsi` or `mxa_rsi`, the driver will verify that the robot model specified in the launch arguments matches the configuration reported by the controller. If set to `false`, the reported configuration won't be checked (defaults to `true`).
- `rt_core`: CPU core index for taskset pinning of the realtime control thread. (default: -1 = do not pin)
- `rt_prio`: The realtime priority of the thread that runs the control loop [0-99] (default: 70)
- `non_rt_cores`: Comma-separated CPU core indices for taskset pinning of non-RT threads (e.g. '2,3,4'). Leave empty to disable pinning. (defaults to empty string)
- `lock_memory`: Whether to lock memory of the control loop with mlockall to avoid paging (defaults to true)


The `startup_with_rviz.launch.py` additionally contains one argument:

- `rviz_config`: the location of the `rviz` configuration file (defaults to `kuka_resources/config/view_6_axis_urdf.rviz`)

**Details** about the `mode` parameter can be viewed in the [kuka_robot_descriptions README](https://github.com/kroshu/kuka_robot_descriptions?tab=readme-ov-file#modes).

### Stopping external control

To stop external control, all components have to be deactivated with `ros2 lifecycle set robot_manager deactivate`

BEWARE, that this is a non-realtime process including lifecycle management, so the connection is not terminated immediately, in cases where an abrupt stop is needed, the safety stop of the Teach Pendant should be used!

## Simulation

To try out the driver with an open-loop simulation, the driver and the `kuka_rsi_simulator` must be started, (before activation only a "collapsed" robot will be visible in `rviz`):

```bash
ros2 launch kuka_rsi_driver startup_with_rviz.launch.py
```

```bash
ros2 launch kuka_rsi_simulator kuka_rsi_simulator.launch.py
```

After all components have started successfully, the system needs to be configured and activated to start the simulation. The robot will be visible in rviz after activation:

```bash
ros2 lifecycle set robot_manager configure
ros2 lifecycle set robot_manager activate
```


## External axes configuration

Both KSS and the RSI option package support adding external axes to the robot. We provide an [example](https://github.com/kroshu/examples/blob/master/kuka_external_axis_examples) that integrates a single linear axis. This example, together with the structure and documentation, should help users implement their own external‑axis configurations.

### Controller-side configuration

#### Context

The `Config/User/Common/SensorInterface/rsi_ext_axis_example.rsix` contains an example setup with one linear external axis.

Compared to the original context the following changes were required:

- Add the `AxisCorrExt` object.
- Connect `Ethernet` object's `Out8` output to the first input of `AxisCorrExt`.
- Update the `LowerLimE1` and `UpperLimE1` parameters of `AxisCorrExt`.
- Update the `MaxE1` parameter of the `AxisCorrMon` object.

To create a new custom context:

- Connect the next `OutX` output of the `Ethernet` object to the corresponding `CorrEX` input of `AxisCorrExt` for each external axis.
- Adjust limits (`MaxEX`, `LowerLimX`, `UpperLimX`) accordingly.

#### Ethernet configuration

The configuration file referenced by the Ethernet object must also be updated. See the example in: `Config/User/Common/SensorInterface/rsi_ext_axis_ethernet.xml`.

The only difference from the original configuration is an additional line in the `RECEIVE` block:

```xml
<ELEMENT TAG="EK.E1" TYPE="DOUBLE" INDX="8" HOLDON="1" />
```

This allows RSI to parse data from the driver.

> [!IMPORTANT]
> Use a consistent naming convention for external-axis values (`TAG="EK.EX"`), incrementing `X` for each axis. Ensure `INDX` values also increase sequentially.

> [!IMPORTANT]
> When using GPIOs, list external axes before adding GPIO message configuration. The correct order is: internal axes &rarr; external axes &rarr; GPIOs.

#### Program

To adapt the KRL program for the external-axis example:

- KSS: update the RSI context name in `KRC/R1/Program/RSI/rsi_joint_pos_4ms.src` or `KRC/R1/Program/RSI/rsi_joint_pos_12ms.src` to `rsi_ext_axis_example`.
- iiQKA.OS2: update the `CONTEXT_NAME` variable in `Program/RSI/rsi_joint_pos.dat` to `rsi_ext_axis_example`.

For custom setups, use the name of the corresponding context file.

### Client-side configuration

See the [kuka_robot_descriptions README](https://github.com/kroshu/kuka_robot_descriptions/blob/master/README.md#external-axes-configuration) for all client-side configuration steps.

> [!NOTE]
> The driver supports only __revolute__ and __prismatic__ external joints.


## Known issues and limitations

- In case of an error on the controller side, the driver is not deactivated
- Cartesian position control mode is not yet supported