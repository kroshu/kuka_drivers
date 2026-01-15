# KSS driver (RSI)

This guide provides instructions for setting up and using a ROS 2 driver to control KUKA robots running on **KUKA System Software (KSS)**. The driver supports two configurations:

- [RSI-only driver](#rsi-only-driver): Utilizes the **Robot Sensor Interface (RSI)** option package for direct robot control
- [EKI + RSI driver](#eki--rsi-driver): Combines the **Ethernet KRL Interface (EKI)** with RSI to enable runtime configuration

## RSI-only driver

This section is for users who want to control their KUKA robot using only the RSI option package.

### Test setup for the RSI-only driver

Tested configurations:

| Controller | Robot                | KSS Version | RSI Version |
|------------|----------------------|-------------|-------------|
| KR C4 OPS  | &ndash;              | 8.6.11      | 4.1.3       |
| KR C5 OPS  | &ndash;              | 8.7.5       | 5.0.2       |
| KR C5      | KR 120 R2700-2 Dummy | 8.7.5       | 5.0.2       |
| KR C5      | KR 6 R900-2          | 8.7.5       | 5.0.2       |

### Driver setup with RSI only

#### Client side

It is recommended to run the driver on a real-time capable client machine. Detailed instructions for setting up the `PREEMPT_RT` path are available on the [Realtime](https://github.com/kroshu/kuka_drivers/wiki/6_Realtime) wiki page.

To set up the controller with WorkVisual (which is necessary if RSI is not yet installed), a Windows machine is also required.

##### Client IP configuration

- Set a fixed IP in the subnet of the KLI interface for the Windows machine, which is required to connect to WorkVisual and transfer the project
- Set a fixed IP in the subnet of the RSI interface for the real-time machine, which is required to send commands via the RSI interface.

#### Controller side

##### Controller network configuration

Windows runs behind the SmartHMI on the teach pad. Make sure that the **Windows interface** of the controller and the **PC with ROS** is connected to the same subnet.

1. Log in as **Expert** or **Administrator** on the teach pad and navigate to **Network configuration** (**Start-up > Network configuration > Activate advanced configuration**). There should already be an interface checked out as the **Windows interface**.
    - **Windows interface checkbox** should be checked.
2. Add a new network for RSI:
    - Press the **Advanced** button and **New interface**.
    - Select **Mixed IP address** and keep the default settings:
      - **Receiving task: Target subnet**
      - **Real-time receiving Task: UDP**
    - Set the IP address to a different subnet then the **KLI interface**.
      - **Default gateway**: leave it empty
      - **Windows interface checkbox** should NOT be checked
3. Reboot the controller with a cold restart (**Shutdown > Check *Force cold start* and *Reload files* > Reboot control PC**).

##### Update and upload configuration files

Several files required for RSI can be found in the [`kuka-external-control-sdk`](https://github.com/kroshu/kuka-external-control-sdk) repository, located in the `kuka_external_control_sdk/krc_setup/kss` directory with a tree structure that resembles the krc folder structure:

- `Config/User/Common/SensorInterface/rsi_ethernet.xml`: specifies the data transferred via RSI and contains the IP configuration of the client machine:
  - The `IP_NUMBER` tag should be modified so that it corresponds to the IP address previously added for your (real-time) PC.
  - The `PORT` might be left as it is (59152), but can be also changed if a different port is to be used on the client machine.
- `Config/User/Common/SensorInterface/rsi_joint_pos.rsix`: This contains the RSI context (can be visualized with **RSIVisual**). It can be modified for example to add GPIO handling ([See further documentation on this here](#io-configuration)), or to add filtering behaviour, but that is not recommended and should be implemented on the client side instead.
- `KRC/R1/Program/RSI/rsi_helper.dat` and `KRC/R1/Program/RSI/rsi_helper.src`: These are used for configuring the RSI context based on the current robot position.
- `KRC/R1/Program/RSI/rsi_joint_pos_4ms.src` and `KRC/R1/Program/RSI/rsi_joint_pos_12ms.src`: These contain KRL programs that start external control. You may choose what cycle time RSI should use (4 ms or 12 ms).

If you are using an older version of RSI (i.e., <=4.0.3), the RSI context must be defined using three separate files&mdash;`rsi_joint_pos.rsi`, `rsi_joint_pos.rsi.diagram` and `rsi_joint_pos.rsi.xml`&mdash;instead of a single `.rsix` file. These files can be found in the `kuka_external_control_sdk/kss/krl/SensorInterface/deprecated` directory of the [`kuka-external-control-sdk`](https://github.com/kroshu/kuka-external-control-sdk) repository. Use these files in place of the `rsi_joint_pos.rsix` context file mentioned above.

There are two options to upload these files to the controller:

Method 1:

1. Copy the files to a USB-stick.
2. Plug it into the teach pad or controller.
3. Log in as **Expert** or **Administrator** on the controller.
4. Copy the contents of the `krc_setup/kss/KRC/R1/Program` folder to `KRC:\R1\Program`.
5. Copy the contents of the `krc_setup/kss/Config/User/Common` folder to `C:\KRC\ROBOTER\Config\User\Common`.

Method 2:

1. Connect to the KRC with WorkVisual
2. Log in as **Expert** or **Administrator** on the controller.
3. Copy the contents of the `krc_setup/kss/KRC/R1/Program` folder to `KRC:\R1\Program` in WorkVisual
4. Copy the contents of the `krc_setup/kss/Config/User/Common` folder to `C:\KRC\ROBOTER\Config\User\Common` in WorkVisual
5. Deploy the project

### Configuration

#### Startup configuration

The following configuration files are available in the `config` directory of the package:

- `driver_config.yaml`: : contains runtime parameters of the `robot_manager` node
- `ros2_controller_config.yaml`: contains the controller types for every controller name. Should be only modified if a different controller is to be used.
- configuration files for specific controllers, for further information, see the documentation of the given controller

##### IP configuration

The following parameters must be set in the driver configuration file:

- `client_ip`: IP address of the client machine, should be identical to the one set in `rsi_ethernet.xml`
- `client_port`: port of the real-time communication on the client machine, should be identical to the one set in `rsi_ethernet.xml`

#### Runtime parameters

The parameters in the driver configuration file can be also changed during runtime using the parameter interface of the `robot_manager` node:

- `position_controller_name`: The name of the controller (string) that controls the `position` interface of the robot. It can't be changed in active state.

#### I/O configuration

The KSS robot system supports the use of inputs and outputs to control grippers, conveyor belts, or to read sensor data. The RSI system also supports controlling these I/Os in real time. The I/Os are defined from the robot controller’s point of view: an `input` can only have state interfaces in ROS Control, while an `output` can have both state and command interfaces. In the KSS robot system, I/Os can be configured with different types according to their size and representation.

RSI groups the I/Os into three categories:

- `BOOL`: The I/Os act as two-state signals and can be set to `true` or `false`.
- `DOUBLE`: The I/Os can store decimal numbers represented in floating-point format.
- `LONG`: The I/Os can store whole numbers using a 64-bit integer representation.

Generally, only a few constraints are imposed on naming the I/Os:

- The names of the I/Os must be unique keys.
- The names must be unique across both the state and command interfaces.
- Since `outputs` can have both state and command interfaces, if these interfaces are configured with the same name, they are considered connected. In this case, the system will handle them by first reading the state of the `output`, then writing to it via the command interface.

##### Controller side configuration

To configure the controller side, three addition files are available in the `kuka_external_control_sdk/krc_setup/kss` directory:

1. The `Config/User/Common/SensorInterface/rsi_gpio_joint_pos.rsix` file provides an example of how to set up the different I/Os. For detailed instructions, please refer to the RSI manual on KUKA Xpert.
   - The file can be edited via the RSI Visual in WorkVisual.
   - All I/Os should be connected to the inputs or outputs of the Ethernet RSI object.
2. To run the GPIO example, a `KRC/R1/Program/rsi_gpio_example.src` file has been added.
3. The Ethernet RSI object in the `Config/User/Common/SensorInterface/rsi_gpio_joint_pos.rsix` requires the `Config/User/Common/SensorInterface/rsi_gpio_ethernet.xml` configuration file.
   - The `<SEND>` object contains all parameters that are sent to the client.
   - The `<RECEIVE>` object contains all the parameters that are received from the client.
   - To add a new I/O element the following parameters must be set:
     - `TAG`: Contains the aforementioned unique key. The tag format is: `GPIO.UniqueKey`. It must start with `GPIO`, followed by a `.`, and then the `key`.
     - `TYPE`: The type can be one of the following: `BOOL`, `DOUBLE`, or `LONG`.
     - `INDX`: This must match the configuration of the I/O object in RSI Visual for the Ethernet object. This is the only parameter that connects the XML file entries to those in the `.rsix` file.
     - `HOLDON`: This is only used in the `<RECEIVE>` object. It sets the behavior of the output when packets are missed:
       - `0`: The output is reset.
       - `1`: The most recent valid value remains at the output.
   - A sample configuration for one I/O element:

      ```xml
      <ELEMENT TAG="GPIO.01" TYPE="DOUBLE" INDX="1" HOLDON="1" />
      ```

##### Client side configuration

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
   - To setup the prvided example uncomment the constructed interfaces in the `gpio_config.xacro` config file
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

### Usage

#### Starting the driver

1. To start the driver, two launch file are available, with and without `rviz`. To launch (without `rviz`), run:

    ```bash
    ros2 launch kuka_rsi_driver startup.launch.py
    ```

    - This starts the 3 core components of every driver (described in the [Non-real-time interface](https://github.com/kroshu/kuka_drivers/wiki#non-real-time-interface) section of the project overview) and the following controllers:
      - `joint_state_broadcaster` (no configuration file, all state interfaces are published)
      - `joint_trajectory_controller` ([configuration file](https://github.com/kroshu/kuka_drivers/tree/master/kuka_rsi_driver/config/joint_trajectory_controller_config.yaml))
    - There is no need to set the Client IP, since the driver automatically listens on the `0.0.0.0` address.
    - After successful startup, the `robot_manager` node has to be activated to start the cyclic communication with the robot controller, see further steps (before this only a collapsed robot is visible in `rviz`):

2. Configure and activate all components the driver:

    ```bash
    ros2 lifecycle set robot_manager configure
    ros2 lifecycle set robot_manager activate
    ```

    - The hardware interface is now waiting for the robot controller to connect, the timeout for this is currently 10 seconds

3. Start the `KRC:\R1\Program\rsi_joint_pos_4ms.src`/`KRC:\R1\Program\rsi_joint_pos_12ms.src` program on the controller and execute the line of `RSI_MOVECORR()`
    - in T1, a warning (*!!! Attention - Sensor correction goes active !!!*) should be visible after reaching `RSI_MOVECORR()`, which should be confirmed to start this step

On successful activation the brakes of the robot will be released and external control is started. To test moving the robot, the `rqt_joint_trajectory_controller` is not recommended, use the launch file in the `iiqka_moveit_example` package instead (usage is described in the [Additional packages](https://github.com/kroshu/kuka_drivers/wiki#additional-packages) section of the project overview).

##### Launch arguments

Both launch files support the following arguments:

- `client_port`: port of the client machine (default: 59152)
- `robot_model` and `robot_family`: defines which robot to use. The available options for the valid model and family combinations can be found in the [readme](https://github.com/kroshu/kuka_robot_descriptions?tab=readme-ov-file#what-data-is-verified) of the `kuka_robot_descriptions` repository.
- `mode`: if set to 'mock', the `KukaMockHardwareInterface` will be used instead of the `KukaRSIHardwareInterface`. This enables trying out the driver without actual hardware.
- `use_gpio`: if set to `false` the usage of I/Os are disabled (defaults to `true`).
- `namespace`: adds a namespace to all nodes and controllers of the driver, and modifies the `prefix` argument of the robot description macro to `namespace_`
- `x`, `y`, `z`: define the position of `base_link` relative to the `world` frame in meters (default: [0, 0, 0])
- `roll`, `pitch`, `yaw`: define the orientation of `base_link` relative to the `world` frame in radians (default: [0, 0, 0])
- `roundtrip_time`: The roundtrip time (in microseconds) to be enforced by the [KUKA mock hardware interface](https://github.com/kroshu/kuka_robot_descriptions?tab=readme-ov-file#custom-mock-hardware), (defaults to 2500 us, only used if `mode` is set to 'mock')
- `controller_config`: the location of the `ros2_control` configuration file (defaults to `kuka_rsi_driver/config/ros2_controller_config.yaml`)
- `jtc_config`: the location of the configuration file for the `joint_trajectory_controller` (defaults to `kuka_rsi_driver/config/joint_trajectory_controller_config.yaml`)
- `driver_version`: configures which driver to use. Possible values are `rsi_only` and `eki_rsi` (defaults to `rsi_only`)
- `verify_robot_model`: If set to `true` and `driver_version` is set to `eki_rsi`, the driver will verify that the robot model specified in the launch arguments matches the configuration reported by the controller. If set to `false`, the reported configuration won't be checked (defaults to `true`).
- `cm_log_level` (only jazzy): It is possible to set the `controller_manager`'s log level with this argument, to avoid flooding the log output by warnings about cycle time violations
- `rt_core`: CPU core index for taskset pinning of the realtime control thread. (default: -1 = do not pin)
- `rt_prio`: The realtime priority of the thread that runs the control loop [0-99] (default: 70)
- `non_rt_cores`: Comma-separated CPU core indices for taskset pinning of non-RT threads (e.g. '2,3,4'). Leave empty to disable pinning. (defaults to empty string)


The `startup_with_rviz.launch.py` additionally contains one argument:

- `rviz_config`: the location of the `rviz` configuration file (defaults to `kuka_resources/config/view_6_axis_urdf.rviz`)

**Details** about the `mode` parameter can be viewed in the [kuka_robot_descriptions README](https://github.com/kroshu/kuka_robot_descriptions?tab=readme-ov-file#modes).

#### Stopping external control

To stop external control, all components have to be deactivated with `ros2 lifecycle set robot_manager deactivate`

BEWARE, that this is a non-realtime process including lifecycle management, so the connection is not terminated immediately, in cases where an abrupt stop is needed, the safety stop of the Teach Pendant should be used!

### Simulation

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

### Known issues and limitations

- In case of an error on the controller side, the driver is not deactivated
- Cartesian position control mode and I/O-s not yet supported

## EKI + RSI driver

This section explains how to configure and use the driver when both EKI and RSI are available on the KUKA controller.

### Test setup for the EKI + RSI driver

Tested configurations:

| Controller | Robot                | KSS Version | EthernetKRL Version | RSI Version |
|------------|----------------------|-------------|---------------------|-------------|
| KR C4 OPS  | &ndash;              | 8.6.11      | 3.1.4               | 4.1.3       |
| KR C5 OPS  | &ndash;              | 8.7.5       | 3.2.5               | 5.0.2       |
| KR C5      | KR 120 R2700-2 Dummy | 8.7.5       | 3.2.5               | 5.0.2       |
| KR C5      | KR 6 R900-2          | 8.7.5       | 3.2.5               | 5.0.2       |

### Setting up the EKI + RSI driver

To set up the driver for use with both EKI and RSI, follow the instructions in the guide posted in the SDK repository: [External control setup for KSS with EKI](https://github.com/kroshu/kuka-external-control-sdk/blob/master/kuka_external_control_sdk/doc/kss_eki_setup.md)

### Launching the EKI + RSI driver

To start the KUKA RSI driver, use one of the provided launch files:

- `startup.launch.py`: Launches the driver only
- `startup_with_rviz.launch.py`: Launches the driver along with RViz for visualization

Regardless of the launch file you choose, the following arguments must be specified:

- `robot_family`: Defines the family of the robot (e.g., agilus)
- `robot_model`: Specifies the exact robot model. A list of supported models and their corresponding families is available in the `kuka_robot_descriptions` [README](https://github.com/kroshu/kuka_robot_descriptions?tab=readme-ov-file#what-is-verified)
- `controller_ip`: The IP address of the **KUKA Line Interface (KLI)**
- `driver_version`: Set this parameter to `eki_rsi`. If not set the plain RSI driver will be used.

For instance, to launch the driver for a KR 10 R1100-2 robot from the Agilus family, one could use the following command:

```bash
ros2 launch kuka_rsi_driver startup.launch.py robot_family:=agilus robot_model:=kr10_r1100_2 controller_ip:=172.31.1.147 driver_version:=eki_rsi
```

All other launch arguments described in the [Launch arguments](#launch-arguments) section are supported and applicable to this variant of the driver as well.

Once the driver is launched, one can use the standard ROS 2 lifecycle transitions:

- `ros2 lifecycle set /robot_manager configure`: Establishes a connection to the robot controller
- `ros2 lifecycle set /robot_manager activate`: Starts the RSI program on the robot controller and initiates external control
- `ros2 lifecycle set /robot_manager deactivate`: Stops external control and cancels the RSI program
- `ros2 lifecycle set /robot_manager cleanup`: Terminates the connection to the robot controller

The integration of EKI not only helps the initiation of external control but also unlocks additional capabilities via ROS 2 controllers. For more details, refer to the [Controllers](https://github.com/kroshu/kuka_drivers/wiki/5_Controllers) wiki page.
