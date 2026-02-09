# iiQka driver (RSI)

## Setup

### Test setup

Tested configurations:

| Controller | Robot              | iiQKA.OS2 Version | RSI Version |
|------------|--------------------|-------------------|-------------|
| KR C5 OPS  | &ndash;            | 9.1.0             | 6.1.2       |
| KR C5      | KR 16 R1610-2      | 9.1.0             | 6.1.2       |

### Client side

It is recommended to run the driver on a real-time capable client machine. Detailed instructions for setting up the `PREEMPT_RT` path are available on the [Realtime](https://github.com/kroshu/kuka_drivers/wiki/6_Realtime) wiki page.

To set up the controller with iiQWorks.Sim (which is necessary if RSI is not yet installed), a Windows machine is also required.

#### Client IP configuration

- Use the KRC's built in DHCP server (KSI) to connect to the windows machine (in this case leave the subnet settings as default) or set a fixed IP in the subnet of the KLI interface. This is required to connect to iiQWorks.Sim and transfer the project to the KRC
- Set a fixed IP in the subnet of the KLI interface for the real-time machine, which is required to send commands via the RSI interface.

### Controller side

These instructions were tested with RSI 6.0.0 (on iiQKA.OS2)

#### Controller network configuration

RSI can only communicate via the KLI network. Make sure that the controller network and the **PC with ROS** is connected to the same subnet.

1. Log in as **Expert**, **Safety Maintenance Technician** or **Administrator** on the smartped and in the main menu bar navigate to **Network** menu (**System > System settings > Network**).

2. Change network settings:
    - Set the **Allocation** to manual, to connect the ROS PC directly with the controller
    - Set the **IP address** and the **Subnet mask** to the chosen address

3. After pressing **Apply** the new network address should be configured.

#### RSI configuration

Configure your project in iiQWorks.Sim with the following steps:

1. Add the desired RSI version (6.0.0 or above) to iiQWorks.Sim
    - Download RSI option package from the **Software Repository** in **iiQworks.Cockpit**
    - Open the Option Package Manager by the manage button in the **Add-ons** (**File > Options > Add-on > KUKA Option packages > Manage**)
    - By clicking on the **plus sign** add the downloaded option package from your file system (it should be a **.kop** file).

2. After adding the desired robot from the eCatalog by dragging it to the layout configure the option packages to use.
    - In the **Component properties**, under **Option packages configuration** add the desired option packages with the plus sign.

3. After adding the RSI option package under the **Home** page in the **devices** tab under **Option packages > iiQka.RobotSensorInterface**, there are two menu options available:
    - The first one is a list of available context files. You can import, export and create new context files. The files can be visualized and edited with **RSIVisual**.
    - The second is a list of ethernet configuration files. You can import, export and create new context files. The files can be visualized and edited with **XML editor**.

4. Under the **Program** page the KRL program files are available

#### Update and upload configuration files

Several files required for RSI can be found in the [`kuka-external-control-sdk`](https://github.com/kroshu/kuka-external-control-sdk) repository, located in the `kuka_external_control_sdk/krc_setup/iiqka_os2` directory:

- `Program/RSI/rsi_joint_pos.src`: This contains the KRL program that starts external control with 4 ms cycle time.
- `Program/RSI/rsi_helper.dat` and `Program/RSI/rsi_helper.src`: These are used for configuring the RSI context based on the current robot position.
- `RobotSensorInterface/Context/ros_rsi.rsix`: This contains the RSI context (can be visualized with **RSIVisual**). It can be modified for example to add GPIO handling ([See further documentation on this here](#io-configuration)), or to add filtering behaviour, but that is not recommended and should be implemented on the client side instead.
  - RSI context files for KSS systems (RSI < 6.0.0) are currently not importable to iiQWorks.Sim.
- `RobotSensorInterface/Ethernet_configuration/rsi_ethernet.xml`: specifies the data transferred via RSI and contains the IP configuration of the client machine:
  - The `IP_NUMBER` tag should be modified so that it corresponds to the IP address previously added for your (real-time) PC.
  - The `PORT` might be left as it is (59152), but can be also changed if a different port is to be used on the client machine.

Upload files to the controller:

1. Connect to the KRC with iiQWorks.Sim
2. Log in as **Expert** or **Administrator** on the controller and change the operation mode to **T1**.
3. Import the `rsi_joint_pos.src`, `rsi_helper.dat` and `rsi_helper.src` files to `Program` folder under **Program** page in iiQWorks.Sim.
4. Import the `rsi_joint_pos.rsix` file under **Context** field in the **Home** page.
5. Import the `rsi_ethernet.xml` file under **Ethernet configurations** field in the **Home** page.
6. Move to the **Configuration** page and deploy the configuration.

## Configuration

Important to note that for the iiQKA.OS2 you can use the same RSI driver as for the KSS OS, as they use the same [SDK implementation](https://github.com/kroshu/kuka-external-control-sdk/tree/master/kuka_external_control_sdk/kss).

### Startup configuration

The following configuration files are available in the `config` directory of the package:

- `driver_config.yaml`: : contains runtime parameters of the `robot_manager` node
- `ros2_controller_config.yaml`: contains the controller types for every controller name. Should be only modified if a different controller is to be used.
- configuration files for specific controllers, for further information, see the documentation of the given controller

#### IP configuration

The following parameters must be set in the driver configuration file:

- `client_ip`: IP address of the client machine, should be identical to the one set in `ros_rsi_ethernet.xml`
- `client_port`: port of the real-time communication on the client machine, should be identical to the one set in `ros_rsi_ethernet.xml`

### Runtime parameters

The parameters in the driver configuration file can be also changed during runtime using the parameter interface of the `robot_manager` node:

- `position_controller_name`: The name of the controller (string) that controls the `position` interface of the robot. It can't be changed in active state.

### IP Configuration

The IP address of the client machine must be provided as a launch argument. For further information see section [launch arguments](#launch-arguments).

#### I/O configuration

The iiQKA.OS2 robot system supports the use of inputs and outputs to control grippers, conveyor belts, or to read sensor data. The RSI system also supports controlling these I/Os in real time. The I/Os are defined from the robot controller’s point of view: an `input` can only have state interfaces in ROS Control, while an `output` can have both state and command interfaces. In the iiQKA.OS2 robot system, I/Os can be configured with different types according to their size and representation.

RSI groups the I/Os into three categories:

- `BOOL`: The I/Os act as two-state signals and can be set to `true` or `false`.
- `DOUBLE`: The I/Os can store decimal numbers represented in floating-point format.
- `LONG`: The I/Os can store whole numbers using a 64-bit integer representation.

Generally, only a few constraints are imposed on naming the I/Os:

- The names of the I/Os must be unique keys.
- The names must be unique across both the state and command interfaces.
- Since `outputs` can have both state and command interfaces, if these interfaces are configured with the same name, they are considered connected. In this case, the system will handle them by first reading the state of the `output`, then writing to it via the command interface.

##### Controller side configuration

To configure the controller side, two additional files are available in the `kuka_external_control_sdk/krc_setup/iiqka_os2` directory:

1. The `RobotSensorInterface/Context/rsi_gpio_joint_pos.rsix` file offers an example of how to set up the different I/Os. For detailed instructions, please refer to the RSI manual on KUKA Xpert.
   - The file can be edited via RSI Visual in iiQWorks.Sim.
   - All I/Os should be connected to the inputs or outputs of the Ethernet RSI object.

2. The Ethernet RSI object requires the `RobotSensorInterface/Ethernet_configuration/rsi_gpio_ethernet.xml` configuration file.
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
      <ELEMENT TAG="GPIO.OUTPUT_01" TYPE="DOUBLE" INDX="1" HOLDON="1" />
      ```

3. To run the GPIO example, a `Program/RSI/rsi_gpio_example.src` file has been added.

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
   - For I/O control, the [GpioCommandController](https://control.ros.org/humble/doc/ros2_controllers/gpio_controllers/doc/userdoc.html) from ROS Control is used.
   - The controller requires a configuration file that describes the available state and command interfaces.
   - The `gpios` field contains a list of available GPIO interface groups. Currently, only one group is supported, and it **must** be named `gpio`.
   - Additionally, the file contains lists of available state and command interfaces:
     - These must be listed in groups, as explained in the linked controller documentation.
     - Ensure that the interface names match those defined earlier.

## Usage

### Starting the driver

1. To start the driver, two launch file are available, with and without `rviz`. To launch (without `rviz`), run:

    ```bash
    ros2 launch kuka_rsi_driver startup.launch.py
    ```

    - This starts the 3 core components of every driver (described in the [Non-real-time interface](https://github.com/kroshu/kuka_drivers/wiki#non-real-time-interface) section of the project overview) and the following controllers:
      - `joint_state_broadcaster` (no configuration file, all state interfaces are published)
      - `joint_trajectory_controller` ([configuration file](https://github.com/kroshu/kuka_drivers/tree/humble/kuka_rsi_driver/config/joint_trajectory_controller_config.yaml))
    - There is no need to set the Client IP, since the driver automatically listens on the `0.0.0.0` address.
    - After successful startup, the `robot_manager` node has to be activated to start the cyclic communication with the robot controller, see further steps (before this only a collapsed robot is visible in `rviz`):

2. Configure and activate all components the driver:

    ```bash
    ros2 lifecycle set robot_manager configure
    ros2 lifecycle set robot_manager activate
    ```

    - The hardware interface is now waiting for the robot controller to connect, the timeout for this is currently 10 seconds
3. In the **Programming** menu, under the **Navigator** tab start the `Program\ros_rsi.src` program on the controller and execute the line of `RSI_MOVECORR()`
   - in T1, a warning (*!!! Attention - Sensor correction goes active !!!*) should be visible after reaching `RSI_MOVECORR()`, which should be confirmed to start this step

On successful activation the brakes of the robot will be released and external control is started. To test moving the robot, the `rqt_joint_trajectory_controller` is not recommended, use the launch file in the `iiqka_moveit_example` package instead (usage is described in the [Additional packages](https://github.com/kroshu/kuka_drivers/wiki#additional-packages) section of the project overview).

#### Launch arguments

Both launch files support the following arguments:

- `client_port`: port of the client machine (default: 59152)
<<<<<<< HEAD
- `robot_model` and `robot_family`: defines which robot to use. The available options for the valid model and family combinations can be found in the [readme](https://github.com/kroshu/kuka_robot_descriptions/tree/humble?tab=readme-ov-file#what-is-verified) of the `kuka_robot_descriptions` repository.
=======
- `mxa_client_port`: port of the client machine where mxAutomation packets are received (default: 1337)
- `robot_model` and `robot_family`: defines which robot to use. The available options for the valid model and family combinations can be found in the [readme](https://github.com/kroshu/kuka_robot_descriptions?tab=readme-ov-file#what-is-verified) of the `kuka_robot_descriptions` repository.
>>>>>>> 112b893 (Fix the usage of the Client IP and Client Port for RSI connection and clarify outputs (#300))
- `mode`: if set to 'mock', the `KukaMockHardwareInterface` will be used instead of the `KukaRSIHardwareInterface`. This enables trying out the driver without actual hardware.
- `use_gpio`: if set to `false` the usage of I/Os are disabled (defaults to `true`).
- `namespace`: adds a namespace to all nodes and controllers of the driver, and modifies the `prefix` argument of the robot description macro to `namespace_`
- `x`, `y`, `z`: define the position of `base_link` relative to the `world` frame in meters (default: [0, 0, 0])
- `roll`, `pitch`, `yaw`: define the orientation of `base_link` relative to the `world` frame in radians (default: [0, 0, 0])
- `roundtrip_time`: The roundtrip time (in microseconds) to be enforced by the [KUKA mock hardware interface](https://github.com/kroshu/kuka_robot_descriptions/tree/humble?tab=readme-ov-file#custom-mock-hardware), (defaults to 2500 us, only used if `mode` is set to 'mock')
- `controller_config`: the location of the `ros2_control` configuration file (defaults to `kuka_rsi_driver/config/ros2_controller_config.yaml`)
- `jtc_config`: the location of the configuration file for the `joint_trajectory_controller` (defaults to `kuka_rsi_driver/config/joint_trajectory_controller_config.yaml`).
- `rt_core`: CPU core index for taskset pinning of the realtime control thread. (default: -1 = do not pin)
- `rt_prio`: The realtime priority of the thread that runs the control loop [0-99] (default: 70)
- `non_rt_cores`: Comma-separated CPU core indices for taskset pinning of non-RT threads (e.g. '2,3,4'). Leave empty to disable pinning. (defaults to empty string)

The `startup_with_rviz.launch.py` additionally contains one argument:

- `rviz_config`: the location of the `rviz` configuration file (defaults to `kuka_resources/config/view_6_axis_urdf.rviz`)

**Details** about the `mode` parameter can be viewed in the [kuka_robot_descriptions README](https://github.com/kroshu/kuka_robot_descriptions/tree/humble?tab=readme-ov-file#modes).

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

## Known issues and limitations

- In case of an error on the controller side, the driver is not deactivated
- Cartesian position control mode not yet supported
