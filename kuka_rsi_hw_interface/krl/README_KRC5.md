# Configuring RSI on the controller

This guide highlights the steps needed in order to successfully configure the **RSI interface** on the KRC5 controller to work with the **kuka_rsi_hardware_interface** on your PC with ROS.

## 1. Controller network configuration

Windows runs behind the SmartHMI on the teach pad. Make sure that the **Windows interface** of the controller and the **PC with ROS** is connected to the same subnet.

1. Log in as **Expert** or **Administrator** on the teach pad and navigate to **Network configuration** (**Start-up > Network configuration > Activate advanced configuration**).
2. There should already be an interface checked out as the **Windows interface**. For example:
   * **IP**: 192.168.250.20
   * **Subnet mask**: 255.255.255.0
   * **Default gateway**: 192.168.250.20 or leave it empty
   * **Windows interface checkbox** should be checked.
3. Add a new interface bz pressign the **Advanced** button and **New interface**.
4. Select **Mixed IP address** and keep the default **Receiving task: Target subnet** and **Real-time receiving Task: UDP**
5. Set the IP address to a different subnet then the **KLI interface**. For example:
   * **IP**: 192.168.10.20
   * **Subnet mask**: 255.255.255.0
   * **Default gateway**: leave it empty
   * **Windows interface checkbox** should NOT be checked
6. Save the changes and reboot the robot controller using the settings **Cold start** and **Reload files**

## 2. Set fix IP in your PC
1. Set one IP in the subnet of the KLI interface, it is required to connect the WorkVisual and transfer project. For example:
   * **IP**: 192.168.250.10   
   * **Subnet mask**: 255.255.255.0
   * **Default gateway**: 192.168.250.20 or leave it empty
2. Add another IP in the subnet of the RSI interface, it is required to send commands via the RSI interface. For example:
   * In windows **Network and Sharing center**, select the internet adapter settings *e.g. Ethernet Properties.
   * Select the IPV4 properties and their the **Advanced** settings.
   * In the new window, you can **Add** new IP addresses.

## 2. KRL Files

The files included in this folder specifies the data transferred via RSI. Some of the files needs to be modified to work for your specific configuration.

##### ros_rsi_ethernet.xml
1. Edit the `IP_NUMBER` tag so that it corresponds to the IP address (192.168.1.xx) previously added for your PC.
2. Keep the `PORT` tag as it is (59152) or change it if you want to use another port.

Note that the `rsi_ip_address` and `rsi_port` tags of the kuka_kr6_support/urdf/kr6r700sixx_macro_ros2_control.xacro (inside kuka_simulators) must correspond to the `IP_NUMBER`and `PORT` set in these KRL files.

##### ros_rsi.src
This should only be edited if the start position specified within the file is not desirable for your application.

##### Copy files to controller
The files **ros_rsi.rsi** and **ros_rsi.rsi.diagram** should not be edited. All files are now ready to be copied to the Kuka controller:

Method 1:
1. Copy the files to a USB-stick.
2. Plug it into the teach pad or controller.
3. Log in as **Expert** or **Administrator**.
4. Copy the `ros_rsi.src` file to `KRC:\R1\Program`.
5. Copy the rest of the files to `C:\KRC\ROBOTER\Config\User\Common\SensorInterface`.

Method 2:
1. Use the WorkVisual, connect to the KRC
2. Log in as **Expert** or **Administrator**.
4. Copy the `ros_rsi.src` file to `KRC:\R1\Program` in the WorkVisual
5. Copy the rest of the files to `C:\KRC\ROBOTER\Config\User\Common\SensorInterface` in the WorkVisual
6. Deploy the project, and follow the orders

## 3. Testing
At this point you are ready to test the RSI interface. Before the test, make sure that:

* You have specified the `rsi_ip_address` and `rsi_port` tags in the urdf (kuka_kr6_support/urdf/kr6r700sixx_macro_ros2_control.xacro) to correspond with the KRL files on the controller.

The next steps describe how to start external control using RSI:

* Start the driver: ```ros2 launch kuka_rsi_hw_interface startup.launch.py```

* In a new terminal: ```ros2 lifecycle set robot_manager configure```

*	Start the `KRC:\R1\Program\ros_rsi.src` program on the controller and move to the step before RSI_MOVECORR() is run
  * in T1, a warning (!!! Attention - Sensor correction goes active !!!) should be visible after reaching RSI_MOVECORR(), which is also okay
* Activate driver and controllers: ```ros2 lifecycle set robot_manager activate```
  * The hardware interface is now waiting for the robot controller to connect, the timeout for this is currently 2 seconds
* Start step RSI_MOVECORR() withing the given timeout
  * in T1 this can be done with confirming the previously described warning
  * This time the terminal where the driver is running should output **Got connection from robot**. The RSI connection is now up and running.

After this, starting an rqt joint trajectory controller (```ros2 run rqt_joint_trajectory_controller rqt_joint_trajectory_controller```) should enable moving the robot with sliders
-	This sends the trajectory in batches, which can result in a little jerky movement, so that is not a bug of the driver
