# Configuring RSI on the controller

This tutorial was tested with RSI 4.1.3 (on KSS8.6)

This guide highlights the steps needed in order to successfully configure the **RSI interface** on the controller to work with the **kuka_rsi_hardware_interface** on your PC with ROS2.

## 1. Controller network configuration

Windows runs behind the SmartHMI on the teach pad. Make sure that the **Windows interface** of the controller and the **PC with ROS** is connected to the same subnet.

1. Log in as **Expert** or **Administrator** on the teach pad and navigate to **Network configuration** (**Start-up > Network configuration > Activate advanced configuration**).
2. There should already be an interface checked out as the **Windows interface**. For example:
   * **IP**: 192.168.250.20
   * **Subnet mask**: 255.255.255.0
   * **Default gateway**: 192.168.250.20
   * **Windows interface checkbox** should be checked.
3. Minimize the SmartHMI (**Start-up > Service > Minimize HMI**).
4. Run **RSI-Network** from the Windows Start menu (**All Programs > RSI-Network**).
5. Check that the **Network - Kuka User Interface** show the Windows interface with the specified IP address.
6. Add a new IP address on another subnet (e.g. 192.168.1.20) for the **RSI interface**.
   * Select the entry **New** under **RSI Ethernet** in the tree structure and press **Edit**.
   * Enter the IP address and confirm with **OK**.
   * Close **RSI-Network** and maximize the SmartHMI.
7. Reboot the controller with a cold restart (**Shutdown > Check *Force cold start* and *Reload files* > Reboot control PC**).
8. After reboot, minimize the SmartHMI (**Start-up > Service > Minimize HMI**).
9. Run **cmd.exe** and ping the PC you want to communicate with on the same subnet (e.g. 192.168.250.xx).

If your **PC** has an IP address on the same subnet as the **Windows interface** on the controller, the controller should receive answers from the PC:
* If this is the case, add another IP address to the current PC connection (e.g. 192.168.1.xx) on the same subnet as the **RSI** interface.

## 2. KRL Files

The files included in this folder specifies the data transferred via RSI. Some of the files needs to be modified to work for your specific configuration.

##### ros_rsi_ethernet.xml
1. Edit the `IP_NUMBER` tag so that it corresponds to the IP address (192.168.1.xx) previously added for your PC.
2. Keep the `PORT` tag as it is (59152) or change it if you want to use another port.

Note that the `client_ip` and `client_port` parameters in config/rsi_config.yaml must correspond to the `IP_NUMBER`and `PORT` set in these KRL files.

##### ros_rsi.src
This should only be edited if the start position specified within the file is not desirable for your application.

##### Copy files to controller
The files **ros_rsi.rsi** and **ros_rsi.rsi.diagram** should not be edited. All files are now ready to be copied to the Kuka controller:

1. Copy the files to a USB-stick.
2. Plug it into the teach pad or controller.
3. Log in as **Expert** or **Administrator**.
4. Copy the `ros_rsi.src` file to `KRC:\R1\Program`.
5. Copy the rest of the files to `C:\KRC\ROBOTER\Config\User\Common\SensorInterface`.

## 3. Testing
At this point you are ready to test the RSI interface. Before the test, make sure that:

* You have specified the `client_ip` and `client_port` tags in the configuration file (config/rsi_config.yaml) to correspond with the KRL files on the controller.
* This IP address is available on the client machine (see Network configuration)

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
