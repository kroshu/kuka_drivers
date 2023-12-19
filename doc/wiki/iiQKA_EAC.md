## iiQKA driver (ECI)

### Setup

The driver depends on some KUKA-specific packages, which are only available with the real robot, but setting the MOCK_HW_ONLY flag in the hardware_interface enables the usage of the driver in a simulated way, so that motion planning problems can be tried out with the same components running. By default, the mock libraries are used, this can be changed in the cmake file by setting MOCK_KUKA_LIBS to FALSE before building.

The IP addresses of the client machine and controller must be given in the *config/driver_config.yaml* configuration file. A rebuild is not needed after the changes, but the file has to be modified before starting the nodes. The control mode of the robot can also be modified in the same configuration file: you can choose either 1 (POSITION_CONTROL), 2 (JOINT_IMPEDANCE_CONTROL) or 4 (JOINT_TORQUE_CONTROL). This also sets the control_mode parameter of the `robot_manager` node.

Besides, the setting of scheduling priorities must be allowed for your user (extend /etc/security/limits.conf with "username	 -	 rtprio		 98" and restart) to enable real-time performance.

If the ExternalAPI.Control toolbox is installed on the robot, it is ready for external control. You only have to release SPOC on the Teach Pendant, no other setup is needed on the controller.

### Usage

The usage of the driver is quite simple, one has to start the launch file in the package to start all required nodes:

**`ros2 launch kuka_iiqka_eac_driver startup.launch.py`**

After all components have started successfully, the system needs to be configured and activated to start external control:

**`ros2 lifecycle set robot_manager configure`**


**`ros2 lifecycle set robot_manager activate`**

On successful activation the robot controller and the driver start communication with a 4 ms cycle time, and it is possible to move the robot through the joint trajectory controller.

To stop external control, the components have to be deactivated with **`ros2 lifecycle set robot_manager deactivate`**

BEWARE, that this is a non-realtime process including lifecycle management, so the connection is not terminated immediately, in cases where an abrupt stop is needed, the safety stop of the Teach Pendant should be used!

It is also possible to use different controllers with some modifications in the launch and yaml files (for example ForwardCommandController, which forwards the commands send to a ROS2 topic towards the robot). In these cases, one has to make sure, that the commands sent to the robot are close to the current position, otherwise the machine protection will stop the robot movement.