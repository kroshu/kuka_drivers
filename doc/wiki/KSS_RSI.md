## KUKA KSS driver (RSI)

Another project in the repo centers on the development of a ROS2 driver for KSS robots through Robot Sensor Interface (RSI). It is in an experimental state, with only joint angle states and commands available. The guide to set up this driver on a real robot can be found in kuka_kss_rsi_driver\krl for both KCR4 and KRC5 controllers.

### Simulation

To try out the driver with an open-loop simulation the driver and the kuka_rsi_simulator must be started, (at first only a "collapsed" robot will be visible in rviz):

**`ros2 launch kuka_kss_rsi_driver startup_with_rviz.launch.py`**

**`ros2 launch kuka_rsi_simulator kuka_rsi_simulator_launch.py`**

After all components have started successfully, the system needs to be configured and activated to start the simulation, the robot will be visible in rviz after activation:

**`ros2 lifecycle set robot_manager configure`**

**`ros2 lifecycle set robot_manager activate`**