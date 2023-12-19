## KUKA KSS driver (RSI)

### Setup

See instructions for [KRC4](../../kuka_kss_rsi_driver/krl/README_KRC4.md) and [KRC5](../../kuka_kss_rsi_driver/krl/README_KRC4.md).

### Usage

Start the launch file in the package to start all required nodes:

**`ros2 launch kuka_kss_rsi_driver startup.launch.py`**

After all components have started successfully, the system needs to be configured and activated to start external control:

**`ros2 lifecycle set robot_manager configure`**

**`ros2 lifecycle set robot_manager activate`**


### Simulation

To try out the driver with an open-loop simulation the driver and the `kuka_rsi_simulator` must be started, (before activation only a "collapsed" robot will be visible in `rviz`):

**`ros2 launch kuka_kss_rsi_driver startup_with_rviz.launch.py`**

**`ros2 launch kuka_rsi_simulator kuka_rsi_simulator_launch.py`**

After all components have started successfully, the system needs to be configured and activated to start the simulation, the robot will be visible in rviz after activation:

**`ros2 lifecycle set robot_manager configure`**

**`ros2 lifecycle set robot_manager activate`**