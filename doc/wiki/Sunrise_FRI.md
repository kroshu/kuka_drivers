### Usage

Start the launch file in the package to start all required nodes:

**`ros2 launch kuka_sunrise_fri_driver startup.launch.py`**

After all components have started successfully, the system needs to be configured and activated to start external control:

**`ros2 lifecycle set robot_manager configure`**


**`ros2 lifecycle set robot_manager activate`**