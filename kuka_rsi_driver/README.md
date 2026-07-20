ROS2 ported HW interface based on RSI communication.

This package and HW interface is heavily influenced and originated by https://github.com/ros-industrial/kuka_experimental.

`launch/startup.launch.py` supports optional robot + KL composition via the reusable
`kuka_resources/urdf/robot_with_external_axis_template.urdf.xacro` template:
- `use_external_axis:=true`
- `kl_model:=<model>` (default: `kl100_2`)
- `kl_urdf_package:=<package>` (default fallback: `kuka_kl_support`)
- `kl_prefix:=<prefix>` (default: `rail_`)

When external-axis mode is enabled and `jtc_config` is empty, the launch file selects
`config/joint_trajectory_controller_config_6_axis_kl.yaml` automatically.
