# from launch import LaunchDescription
# from launch.substitutions import EnvironmentVariable
# import os
# import launch_ros.actions
# import pathlib

# test_params_sim_file_name = 'test_params_sim.yaml'
# test_params_file_name = 'test_params.yaml'
# hardware_controllers_file_name = 'hardware_controllers.yaml'
# controller_joint_names_file_name = 'controller_joint_names.yaml'


# def generate_launch_description():
#     test_params_sim_file_path = Path(get_package_share_directory('the_package'), 'config', test_params_sim_file_name)
#     test_params_file_path = Path(get_package_share_directory('the_package'), 'config', test_params_file_name)

#     kuka_rsi_hw_interface_node = launch_ros.actions.Node(
#             package='kuka_rsi_hw_interface',
#             node_executable='kuka_hardware_interface',
#             output='screen',
#             parameters=[
#                 test_params_sim_file_path
#             ],
#          )    

#     return LaunchDescription([
#         kuka_rsi_hw_interface_node,
#     ])

# <?xml version="1.0" encoding="utf-8"?>
# <launch>
#     <arg name="sim" default="true" />

#     <rosparam file="$(find kuka_rsi_hw_interface)/test/test_params_sim.yaml" command="load" if="$(arg sim)"/>
#     <rosparam file="$(find kuka_rsi_hw_interface)/test/test_params.yaml" command="load" unless="$(arg sim)"/>

#     <!-- Start node without FT sensor -->
#     <node name="kuka_hardware_interface" pkg="kuka_rsi_hw_interface"
#       type="kuka_hardware_interface_node" respawn="false"
#       output="screen"
#       required="true"/>

#     <!-- Load joint controller configurations from YAML file to parameter server -->
#     <rosparam file="$(find kuka_rsi_hw_interface)/config/hardware_controllers.yaml" command="load"/>
#     <!-- Load standard kuka controller joint names from YAML file to parameter server -->
#     <rosparam file="$(find kuka_rsi_hw_interface)/config/controller_joint_names.yaml" command="load"/>

#     <!-- Load controllers -->
#     <node name="controller_spawner" pkg="controller_manager" type="spawner" respawn="false" output="screen"
#         args="position_trajectory_controller joint_state_controller --shutdown-timeout 1"/>
#     <!-- Load robot state publisher -->
#     <node name="robot_state_publisher" pkg="robot_state_publisher" type="robot_state_publisher" />

#     <!-- Load RSI simulation node -->
#     <node name='kuka_rsi_simulator' pkg='kuka_rsi_simulator' type="kuka_rsi_simulator" args="127.0.0.1 59152" if="$(arg sim)" />

# </launch>
