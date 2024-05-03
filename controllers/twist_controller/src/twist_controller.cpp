// Copyright 2023 Kristófi Mihály
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "twist_controller.hpp"

namespace kuka_controllers
{
controller_interface::CallbackReturn TwistController::on_init()
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration TwistController::
command_interface_configuration()
const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  // config.names.emplace_back("twist commands");
  config.names.emplace_back("twist/linear/x");
  config.names.emplace_back("twist/linear/y");
  config.names.emplace_back("twist/linear/z");
  config.names.emplace_back("twist/angular/x");
  config.names.emplace_back("twist/angular/y");
  config.names.emplace_back("twist/angular/z");
  return config;
}

controller_interface::InterfaceConfiguration TwistController::state_interface_configuration()
const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  config.names.emplace_back("twist/linear/x");
  config.names.emplace_back("twist/linear/y");
  config.names.emplace_back("twist/linear/z");
  config.names.emplace_back("twist/angular/x");
  config.names.emplace_back("twist/angular/y");
  config.names.emplace_back("twist/angular/z");
  config.names.emplace_back("Pose/position/x");
  config.names.emplace_back("Pose/position/y");
  config.names.emplace_back("Pose/position/z");
  config.names.emplace_back("Pose/orientation/x");
  config.names.emplace_back("Pose/orientation/y");
  config.names.emplace_back("Pose/orientation/z");
  config.names.emplace_back("Pose/orientation/w");

  return config;
}

controller_interface::CallbackReturn
TwistController::on_configure(const rclcpp_lifecycle::State &)
{
  twist_subscriber_ = get_node()->create_subscription<geometry_msgs::msg::Twist>(
    "twist_controller/twist_cmd", rclcpp::SystemDefaultsQoS(),
    [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
        last_command_msg_ .linear.x= msg->linear.x;
        last_command_msg_ .linear.y= msg->linear.y;
        last_command_msg_ .linear.z= msg->linear.z;
        last_command_msg_ .angular.x= msg->angular.x;
        last_command_msg_ .angular.y= msg->angular.y;
        last_command_msg_ .angular.z= msg->angular.z;
    });
  return controller_interface::CallbackReturn::SUCCESS;

}

controller_interface::CallbackReturn
TwistController::on_activate(const rclcpp_lifecycle::State &)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
TwistController::on_deactivate(const rclcpp_lifecycle::State &)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type TwistController::update(
  const rclcpp::Time &,
  const rclcpp::Duration &)
{


    command_interfaces_[0].set_value(last_command_msg_ .linear.x);
    command_interfaces_[1].set_value(last_command_msg_ .linear.y);
    command_interfaces_[2].set_value(last_command_msg_ .linear.z);
    command_interfaces_[3].set_value(last_command_msg_ .angular.x);
    command_interfaces_[4].set_value(last_command_msg_ .angular.y);
    command_interfaces_[5].set_value(last_command_msg_ .angular.z);
       
    static tf2_ros::TransformBroadcaster tf_broadcaster(get_node());

    geometry_msgs::msg::TransformStamped transformStamped;
    //transformStamped.header.stamp=Time::now();
    transformStamped.header.frame_id="odom";
    transformStamped.child_frame_id="generic_mr_body_dummy";
    transformStamped.transform.translation.x=state_interfaces_[6].get_value();
    transformStamped.transform.translation.y=state_interfaces_[7].get_value();
    transformStamped.transform.translation.y=state_interfaces_[8].get_value();
   
    transformStamped.transform.rotation.x=state_interfaces_[9].get_value();
    transformStamped.transform.rotation.y=state_interfaces_[10].get_value();
    transformStamped.transform.rotation.z=state_interfaces_[11].get_value();
    transformStamped.transform.rotation.w=state_interfaces_[12].get_value(); 
    
    tf2::Quaternion q;
    q.setRPY(0,0,state_interfaces_[8].get_value());
    tf_broadcaster.sendTransform(transformStamped);
  return controller_interface::return_type::OK;
}

}  // namespace kuka_controllers

PLUGINLIB_EXPORT_CLASS(
  kuka_controllers::TwistController,
  controller_interface::ControllerInterface)
