// Copyright 2022 Áron Svastits
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
// limitations under the License.-------------------------------------------------------------------

#include "kuka_controllers/robot_state_broadcaster.hpp"

namespace kuka_controllers
{
controller_interface::CallbackReturn RobotStateBroadcaster::on_init()
{
  // TODO(Svastits): choose appropriate QoS settings for late joiners
  fri_state_publisher_ = get_node()->create_publisher<std_msgs::msg::Int32>(
    "fri_state", rclcpp::SystemDefaultsQoS());
  connection_quality_publisher_ = get_node()->create_publisher<std_msgs::msg::Int32>(
    "connection_quality", rclcpp::SystemDefaultsQoS());
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration RobotStateBroadcaster::command_interface_configuration()
const
{
  return controller_interface::InterfaceConfiguration{controller_interface::
    interface_configuration_type::NONE};
}

controller_interface::InterfaceConfiguration RobotStateBroadcaster::state_interface_configuration()
const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  config.names.push_back("state/fri_state");
  config.names.push_back("state/connection_quality");
  return config;
}

controller_interface::CallbackReturn
RobotStateBroadcaster::on_configure(const rclcpp_lifecycle::State &)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
RobotStateBroadcaster::on_activate(const rclcpp_lifecycle::State &)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
RobotStateBroadcaster::on_deactivate(const rclcpp_lifecycle::State &)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type RobotStateBroadcaster::update(
  const rclcpp::Time &,
  const rclcpp::Duration &)
{
  // TODO(Svastits): create custom state msg type and publish all robot state info
  //   with given frequency -> measure additional system overload
  state_msg_.data = state_interfaces_[0].get_value();
  if (fri_state_ != state_msg_.data) {
    fri_state_ = state_msg_.data;
    fri_state_publisher_->publish(state_msg_);
  }

  state_msg_.data = state_interfaces_[1].get_value();
  if (connection_quality_ != state_msg_.data) {
    connection_quality_ = state_msg_.data;
    connection_quality_publisher_->publish(state_msg_);
  }
  return controller_interface::return_type::OK;
}

}  // namespace kuka_controllers

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  kuka_controllers::RobotStateBroadcaster,
  controller_interface::ControllerInterface)
