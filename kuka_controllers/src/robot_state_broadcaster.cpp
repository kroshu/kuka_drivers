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
  robot_state_publisher_ = get_node()->create_publisher<kuka_sunrise_interfaces::msg::RobotState>(
    "robot_state", rclcpp::SystemDefaultsQoS());
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
RobotStateBroadcaster::command_interface_configuration()
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
  // TODO(Svastits): measure additional system overload, limit rate?
  state_msg_.fri_state = state_interfaces_[0].get_value();
  state_msg_.connection_quality = state_interfaces_[1].get_value();

  if (counter_++ == 10) {
    robot_state_publisher_->publish(state_msg_);
    counter_ = 0;
  }

  return controller_interface::return_type::OK;
}

}  // namespace kuka_controllers

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  kuka_controllers::RobotStateBroadcaster,
  controller_interface::ControllerInterface)
