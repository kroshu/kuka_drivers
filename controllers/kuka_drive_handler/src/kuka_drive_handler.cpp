// Copyright 2025 Kristóf Pásztor
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

#include "pluginlib/class_list_macros.hpp"

#include "kuka_drive_handler/kuka_drive_handler.hpp"
#include "kuka_drivers_core/hardware_interface_types.hpp"

namespace kuka_controllers
{

CR KukaDriveHandler::on_init() { return CR::SUCCESS; }

IC KukaDriveHandler::command_interface_configuration() const
{
  IC config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  config.names.emplace_back(
    std::string(hardware_interface::CONFIG_PREFIX) + "/" + hardware_interface::DRIVE_STATE);
  return config;
}

IC KukaDriveHandler::state_interface_configuration() const
{
  return IC{controller_interface::interface_configuration_type::NONE};
}

CR KukaDriveHandler::on_configure(const rclcpp_lifecycle::State &)
{
  drive_state_ = 1.0;
  auto CR = [this](const std_msgs::msg::Bool::SharedPtr msg)
  { drive_state_ = msg->data ? 1.0 : 0.0; };
  drive_state_subscription_ = get_node()->create_subscription<std_msgs::msg::Bool>(
    "~/drive_state", rclcpp::SystemDefaultsQoS(), CR);
  RCLCPP_INFO(get_node()->get_logger(), "Drive handler configured");
  return CR::SUCCESS;
}

CR KukaDriveHandler::on_activate(const rclcpp_lifecycle::State &) { return CR::SUCCESS; }

CR KukaDriveHandler::on_deactivate(const rclcpp_lifecycle::State &) { return CR::SUCCESS; }

RT KukaDriveHandler::update(const rclcpp::Time &, const rclcpp::Duration &)
{
  bool success = command_interfaces_[0].set_value(drive_state_);
  return success ? RT::OK : RT::ERROR;
}

}  // namespace kuka_controllers

PLUGINLIB_EXPORT_CLASS(
  kuka_controllers::KukaDriveHandler, controller_interface::ControllerInterface)
