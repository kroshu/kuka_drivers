// Copyright 2025 Kristof Pasztor
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

#include "kuka_drivers_core/hardware_interface_types.hpp"
#include "kuka_nrt_message_handler/kuka_nrt_message_handler.hpp"

namespace kuka_controllers
{

CallbackReturn KukaNrtMessageHandler::on_init() { return CallbackReturn::SUCCESS; }

InterfaceConfig KukaNrtMessageHandler::command_interface_configuration() const
{
  InterfaceConfig config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  config.names.emplace_back(
    std::string(hardware_interface::CONFIG_PREFIX) + "/" + hardware_interface::DRIVE_STATE);
  config.names.emplace_back(
    std::string(hardware_interface::CONFIG_PREFIX) + "/" + hardware_interface::CYCLE_TIME);
  return config;
}

InterfaceConfig KukaNrtMessageHandler::state_interface_configuration() const
{
  return InterfaceConfig{controller_interface::interface_configuration_type::NONE};
}

CallbackReturn KukaNrtMessageHandler::on_configure(const rclcpp_lifecycle::State &)
{
  // Drive state
  drive_state_ = 1.0;
  drive_state_subscription_ = get_node()->create_subscription<std_msgs::msg::Bool>(
    "~/drive_state", rclcpp::SystemDefaultsQoS(),
    std::bind(&KukaNrtMessageHandler::DriveStateChangedCallback, this, std::placeholders::_1));

  // RSI cycle time
  cycle_time_ = 1.0;
  cycle_time_subscription_ =
    get_node()->create_subscription<kuka_driver_interfaces::msg::RsiCycleTime>(
      "~/cycle_time", rclcpp::SystemDefaultsQoS(),
      std::bind(&KukaNrtMessageHandler::RsiCycleTimeChangedCallback, this, std::placeholders::_1));

  RCLCPP_INFO(get_node()->get_logger(), "Non-real time message handler configured");
  return CallbackReturn::SUCCESS;
}

CallbackReturn KukaNrtMessageHandler::on_activate(const rclcpp_lifecycle::State &)
{
  return CallbackReturn::SUCCESS;
}

CallbackReturn KukaNrtMessageHandler::on_deactivate(const rclcpp_lifecycle::State &)
{
  return CallbackReturn::SUCCESS;
}

ReturnType KukaNrtMessageHandler::update(const rclcpp::Time &, const rclcpp::Duration &)
{
  bool drive_state_set = command_interfaces_[0].set_value(drive_state_);
  bool cycle_time_set = command_interfaces_[1].set_value(cycle_time_);
  return drive_state_set && cycle_time_set ? ReturnType::OK : ReturnType::ERROR;
}

void KukaNrtMessageHandler::DriveStateChangedCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
  drive_state_ = msg->data ? 1.0 : 0.0;
}

void KukaNrtMessageHandler::RsiCycleTimeChangedCallback(
  const kuka_driver_interfaces::msg::RsiCycleTime::SharedPtr msg)
{
  if (
    msg->cycle_time == kuka_driver_interfaces::msg::RsiCycleTime::RSI_4MS ||
    msg->cycle_time == kuka_driver_interfaces::msg::RsiCycleTime::RSI_12MS)
  {
    cycle_time_ = static_cast<double>(msg->cycle_time);
  }
  else
  {
    RCLCPP_ERROR(
      get_node()->get_logger(), "'%u' is not a valid value for the RSI cycle time.", msg->cycle_time);
  }
}

}  // namespace kuka_controllers

PLUGINLIB_EXPORT_CLASS(
  kuka_controllers::KukaNrtMessageHandler, controller_interface::ControllerInterface)
