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

#include <chrono>

#include "pluginlib/class_list_macros.hpp"

#include "kuka_drivers_core/hardware_interface_types.hpp"
#include "kuka_nrt_message_handler/kuka_nrt_message_handler.hpp"

namespace kuka_controllers
{

InterfaceConfig NrtMessageHandler::command_interface_configuration() const
{
  InterfaceConfig config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  config.names.emplace_back(
    std::string{hardware_interface::CONFIG_PREFIX} + "/" + hardware_interface::DRIVE_STATE);
  config.names.emplace_back(
    std::string{hardware_interface::CONFIG_PREFIX} + "/" + hardware_interface::CYCLE_TIME);
  return config;
}

InterfaceConfig NrtMessageHandler::state_interface_configuration() const
{
  InterfaceConfig config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  const std::vector<std::string> state_interfaces = {
    hardware_interface::CONTROL_MODE,    hardware_interface::CYCLE_TIME,
    hardware_interface::DRIVES_POWERED,  hardware_interface::EMERGENCY_STOP,
    hardware_interface::GUARD_STOP,      hardware_interface::IN_MOTION,
    hardware_interface::MOTION_POSSIBLE, hardware_interface::OPERATION_MODE,
    hardware_interface::ROBOT_STOPPED};

  for (const auto & interface : state_interfaces)
  {
    config.names.emplace_back(std::string{hardware_interface::STATE_PREFIX} + "/" + interface);
  }

  return config;
}

CallbackReturn NrtMessageHandler::on_configure(const rclcpp_lifecycle::State &)
{
  // Drive state
  drive_state_ = 1.0;
  drive_state_subscription_ = get_node()->create_subscription<std_msgs::msg::Bool>(
    "~/drive_state", rclcpp::SystemDefaultsQoS(),
    [this](const std_msgs::msg::Bool::SharedPtr msg) { drive_state_ = msg->data ? 1.0 : 0.0; });

  // RSI cycle time
  cycle_time_ = static_cast<double>(kuka_driver_interfaces::msg::KssStatus::RSI_12MS);
  cycle_time_subscription_ = get_node()->create_subscription<std_msgs::msg::UInt8>(
    "~/cycle_time", rclcpp::SystemDefaultsQoS(),
    std::bind(&NrtMessageHandler::RsiCycleTimeChangedCallback, this, std::placeholders::_1));

  // Status
  status_publisher_ = get_node()->create_publisher<kuka_driver_interfaces::msg::KssStatus>(
    "~/status", rclcpp::SystemDefaultsQoS());
  timer_ = get_node()->create_wall_timer(
    STATUS_PUBLISH_INTERVAL,
    [this]
    {
      if (status_.StatusChanged())
      {
        status_.UpdateMessage();
        status_publisher_->publish(status_.GetMessage());
      }
    });

  RCLCPP_INFO(get_node()->get_logger(), "Non-real time message handler configured");
  return CallbackReturn::SUCCESS;
}

ReturnType NrtMessageHandler::update(const rclcpp::Time &, const rclcpp::Duration &)
{
  bool drive_state_set = command_interfaces_[0].set_value(drive_state_);
  bool cycle_time_set = command_interfaces_[1].set_value(cycle_time_);
  status_ = state_interfaces_;
  return drive_state_set && cycle_time_set ? ReturnType::OK : ReturnType::ERROR;
}

void NrtMessageHandler::RsiCycleTimeChangedCallback(const std_msgs::msg::UInt8::SharedPtr msg)
{
  if (
    msg->data == kuka_driver_interfaces::msg::KssStatus::RSI_4MS ||
    msg->data == kuka_driver_interfaces::msg::KssStatus::RSI_12MS)
  {
    cycle_time_ = static_cast<double>(msg->data);
  }
  else
  {
    RCLCPP_ERROR(
      get_node()->get_logger(), "'%hhu' is not a valid value for the RSI cycle time", msg->data);
  }
}

NrtMessageHandler::Status & NrtMessageHandler::Status::operator=(
  const std::vector<hardware_interface::LoanedStateInterface> & state_interfaces)
{
  for (const auto & [value_ptr, idx] : UINT8_MAPPINGS)
  {
    *value_ptr = static_cast<uint8_t>(
      state_interfaces[idx].get_optional().value_or(static_cast<double>(*value_ptr)));
  }

  for (const auto & [value_ptr, idx] : BOOL_MAPPINGS)
  {
    *value_ptr = static_cast<bool>(
      state_interfaces[idx].get_optional().value_or(static_cast<double>(*value_ptr)));
  }

  return *this;
}

}  // namespace kuka_controllers

PLUGINLIB_EXPORT_CLASS(
  kuka_controllers::NrtMessageHandler, controller_interface::ControllerInterface)
