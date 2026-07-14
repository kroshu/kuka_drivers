// Copyright 2025 KUKA Hungaria Kft.
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
#include <cmath>

#include "pluginlib/class_list_macros.hpp"

#include "kuka_drivers_core/hardware_interface_types.hpp"
#include "kuka_kss_message_handler/kuka_kss_message_handler.hpp"

namespace kuka_controllers
{

CallbackReturn KssMessageHandler::on_init()
{
  param_listener_ = std::make_shared<ParamListener>(get_node());
  params_ = param_listener_->get_params();
  return CallbackReturn::SUCCESS;
}

std::string KssMessageHandler::ComposeInterfaceName(
  const std::string & robot_prefix, const std::string & interface_group,
  const std::string & interface_name)
{
  if (robot_prefix.empty())
  {
    return interface_group + "/" + interface_name;
  }
  else
  {
    return robot_prefix + "_" + interface_group + "/" + interface_name;
  }
}

InterfaceConfig KssMessageHandler::command_interface_configuration() const
{
  InterfaceConfig config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (const auto & robot_prefix : params_.robot_prefixes)
  {
    config.names.emplace_back(ComposeInterfaceName(
      robot_prefix, hardware_interface::CONFIG_PREFIX, hardware_interface::CYCLE_TIME));
  }

  return config;
}

InterfaceConfig KssMessageHandler::state_interface_configuration() const
{
  InterfaceConfig config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  const std::vector<std::string> state_interfaces = {
    hardware_interface::CONTROL_MODE,    hardware_interface::CYCLE_TIME,
    hardware_interface::DRIVES_POWERED,  hardware_interface::EMERGENCY_STOP,
    hardware_interface::GUARD_STOP,      hardware_interface::IN_MOTION,
    hardware_interface::MOTION_POSSIBLE, hardware_interface::OPERATION_MODE,
    hardware_interface::ROBOT_STOPPED};

  for (const auto & robot_prefix : params_.robot_prefixes)
  {
    for (const auto & interface : state_interfaces)
    {
      config.names.emplace_back(
        ComposeInterfaceName(robot_prefix, hardware_interface::STATE_PREFIX, interface));
    }
  }

  return config;
}

CallbackReturn KssMessageHandler::on_configure(const rclcpp_lifecycle::State &)
{
  robot_prefixes_ = params_.robot_prefixes;
  current_statuses_.assign(robot_prefixes_.size(), kuka_driver_interfaces::msg::KssStatus{});

  status_msg_.robot_names = robot_prefixes_;
  status_msg_.statuses = current_statuses_;

  // RSI cycle time: default to 4ms, as 12 ms is not supported for iiQKA.OS2
  cycle_time_.store(static_cast<double>(kuka_driver_interfaces::msg::KssStatus::RSI_4MS));
  prev_cycle_time_ = std::numeric_limits<double>::quiet_NaN();
  cycle_time_subscription_ = get_node()->create_subscription<std_msgs::msg::UInt8>(
    "~/cycle_time", rclcpp::SystemDefaultsQoS(),
    std::bind(&KssMessageHandler::RsiCycleTimeChangedCallback, this, std::placeholders::_1));

  // Status publisher for all robots. One message contains all robot statuses.
  status_publisher_ = get_node()->create_publisher<kuka_driver_interfaces::msg::KssStatusArray>(
    "~/status", rclcpp::SystemDefaultsQoS());

  timer_ = get_node()->create_wall_timer(
    STATUS_PUBLISH_INTERVAL,
    [this]
    {
      status_msg_.statuses = current_statuses_;
      status_publisher_->publish(status_msg_);
    });

  RCLCPP_INFO(
    get_node()->get_logger(),
    "KSS message handler configured with %zu robot instance(s); cycle_time command topic is shared",
    robot_prefixes_.size());
  return CallbackReturn::SUCCESS;
}

ReturnType KssMessageHandler::update(const rclcpp::Time &, const rclcpp::Duration &)
{
  const double cycle_time = cycle_time_.load();
  const bool cycle_time_changed = std::isnan(prev_cycle_time_) || cycle_time != prev_cycle_time_;
  bool all_cycle_time_set = true;

  if (cycle_time_changed)
  {
    for (size_t idx = 0; idx < command_interfaces_.size(); ++idx)
    {
      const bool cycle_time_set = command_interfaces_[idx].set_value(cycle_time);
      all_cycle_time_set = all_cycle_time_set && cycle_time_set;
      if (!cycle_time_set)
      {
        RCLCPP_WARN_THROTTLE(
          get_node()->get_logger(), *get_node()->get_clock(), WARN_THROTTLE_DURATION_MS,
          "Failed to set cycle time command interface for robot '%s'",
          idx < robot_prefixes_.size() ? robot_prefixes_[idx].c_str() : "unknown");
      }
    }

    if (all_cycle_time_set)
    {
      prev_cycle_time_ = cycle_time;
    }
  }

  for (size_t idx = 0; idx < current_statuses_.size(); ++idx)
  {
    AssignStatusFromInterfaces(
      current_statuses_[idx], state_interfaces_, idx * STATE_INTERFACE_COUNT);
  }

  return all_cycle_time_set ? ReturnType::OK : ReturnType::ERROR;
}

void KssMessageHandler::RsiCycleTimeChangedCallback(const std_msgs::msg::UInt8::SharedPtr msg)
{
  if (
    msg->data == kuka_driver_interfaces::msg::KssStatus::RSI_4MS ||
    msg->data == kuka_driver_interfaces::msg::KssStatus::RSI_12MS)
  {
    cycle_time_.store(static_cast<double>(msg->data));
    RCLCPP_INFO(
      get_node()->get_logger(),
      "RSI cycle time changed to %s, "
      "this will be sent to the KUKA controller during activation",
      msg->data == 2   ? "12 ms"
      : msg->data == 1 ? "4 ms"
                       : "UNSPECIFIED");
  }
  else
  {
    RCLCPP_ERROR(
      get_node()->get_logger(), "'%hhu' is not a valid value for the RSI cycle time", msg->data);
  }
}

void KssMessageHandler::AssignStatusFromInterfaces(
  kuka_driver_interfaces::msg::KssStatus & status,
  const std::vector<hardware_interface::LoanedStateInterface> & state_interfaces,
  const size_t start_idx)
{
  for (const auto & [member, index] : UINT8_STATUS_FIELDS)
  {
    status.*member =
      ReadInterfaceValue<uint8_t>(state_interfaces[start_idx + index], status.*member);
  }

  for (const auto & [member, index] : BOOL_STATUS_FIELDS)
  {
    status.*member = ReadInterfaceValue<bool>(state_interfaces[start_idx + index], status.*member);
  }
}

}  // namespace kuka_controllers

PLUGINLIB_EXPORT_CLASS(
  kuka_controllers::KssMessageHandler, controller_interface::ControllerInterface)
