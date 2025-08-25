// Copyright 2022 Aron Svastits
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

#include "kuka_drivers_core/hardware_interface_types.hpp"

#include "fri_state_broadcaster/fri_state_broadcaster.hpp"

namespace kuka_controllers
{
controller_interface::CallbackReturn FRIStateBroadcaster::on_init()
{
  robot_state_publisher_ = get_node()->create_publisher<kuka_driver_interfaces::msg::FRIState>(
    "~/fri_state", rclcpp::SystemDefaultsQoS());
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration FRIStateBroadcaster::command_interface_configuration()
  const
{
  return controller_interface::InterfaceConfiguration{
    controller_interface::interface_configuration_type::NONE};
}

controller_interface::InterfaceConfiguration FRIStateBroadcaster::state_interface_configuration()
  const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  config.names.emplace_back(
    std::string(hardware_interface::FRI_STATE_PREFIX) + "/" + hardware_interface::SESSION_STATE);
  config.names.emplace_back(
    std::string(hardware_interface::FRI_STATE_PREFIX) + "/" +
    hardware_interface::CONNECTION_QUALITY);
  config.names.emplace_back(
    std::string(hardware_interface::FRI_STATE_PREFIX) + "/" + hardware_interface::SAFETY_STATE);
  config.names.emplace_back(
    std::string(hardware_interface::FRI_STATE_PREFIX) + "/" + hardware_interface::COMMAND_MODE);
  config.names.emplace_back(
    std::string(hardware_interface::FRI_STATE_PREFIX) + "/" + hardware_interface::CONTROL_MODE);
  config.names.emplace_back(
    std::string(hardware_interface::FRI_STATE_PREFIX) + "/" + hardware_interface::OPERATION_MODE);
  config.names.emplace_back(
    std::string(hardware_interface::FRI_STATE_PREFIX) + "/" + hardware_interface::DRIVE_STATE);
  config.names.emplace_back(
    std::string(hardware_interface::FRI_STATE_PREFIX) + "/" + hardware_interface::OVERLAY_TYPE);
  config.names.emplace_back(
    std::string(hardware_interface::FRI_STATE_PREFIX) + "/" +
    hardware_interface::TRACKING_PERFORMANCE);
  return config;
}

controller_interface::CallbackReturn FRIStateBroadcaster::on_configure(
  const rclcpp_lifecycle::State &)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn FRIStateBroadcaster::on_activate(
  const rclcpp_lifecycle::State &)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn FRIStateBroadcaster::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type FRIStateBroadcaster::update(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  auto session_state =
    static_cast<int32_t>(state_interfaces_[0].get_optional().value_or(state_msg_.session_state));
  if (session_state != state_msg_.session_state)
  {
    state_msg_.session_state = session_state;  // Update session state if it has changed
  }

  auto connection_quality = static_cast<int32_t>(
    state_interfaces_[1].get_optional().value_or(state_msg_.connection_quality));
  if (connection_quality != state_msg_.connection_quality)
  {
    state_msg_.connection_quality =
      connection_quality;  // Update connection quality if it has changed
  }

  auto safety_state =
    static_cast<int32_t>(state_interfaces_[2].get_optional().value_or(state_msg_.safety_state));
  if (safety_state != state_msg_.safety_state)
  {
    state_msg_.safety_state = safety_state;  // Update safety state if it has changed
  }

  auto command_mode =
    static_cast<int32_t>(state_interfaces_[3].get_optional().value_or(state_msg_.command_mode));
  if (command_mode != state_msg_.command_mode)
  {
    state_msg_.command_mode = command_mode;  // Update command mode if it has changed
  }

  auto control_mode =
    static_cast<int32_t>(state_interfaces_[4].get_optional().value_or(state_msg_.control_mode));
  if (control_mode != state_msg_.control_mode)
  {
    state_msg_.control_mode = control_mode;  // Update control mode state if it has changed
  }

  auto operation_mode =
    static_cast<int32_t>(state_interfaces_[5].get_optional().value_or(state_msg_.operation_mode));
  if (operation_mode != state_msg_.operation_mode)
  {
    state_msg_.operation_mode = operation_mode;  // Update operation mode if it has changed
  }

  auto drive_state =
    static_cast<int32_t>(state_interfaces_[6].get_optional().value_or(state_msg_.drive_state));
  if (drive_state != state_msg_.drive_state)
  {
    state_msg_.drive_state = drive_state;  // Update drive state if it has changed
  }

  auto overlay_type =
    static_cast<int32_t>(state_interfaces_[7].get_optional().value_or(state_msg_.overlay_type));
  if (overlay_type != state_msg_.overlay_type)
  {
    state_msg_.overlay_type = overlay_type;  // Update overlay type if it has changed
  }

  auto tracking_performance = static_cast<int32_t>(
    state_interfaces_[8].get_optional().value_or(state_msg_.tracking_performance));
  if (tracking_performance != state_msg_.tracking_performance)
  {
    state_msg_.tracking_performance =
      tracking_performance;  // Update tracking performance if it has changed
  }

  if (counter_++ == 10)
  {
    robot_state_publisher_->publish(state_msg_);
    counter_ = 0;
  }

  return controller_interface::return_type::OK;
}

}  // namespace kuka_controllers

PLUGINLIB_EXPORT_CLASS(
  kuka_controllers::FRIStateBroadcaster, controller_interface::ControllerInterface)
