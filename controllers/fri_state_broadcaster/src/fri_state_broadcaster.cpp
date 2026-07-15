// Copyright 2022 KUKA Hungaria Kft.
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
#include "fri_state_broadcaster/fri_state_broadcaster.hpp"

namespace kuka_controllers
{

controller_interface::CallbackReturn FRIStateBroadcaster::on_init()
{
  param_listener_ = std::make_shared<ParamListener>(get_node());
  params_ = param_listener_->get_params();
  return controller_interface::CallbackReturn::SUCCESS;
}

std::string FRIStateBroadcaster::ComposeInterfaceName(
  const std::string & robot_prefix, const std::string & interface_name)
{
  if (robot_prefix.empty())
  {
    return std::string(hardware_interface::FRI_STATE_PREFIX) + "/" + interface_name;
  }
  else
  {
    return robot_prefix + "_" + std::string(hardware_interface::FRI_STATE_PREFIX) + "/" +
           interface_name;
  }
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

  const std::vector<std::string> state_interfaces = {
    hardware_interface::SESSION_STATE,      hardware_interface::CONNECTION_QUALITY,
    hardware_interface::SAFETY_STATE,       hardware_interface::COMMAND_MODE,
    hardware_interface::CONTROL_MODE,       hardware_interface::OPERATION_MODE,
    hardware_interface::DRIVE_STATE,        hardware_interface::OVERLAY_TYPE,
    hardware_interface::TRACKING_PERFORMANCE};

  for (const auto & robot_prefix : params_.robot_prefixes)
  {
    for (const auto & interface : state_interfaces)
    {
      config.names.emplace_back(ComposeInterfaceName(robot_prefix, interface));
    }
  }

  return config;
}

controller_interface::CallbackReturn FRIStateBroadcaster::on_configure(
  const rclcpp_lifecycle::State &)
{
  robot_prefixes_ = params_.robot_prefixes;
  current_states_.assign(robot_prefixes_.size(), kuka_driver_interfaces::msg::FRIState{});

  state_msg_.robot_names = robot_prefixes_;
  state_msg_.states = current_states_;

  // Single publisher for all robots. One message contains all robot states.
  state_publisher_ = get_node()->create_publisher<kuka_driver_interfaces::msg::FRIStateArray>(
    "~/fri_state", rclcpp::SystemDefaultsQoS());

  RCLCPP_INFO(
    get_node()->get_logger(),
    "FRI state broadcaster configured with %zu robot instance(s)",
    robot_prefixes_.size());

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

void FRIStateBroadcaster::AssignStateFromInterfaces(
  kuka_driver_interfaces::msg::FRIState & state,
  const std::vector<hardware_interface::LoanedStateInterface> & state_interfaces,
  const size_t start_idx)
{
  state.session_state = static_cast<int32_t>(
    state_interfaces[start_idx].get_optional().value_or(state.session_state));

  state.connection_quality = static_cast<int32_t>(
    state_interfaces[start_idx + 1].get_optional().value_or(state.connection_quality));

  state.safety_state = static_cast<int32_t>(
    state_interfaces[start_idx + 2].get_optional().value_or(state.safety_state));

  state.command_mode = static_cast<int32_t>(
    state_interfaces[start_idx + 3].get_optional().value_or(state.command_mode));

  state.control_mode = static_cast<int32_t>(
    state_interfaces[start_idx + 4].get_optional().value_or(state.control_mode));

  state.operation_mode = static_cast<int32_t>(
    state_interfaces[start_idx + 5].get_optional().value_or(state.operation_mode));

  state.drive_state = static_cast<int32_t>(
    state_interfaces[start_idx + 6].get_optional().value_or(state.drive_state));

  state.overlay_type = static_cast<int32_t>(
    state_interfaces[start_idx + 7].get_optional().value_or(state.overlay_type));

  state.tracking_performance = static_cast<double>(
    state_interfaces[start_idx + 8].get_optional().value_or(state.tracking_performance));
}

controller_interface::return_type FRIStateBroadcaster::update(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  for (size_t idx = 0; idx < current_states_.size(); ++idx)
  {
    AssignStateFromInterfaces(
      current_states_[idx], state_interfaces_, idx * STATE_INTERFACE_COUNT);
  }

  if (counter_++ == 10)
  {
    state_msg_.states = current_states_;
    state_publisher_->publish(state_msg_);
    counter_ = 0;
  }

  return controller_interface::return_type::OK;
}

}  // namespace kuka_controllers

PLUGINLIB_EXPORT_CLASS(
  kuka_controllers::FRIStateBroadcaster, controller_interface::ControllerInterface)

