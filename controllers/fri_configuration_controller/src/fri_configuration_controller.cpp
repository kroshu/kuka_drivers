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

#include "fri_configuration_controller/fri_configuration_controller.hpp"
#include "kuka_drivers_core/hardware_interface_types.hpp"

namespace kuka_controllers
{
controller_interface::CallbackReturn FRIConfigurationController::on_init()
{
  param_listener_ = std::make_shared<ParamListener>(get_node());
  params_ = param_listener_->get_params();

  auto callback = [this](const kuka_driver_interfaces::msg::FriConfiguration::SharedPtr msg)
  {
    receive_multiplier_ = msg->receive_multiplier;
    send_period_ms_ = msg->send_period_ms;
  };
  fri_config_sub_ = get_node()->create_subscription<kuka_driver_interfaces::msg::FriConfiguration>(
    "~/set_fri_config", rclcpp::SystemDefaultsQoS(), callback);
  return controller_interface::CallbackReturn::SUCCESS;
}

std::string FRIConfigurationController::ComposeInterfaceName(
  const std::string & robot_prefix, const std::string & interface_name)
{
  if (robot_prefix.empty())
  {
    return std::string(hardware_interface::CONFIG_PREFIX) + "/" + interface_name;
  }
  else
  {
    return robot_prefix + "_" + std::string(hardware_interface::CONFIG_PREFIX) + "/" +
           interface_name;
  }
}

controller_interface::InterfaceConfiguration
FRIConfigurationController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (const auto & robot_prefix : params_.robot_prefixes)
  {
    config.names.emplace_back(
      ComposeInterfaceName(robot_prefix, hardware_interface::RECEIVE_MULTIPLIER));
    config.names.emplace_back(ComposeInterfaceName(robot_prefix, hardware_interface::SEND_PERIOD));
  }

  return config;
}

controller_interface::InterfaceConfiguration
FRIConfigurationController::state_interface_configuration() const
{
  return controller_interface::InterfaceConfiguration{
    controller_interface::interface_configuration_type::NONE};
}

controller_interface::CallbackReturn FRIConfigurationController::on_configure(
  const rclcpp_lifecycle::State &)
{
  robot_prefixes_ = params_.robot_prefixes;
  RCLCPP_INFO(
    get_node()->get_logger(), "FRI configuration controller configured with %zu robot instance(s)",
    robot_prefixes_.size());
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn FRIConfigurationController::on_activate(
  const rclcpp_lifecycle::State &)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn FRIConfigurationController::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type FRIConfigurationController::update(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  // TODO(Svastits): disable changes if HWIF is active
  // Set the same FRI configuration values for all robots
  bool all_set = true;
  for (size_t idx = 0; idx < robot_prefixes_.size(); ++idx)
  {
    size_t cmd_idx = idx * 2;  // Each robot has 2 command interfaces
    if (!command_interfaces_[cmd_idx].set_value(receive_multiplier_))
    {
      RCLCPP_WARN(
        get_node()->get_logger(), "Failed to set receive multiplier for robot '%s'",
        robot_prefixes_[idx].c_str());
      all_set = false;
    }
    if (!command_interfaces_[cmd_idx + 1].set_value(send_period_ms_))
    {
      RCLCPP_WARN(
        get_node()->get_logger(), "Failed to set send period for robot '%s'",
        robot_prefixes_[idx].c_str());
      all_set = false;
    }
  }

  return all_set ? controller_interface::return_type::OK : controller_interface::return_type::ERROR;
}

}  // namespace kuka_controllers

PLUGINLIB_EXPORT_CLASS(
  kuka_controllers::FRIConfigurationController, controller_interface::ControllerInterface)
