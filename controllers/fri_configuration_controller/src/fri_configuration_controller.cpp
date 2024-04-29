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

#include "fri_configuration_controller/fri_configuration_controller.hpp"

namespace kuka_controllers
{
controller_interface::CallbackReturn FRIConfigurationController::on_init()
{
  auto callback = [this](const kuka_driver_interfaces::msg::FriConfiguration::SharedPtr msg)
  {
    receive_multiplier_ = msg->receive_multiplier;
    send_period_ms_ = msg->send_period_ms;
  };
  fri_config_sub_ = get_node()->create_subscription<kuka_driver_interfaces::msg::FriConfiguration>(
    "~/set_fri_config", rclcpp::SystemDefaultsQoS(), callback);
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
FRIConfigurationController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  config.names.emplace_back(
    std::string(hardware_interface::CONFIG_PREFIX) + "/" + hardware_interface::RECEIVE_MULTIPLIER);
  config.names.emplace_back(
    std::string(hardware_interface::CONFIG_PREFIX) + "/" + hardware_interface::SEND_PERIOD);
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
  command_interfaces_[0].set_value(receive_multiplier_);
  command_interfaces_[1].set_value(send_period_ms_);

  return controller_interface::return_type::OK;
}

}  // namespace kuka_controllers

PLUGINLIB_EXPORT_CLASS(
  kuka_controllers::FRIConfigurationController, controller_interface::ControllerInterface)
