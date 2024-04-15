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
  auto callback = [this](
                    kuka_driver_interfaces::srv::SetInt::Request::SharedPtr request,
                    kuka_driver_interfaces::srv::SetInt::Response::SharedPtr response)
  {
    resend_multiplier_ = true;
    receive_multiplier_ = request->data;
    response->success = true;
  };
  receive_multiplier_service_ = get_node()->create_service<kuka_driver_interfaces::srv::SetInt>(
    "~/set_receive_multiplier", callback);
  // TODO(Svastits): create service to get multiplier changes (or perpaps
  // parameter??)
  //   and set resend_multiplier_ to true in the callback
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
FRIConfigurationController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  config.names.emplace_back(
    std::string(hardware_interface::CONFIG_PREFIX) + "/" + hardware_interface::RECEIVE_MULTIPLIER);
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
  if (resend_multiplier_)
  {
    RCLCPP_INFO(
      get_node()->get_logger(), "Changing receive multiplier of hardware interface to %i",
      receive_multiplier_);
    command_interfaces_[0].set_value(receive_multiplier_);
    resend_multiplier_ = false;
  }
  return controller_interface::return_type::OK;
}

}  // namespace kuka_controllers

PLUGINLIB_EXPORT_CLASS(
  kuka_controllers::FRIConfigurationController, controller_interface::ControllerInterface)
