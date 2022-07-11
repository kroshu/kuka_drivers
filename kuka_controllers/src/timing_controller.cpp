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

#include "kuka_controllers/timing_controller.hpp"

namespace kuka_controllers
{
controller_interface::CallbackReturn TimingController::on_init()
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration TimingController::command_interface_configuration()
const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  config.names.push_back("timing/receive_multiplier");
  return config;
}

controller_interface::InterfaceConfiguration TimingController::state_interface_configuration() const
{
  // TODO(Svastits): create service to get multiplier changes (or perpaps parameter??)
  //   and set resend_multiplier_ to true in the callback
  return controller_interface::InterfaceConfiguration{controller_interface::
    interface_configuration_type::NONE};
}

controller_interface::CallbackReturn
TimingController::on_configure(const rclcpp_lifecycle::State &)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
TimingController::on_activate(const rclcpp_lifecycle::State &)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
TimingController::on_deactivate(const rclcpp_lifecycle::State &)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type TimingController::update(
  const rclcpp::Time &,
  const rclcpp::Duration &)
{
  // TODO(Svastits): disable changes if HWIF is active
  if (resend_multiplier_) {
    command_interfaces_[0].set_value(receive_multiplier_);
    resend_multiplier_ = false;
  }
  return controller_interface::return_type::OK;
}

}  // namespace kuka_controllers

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  kuka_controllers::TimingController,
  controller_interface::ControllerInterface)
