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

#ifndef KUKA_CONTROL_MODE_HANDLER__KUKA_CONTROL_MODE_HANDLER_HPP_
#define KUKA_CONTROL_MODE_HANDLER__KUKA_CONTROL_MODE_HANDLER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "kuka_drivers_core/control_mode.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/time.hpp"
#include "std_msgs/msg/u_int32.hpp"

#include "kuka_control_mode_handler/visibility_control.h"

namespace kuka_controllers
{
class ControlModeHandler : public controller_interface::ControllerInterface
{
public:
  KUKA_CONTROL_MODE_HANDLER_PUBLIC controller_interface::InterfaceConfiguration
  command_interface_configuration() const override;

  KUKA_CONTROL_MODE_HANDLER_PUBLIC controller_interface::InterfaceConfiguration
  state_interface_configuration() const override;

  KUKA_CONTROL_MODE_HANDLER_PUBLIC controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  KUKA_CONTROL_MODE_HANDLER_PUBLIC controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  KUKA_CONTROL_MODE_HANDLER_PUBLIC controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  KUKA_CONTROL_MODE_HANDLER_PUBLIC controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  KUKA_CONTROL_MODE_HANDLER_PUBLIC controller_interface::CallbackReturn on_init() override;

private:
  rclcpp::Subscription<std_msgs::msg::UInt32>::SharedPtr control_mode_subscriber_;
  kuka_drivers_core::ControlMode control_mode_ =
    kuka_drivers_core::ControlMode::CONTROL_MODE_UNSPECIFIED;
};
}  // namespace kuka_controllers
#endif  // KUKA_CONTROL_MODE_HANDLER__KUKA_CONTROL_MODE_HANDLER_HPP_
