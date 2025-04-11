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

#ifndef KUKA_NRT_MESSAGE_HANDLER__KUKA_NRT_MESSAGE_HANDLER_HPP_
#define KUKA_NRT_MESSAGE_HANDLER__KUKA_NRT_MESSAGE_HANDLER_HPP_

#include "controller_interface/controller_interface.hpp"
#include "std_msgs/msg/bool.hpp"

#include "kuka_driver_interfaces/msg/rsi_cycle_time.hpp"
#include "kuka_nrt_message_handler/visibility_control.h"

namespace kuka_controllers
{

using CallbackReturn = controller_interface::CallbackReturn;
using InterfaceConfig = controller_interface::InterfaceConfiguration;
using ReturnType = controller_interface::return_type;

class KukaNrtMessageHandler : public controller_interface::ControllerInterface
{
public:
  KUKA_NRT_MESSAGE_HANDLER_PUBLIC CallbackReturn on_init() override;

  KUKA_NRT_MESSAGE_HANDLER_PUBLIC InterfaceConfig command_interface_configuration() const override;

  KUKA_NRT_MESSAGE_HANDLER_PUBLIC InterfaceConfig state_interface_configuration() const override;

  KUKA_NRT_MESSAGE_HANDLER_PUBLIC CallbackReturn
  on_configure(const rclcpp_lifecycle::State &) override;

  KUKA_NRT_MESSAGE_HANDLER_PUBLIC CallbackReturn
  on_activate(const rclcpp_lifecycle::State &) override;

  KUKA_NRT_MESSAGE_HANDLER_PUBLIC CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &) override;

  KUKA_NRT_MESSAGE_HANDLER_PUBLIC ReturnType
  update(const rclcpp::Time &, const rclcpp::Duration &) override;

private:
  KUKA_NRT_MESSAGE_HANDLER_LOCAL void DriveStateChangedCallback(
    const std_msgs::msg::Bool::SharedPtr msg);

  KUKA_NRT_MESSAGE_HANDLER_LOCAL void RsiCycleTimeChangedCallback(
    const kuka_driver_interfaces::msg::RsiCycleTime::SharedPtr msg);

  // Drive state related variables
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr drive_state_subscription_;
  double drive_state_;

  // Cycle time related variables
  rclcpp::Subscription<kuka_driver_interfaces::msg::RsiCycleTime>::SharedPtr
    cycle_time_subscription_;
  double cycle_time_;
};

}  // namespace kuka_controllers

#endif  // KUKA_NRT_MESSAGE_HANDLER__KUKA_NRT_MESSAGE_HANDLER_HPP_
