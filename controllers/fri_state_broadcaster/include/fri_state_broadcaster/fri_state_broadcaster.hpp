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

#ifndef FRI_STATE_BROADCASTER__FRI_STATE_BROADCASTER_HPP_
#define FRI_STATE_BROADCASTER__FRI_STATE_BROADCASTER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "kuka_driver_interfaces/msg/fri_state.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/time.hpp"

#include "fri_state_broadcaster/visibility_control.h"

namespace kuka_controllers
{
class FRIStateBroadcaster : public controller_interface::ControllerInterface
{
public:
  FRI_STATE_BROADCASTER_PUBLIC controller_interface::InterfaceConfiguration
  command_interface_configuration() const override;

  FRI_STATE_BROADCASTER_PUBLIC controller_interface::InterfaceConfiguration
  state_interface_configuration() const override;

  FRI_STATE_BROADCASTER_PUBLIC controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  FRI_STATE_BROADCASTER_PUBLIC controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  FRI_STATE_BROADCASTER_PUBLIC controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  FRI_STATE_BROADCASTER_PUBLIC controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  FRI_STATE_BROADCASTER_PUBLIC controller_interface::CallbackReturn on_init() override;

private:
  int counter_ = 0;
  rclcpp::Publisher<kuka_driver_interfaces::msg::FRIState>::SharedPtr robot_state_publisher_;
  kuka_driver_interfaces::msg::FRIState state_msg_;
};
}  // namespace kuka_controllers
#endif  // FRI_STATE_BROADCASTER__FRI_STATE_BROADCASTER_HPP_
