// Copyright 2024 Aron Svastits
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

#ifndef EVENT_BROADCASTER__EVENT_BROADCASTER_HPP_
#define EVENT_BROADCASTER__EVENT_BROADCASTER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/time.hpp"
#include "std_msgs/msg/u_int8.hpp"

#include "event_broadcaster/visibility_control.h"

namespace kuka_controllers
{
class EventBroadcaster : public controller_interface::ControllerInterface
{
public:
  EVENT_BROADCASTER_PUBLIC controller_interface::InterfaceConfiguration
  command_interface_configuration() const override;

  EVENT_BROADCASTER_PUBLIC controller_interface::InterfaceConfiguration
  state_interface_configuration() const override;

  EVENT_BROADCASTER_PUBLIC controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  EVENT_BROADCASTER_PUBLIC controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  EVENT_BROADCASTER_PUBLIC controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  EVENT_BROADCASTER_PUBLIC controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  EVENT_BROADCASTER_PUBLIC controller_interface::CallbackReturn on_init() override;

private:
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr event_publisher_;
  std_msgs::msg::UInt8 event_msg_;
  int last_event_ = 0;
};
}  // namespace kuka_controllers
#endif  // EVENT_BROADCASTER__EVENT_BROADCASTER_HPP_
