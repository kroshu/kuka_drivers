// Copyright 2024 KUKA Hungaria Kft.
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

#ifndef KUKA_EVENT_BROADCASTER__KUKA_EVENT_BROADCASTER_HPP_
#define KUKA_EVENT_BROADCASTER__KUKA_EVENT_BROADCASTER_HPP_

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "kuka_driver_interfaces/msg/hardware_event.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/time.hpp"

#include "kuka_event_broadcaster/kuka_event_broadcaster_parameters.hpp"

#include "kuka_event_broadcaster/visibility_control.h"

namespace kuka_controllers
{
class EventBroadcaster : public controller_interface::ControllerInterface
{
public:
  KUKA_EVENT_BROADCASTER_PUBLIC controller_interface::InterfaceConfiguration
  command_interface_configuration() const override;

  KUKA_EVENT_BROADCASTER_PUBLIC controller_interface::InterfaceConfiguration
  state_interface_configuration() const override;

  KUKA_EVENT_BROADCASTER_PUBLIC controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  KUKA_EVENT_BROADCASTER_PUBLIC controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  KUKA_EVENT_BROADCASTER_PUBLIC controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  KUKA_EVENT_BROADCASTER_PUBLIC controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  KUKA_EVENT_BROADCASTER_PUBLIC controller_interface::CallbackReturn on_init() override;

private:
  using Params = kuka_event_broadcaster::Params;
  using ParamListener = kuka_event_broadcaster::ParamListener;

  static std::string ComposeInterfaceName(
    const std::string & robot_prefix, const std::string & interface_group,
    const std::string & interface_name);

  std::shared_ptr<ParamListener> param_listener_;
  Params params_;
  std::vector<std::string> event_robot_prefixes_;
  std::vector<uint8_t> last_events_;
  // uint32_t can be casted to double without loss of precision, uint64_t cannot, so we use uint32_t for the interpolation count interface
  uint32_t interpolation_count_{0};

  rclcpp::Publisher<kuka_driver_interfaces::msg::HardwareEvent>::SharedPtr event_publisher_;
  kuka_driver_interfaces::msg::HardwareEvent event_msg_;
};
}  // namespace kuka_controllers
#endif  // KUKA_EVENT_BROADCASTER__KUKA_EVENT_BROADCASTER_HPP_
