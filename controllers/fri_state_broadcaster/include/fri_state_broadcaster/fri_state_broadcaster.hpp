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

#ifndef FRI_STATE_BROADCASTER__FRI_STATE_BROADCASTER_HPP_
#define FRI_STATE_BROADCASTER__FRI_STATE_BROADCASTER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "hardware_interface/loaned_state_interface.hpp"
#include "kuka_driver_interfaces/msg/fri_state_array.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/time.hpp"

#include "fri_state_broadcaster/fri_state_broadcaster_parameters.hpp"
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
  static std::string ComposeInterfaceName(
    const std::string & robot_prefix, const std::string & interface_name);

  static void AssignStateFromInterfaces(
    kuka_driver_interfaces::msg::FRIState & state,
    const std::vector<hardware_interface::LoanedStateInterface> & state_interfaces,
    size_t start_idx);

  using Params = fri_state_broadcaster::Params;
  using ParamListener = fri_state_broadcaster::ParamListener;
  std::shared_ptr<ParamListener> param_listener_;
  Params params_;

  int counter_ = 0;
  rclcpp::Publisher<kuka_driver_interfaces::msg::FRIStateArray>::SharedPtr state_publisher_;
  std::vector<std::string> robot_prefixes_;
  std::vector<kuka_driver_interfaces::msg::FRIState> current_states_;
  kuka_driver_interfaces::msg::FRIStateArray state_msg_;

  static constexpr size_t STATE_INTERFACE_COUNT = 9;
};
}  // namespace kuka_controllers
#endif  // FRI_STATE_BROADCASTER__FRI_STATE_BROADCASTER_HPP_
