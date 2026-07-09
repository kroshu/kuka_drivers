// Copyright 2025 KUKA Hungaria Kft.
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

#ifndef KUKA_KSS_MESSAGE_HANDLER__KUKA_KSS_MESSAGE_HANDLER_HPP_
#define KUKA_KSS_MESSAGE_HANDLER__KUKA_KSS_MESSAGE_HANDLER_HPP_

#include <array>
#include <atomic>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/u_int8.hpp"

#include "kuka_driver_interfaces/msg/kss_status_array.hpp"
#include "kuka_driver_interfaces/msg/kss_status.hpp"
#include "kuka_kss_message_handler/kuka_kss_message_handler_parameters.hpp"
#include "kuka_kss_message_handler/visibility_control.h"

namespace kuka_controllers
{

using CallbackReturn = controller_interface::CallbackReturn;
using InterfaceConfig = controller_interface::InterfaceConfiguration;
using ReturnType = controller_interface::return_type;

class KssMessageHandler : public controller_interface::ControllerInterface
{
public:
  KUKA_KSS_MESSAGE_HANDLER_PUBLIC CallbackReturn on_init() override;

  KUKA_KSS_MESSAGE_HANDLER_PUBLIC CallbackReturn
  on_activate(const rclcpp_lifecycle::State &) override
  {
    return CallbackReturn::SUCCESS;
  }

  KUKA_KSS_MESSAGE_HANDLER_PUBLIC CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &) override
  {
    return CallbackReturn::SUCCESS;
  }

  KUKA_KSS_MESSAGE_HANDLER_PUBLIC InterfaceConfig command_interface_configuration() const override;

  KUKA_KSS_MESSAGE_HANDLER_PUBLIC InterfaceConfig state_interface_configuration() const override;

  KUKA_KSS_MESSAGE_HANDLER_PUBLIC CallbackReturn
  on_configure(const rclcpp_lifecycle::State &) override;

  KUKA_KSS_MESSAGE_HANDLER_PUBLIC ReturnType
  update(const rclcpp::Time &, const rclcpp::Duration &) override;

private:
  using UInt8StatusMember = uint8_t kuka_driver_interfaces::msg::KssStatus::*;
  using BoolStatusMember = bool kuka_driver_interfaces::msg::KssStatus::*;

  static constexpr std::array<std::pair<UInt8StatusMember, size_t>, 3> UINT8_STATUS_FIELDS = {
    std::pair<UInt8StatusMember, size_t>{&kuka_driver_interfaces::msg::KssStatus::control_mode, 0},
    std::pair<UInt8StatusMember, size_t>{&kuka_driver_interfaces::msg::KssStatus::cycle_time, 1},
    std::pair<UInt8StatusMember, size_t>{
      &kuka_driver_interfaces::msg::KssStatus::operation_mode, 7}};

  static constexpr std::array<std::pair<BoolStatusMember, size_t>, 6> BOOL_STATUS_FIELDS = {
    std::pair<BoolStatusMember, size_t>{&kuka_driver_interfaces::msg::KssStatus::drives_powered, 2},
    std::pair<BoolStatusMember, size_t>{&kuka_driver_interfaces::msg::KssStatus::emergency_stop, 3},
    std::pair<BoolStatusMember, size_t>{&kuka_driver_interfaces::msg::KssStatus::guard_stop, 4},
    std::pair<BoolStatusMember, size_t>{&kuka_driver_interfaces::msg::KssStatus::in_motion, 5},
    std::pair<BoolStatusMember, size_t>{&kuka_driver_interfaces::msg::KssStatus::motion_possible, 6},
    std::pair<BoolStatusMember, size_t>{&kuka_driver_interfaces::msg::KssStatus::robot_stopped, 8}};

  KUKA_KSS_MESSAGE_HANDLER_LOCAL static std::string ComposeInterfaceName(
    const std::string & robot_name, const std::string & interface_group,
    const std::string & interface_name);
  KUKA_KSS_MESSAGE_HANDLER_LOCAL static void AssignStatusFromInterfaces(
    kuka_driver_interfaces::msg::KssStatus & status,
    const std::vector<hardware_interface::LoanedStateInterface> & state_interfaces,
    size_t start_idx);

  template <typename T>
  static T ReadInterfaceValue(
    const hardware_interface::LoanedStateInterface & state_interface,
    const T fallback_value)
  {
    return static_cast<T>(
      state_interface.get_optional().value_or(static_cast<double>(fallback_value)));
  }

  KUKA_KSS_MESSAGE_HANDLER_LOCAL void RsiCycleTimeChangedCallback(
    const std_msgs::msg::UInt8::SharedPtr msg);

  using Params = kuka_kss_message_handler::Params;
  using ParamListener = kuka_kss_message_handler::ParamListener;
  std::shared_ptr<ParamListener> param_listener_;
  Params params_;

  // Cycle time
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr cycle_time_subscription_;
  std::atomic<double> cycle_time_;
  double prev_cycle_time_{std::numeric_limits<double>::quiet_NaN()};

  rclcpp::Publisher<kuka_driver_interfaces::msg::KssStatusArray>::SharedPtr status_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::vector<std::string> robot_names_;
  std::vector<kuka_driver_interfaces::msg::KssStatus> current_statuses_;
  kuka_driver_interfaces::msg::KssStatusArray status_msg_;

  static constexpr size_t STATE_INTERFACE_COUNT = 9;

  static constexpr std::chrono::milliseconds STATUS_PUBLISH_INTERVAL{1'000};
  static constexpr int WARN_THROTTLE_DURATION_MS = 1000;
};

}  // namespace kuka_controllers

#endif  // KUKA_KSS_MESSAGE_HANDLER__KUKA_KSS_MESSAGE_HANDLER_HPP_
