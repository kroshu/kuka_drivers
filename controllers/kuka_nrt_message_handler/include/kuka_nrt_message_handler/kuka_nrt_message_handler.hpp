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

#include <array>
#include <utility>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/u_int8.hpp"

#include "kuka_driver_interfaces/msg/kss_status.hpp"
#include "kuka_nrt_message_handler/visibility_control.h"

namespace kuka_controllers
{

using CallbackReturn = controller_interface::CallbackReturn;
using InterfaceConfig = controller_interface::InterfaceConfiguration;
using ReturnType = controller_interface::return_type;

class NrtMessageHandler : public controller_interface::ControllerInterface
{
public:
  KUKA_NRT_MESSAGE_HANDLER_PUBLIC CallbackReturn on_init() override
  {
    return CallbackReturn::SUCCESS;
  }

  KUKA_NRT_MESSAGE_HANDLER_PUBLIC CallbackReturn
  on_activate(const rclcpp_lifecycle::State &) override
  {
    return CallbackReturn::SUCCESS;
  }

  KUKA_NRT_MESSAGE_HANDLER_PUBLIC CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &) override
  {
    return CallbackReturn::SUCCESS;
  }

  KUKA_NRT_MESSAGE_HANDLER_PUBLIC InterfaceConfig command_interface_configuration() const override;

  KUKA_NRT_MESSAGE_HANDLER_PUBLIC InterfaceConfig state_interface_configuration() const override;

  KUKA_NRT_MESSAGE_HANDLER_PUBLIC CallbackReturn
  on_configure(const rclcpp_lifecycle::State &) override;

  KUKA_NRT_MESSAGE_HANDLER_PUBLIC ReturnType
  update(const rclcpp::Time &, const rclcpp::Duration &) override;

private:
  KUKA_NRT_MESSAGE_HANDLER_LOCAL void RsiCycleTimeChangedCallback(
    const std_msgs::msg::UInt8::SharedPtr msg);

  /* Drive state */
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr drive_state_subscription_;
  double drive_state_;

  /* Cycle time */
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr cycle_time_subscription_;
  double cycle_time_;

  /* Status */
  class Status
  {
  public:
    Status & operator=(
      const std::vector<hardware_interface::LoanedStateInterface> & state_interfaces);

    const kuka_driver_interfaces::msg::KssStatus & GetMessage() const
    {
      return prev_status_message_;
    }

    bool StatusChanged() const { return status_message_ != prev_status_message_; }

    void UpdateMessage() { prev_status_message_ = status_message_; }

  private:
    kuka_driver_interfaces::msg::KssStatus status_message_;
    kuka_driver_interfaces::msg::KssStatus prev_status_message_;

    const std::array<std::pair<uint8_t *, size_t>, 3> UINT8_MAPPINGS = {
      std::pair<uint8_t *, size_t>{&status_message_.control_mode, 0},
      std::pair<uint8_t *, size_t>{&status_message_.cycle_time, 1},
      std::pair<uint8_t *, size_t>{&status_message_.operation_mode, 7}};

    const std::array<std::pair<bool *, size_t>, 6> BOOL_MAPPINGS = {
      std::pair<bool *, size_t>{&status_message_.drives_powered, 2},
      std::pair<bool *, size_t>{&status_message_.emergency_stop, 3},
      std::pair<bool *, size_t>{&status_message_.guard_stop, 4},
      std::pair<bool *, size_t>{&status_message_.in_motion, 5},
      std::pair<bool *, size_t>{&status_message_.motion_possible, 6},
      std::pair<bool *, size_t>{&status_message_.robot_stopped, 8}};
  };

  rclcpp::Publisher<kuka_driver_interfaces::msg::KssStatus>::SharedPtr status_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  Status status_;

  static constexpr std::chrono::milliseconds STATUS_PUBLISH_INTERVAL{1'000};
};

}  // namespace kuka_controllers

#endif  // KUKA_NRT_MESSAGE_HANDLER__KUKA_NRT_MESSAGE_HANDLER_HPP_
