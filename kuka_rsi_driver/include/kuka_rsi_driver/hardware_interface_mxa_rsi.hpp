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

#ifndef KUKA_RSI_DRIVER__HARDWARE_INTERFACE_MXA_RSI_HPP_
#define KUKA_RSI_DRIVER__HARDWARE_INTERFACE_MXA_RSI_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface_rsi_base.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "kuka/external-control-sdk/kss/mxa/robot_interface.h"
#include "kuka_drivers_core/control_mode.hpp"
#include "kuka_rsi_driver/robot_status_manager.hpp"
#include "kuka_rsi_driver/visibility_control.h"

using hardware_interface::return_type;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
using InitializationData = kuka::external::control::kss::mxa::InitializationData;
using RsiCycleTime = kuka::external::control::kss::CycleTime;

namespace kuka_rsi_driver
{
class KukaMxaRsiHardwareInterface : public kuka_rsi_driver::KukaRSIHardwareInterfaceBase
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(KukaMxaRsiHardwareInterface)

  KUKA_RSI_DRIVER_PUBLIC KukaMxaRsiHardwareInterface()
  {
  }

  KUKA_RSI_DRIVER_PUBLIC
  CallbackReturn on_init(const hardware_interface::HardwareInfo &) override;

  KUKA_RSI_DRIVER_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  KUKA_RSI_DRIVER_PUBLIC CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;

  KUKA_RSI_DRIVER_PUBLIC
  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;

  KUKA_RSI_DRIVER_PUBLIC
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;

  KUKA_RSI_DRIVER_PUBLIC
  return_type read(const rclcpp::Time &, const rclcpp::Duration &) override;

  KUKA_RSI_DRIVER_PUBLIC void mxa_init(const InitializationData &);

  KUKA_RSI_DRIVER_PUBLIC void initialize_command_interfaces(
    kuka_drivers_core::ControlMode control_mode, RsiCycleTime cycle_time);

private:
  struct InitSequenceReport
  {
    bool sequence_complete = false;
    bool ok = false;
    std::string reason = "";
  };

  KUKA_RSI_DRIVER_LOCAL bool ConnectToController();

  KUKA_RSI_DRIVER_LOCAL void Read(const int64_t request_timeout);

  KUKA_RSI_DRIVER_LOCAL void Write() override;

  KUKA_RSI_DRIVER_LOCAL kuka::external::control::Status ChangeCycleTime();

  KUKA_RSI_DRIVER_LOCAL void CreateRobotInstance(const kuka::external::control::kss::Configuration&) override;

  std::unique_ptr<kuka::external::control::kss::mxa::Robot> robot_ptr_;
  StatusManager status_manager_;

  double hw_control_mode_command_;
  double cycle_time_command_;

  std::mutex event_mutex_;

  kuka_drivers_core::ControlMode prev_control_mode_ =
    kuka_drivers_core::ControlMode::CONTROL_MODE_UNSPECIFIED;
  kuka_drivers_core::HardwareEvent last_event_ =
    kuka_drivers_core::HardwareEvent::HARDWARE_EVENT_UNSPECIFIED;
  RsiCycleTime prev_cycle_time_ = RsiCycleTime::RSI_12MS;

  InitSequenceReport init_report_;
  std::mutex init_mtx_;
  std::condition_variable init_cv_;

  bool verify_robot_model_;
  std::atomic<bool> stop_requested_{false};

  static constexpr std::chrono::milliseconds IDLE_SLEEP_DURATION{2};
  static constexpr std::chrono::milliseconds INIT_WAIT_DURATION{100};
  static constexpr std::chrono::seconds DRIVES_POWERED_TIMEOUT{10};
  static constexpr std::chrono::milliseconds DRIVES_POWERED_CHECK_INTERVAL{100};
};
}  // namespace kuka_rsi_driver

#endif  // KUKA_RSI_DRIVER__HARDWARE_INTERFACE_MXA_RSI_HPP_
