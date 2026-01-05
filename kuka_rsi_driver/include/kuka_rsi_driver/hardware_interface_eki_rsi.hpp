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

#ifndef KUKA_RSI_DRIVER__HARDWARE_INTERFACE_EKI_RSI_HPP_
#define KUKA_RSI_DRIVER__HARDWARE_INTERFACE_EKI_RSI_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface_rsi_base.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "kuka/external-control-sdk/kss/eki/robot_interface.h"
#include "kuka_drivers_core/control_mode.hpp"
#include "kuka_rsi_driver/visibility_control.h"

using hardware_interface::return_type;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
using InitializationData = kuka::external::control::kss::InitializationData;
using RsiCycleTime = kuka::external::control::kss::CycleTime;

namespace kuka_rsi_driver
{
class KukaEkiRsiHardwareInterface : public kuka_rsi_driver::KukaRSIHardwareInterfaceBase
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(KukaEkiRsiHardwareInterface)

  KUKA_RSI_DRIVER_PUBLIC KukaEkiRsiHardwareInterface()
  : KukaRSIHardwareInterfaceBase("KukaEkiRsiHardwareInterface")
  {
  }

  KUKA_RSI_DRIVER_PUBLIC
  CallbackReturn on_init(const hardware_interface::HardwareInfo &) override;

  KUKA_RSI_DRIVER_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  KUKA_RSI_DRIVER_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  KUKA_RSI_DRIVER_PUBLIC CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;

  KUKA_RSI_DRIVER_PUBLIC
  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;

  KUKA_RSI_DRIVER_PUBLIC
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;

  KUKA_RSI_DRIVER_PUBLIC
  return_type read(const rclcpp::Time &, const rclcpp::Duration &) override;

  KUKA_RSI_DRIVER_PUBLIC void eki_init(const InitializationData &);

private:
  KUKA_RSI_DRIVER_LOCAL void Read(const int64_t request_timeout) override;

  KUKA_RSI_DRIVER_LOCAL void CreateRobotInstance(
    const kuka::external::control::kss::Configuration &) override;

  InitSequenceReport init_report_;
  std::mutex init_mtx_;
  std::condition_variable init_cv_;

  bool verify_robot_model_;
};
}  // namespace kuka_rsi_driver

#endif  // KUKA_RSI_DRIVER__HARDWARE_INTERFACE_EKI_RSI_HPP_
