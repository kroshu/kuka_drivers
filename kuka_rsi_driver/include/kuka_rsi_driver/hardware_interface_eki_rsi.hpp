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

<<<<<<< HEAD
  KUKA_RSI_DRIVER_PUBLIC
  return_type write(const rclcpp::Time &, const rclcpp::Duration &) override;

  KUKA_RSI_DRIVER_PUBLIC void set_server_event(kuka_drivers_core::HardwareEvent);

  KUKA_RSI_DRIVER_PUBLIC void set_stop_flag() { stop_requested_ = true; }

  KUKA_RSI_DRIVER_PUBLIC void eki_init(const InitializationData &);

  KUKA_RSI_DRIVER_PUBLIC void initialize_command_interfaces(
    kuka_drivers_core::ControlMode control_mode, RsiCycleTime cycle_time, bool drives_powered);

=======
  KUKA_RSI_DRIVER_PUBLIC void eki_init(const InitializationData &);

>>>>>>> acb5556 (Add mxAutomation support and refactor (#284))
private:
  KUKA_RSI_DRIVER_LOCAL void Read(const int64_t request_timeout) override;

<<<<<<< HEAD
  KUKA_RSI_DRIVER_LOCAL bool ConnectToController();

  KUKA_RSI_DRIVER_LOCAL void Read(const int64_t request_timeout);

  KUKA_RSI_DRIVER_LOCAL void Write();

  KUKA_RSI_DRIVER_LOCAL bool CheckJointInterfaces(
    const hardware_interface::ComponentInfo & joint) const;

  KUKA_RSI_DRIVER_LOCAL void CopyGPIOStatesToCommands();

  KUKA_RSI_DRIVER_LOCAL kuka::external::control::kss::GPIOConfiguration ParseGPIOConfig(
    const hardware_interface::InterfaceInfo & info);

  KUKA_RSI_DRIVER_LOCAL void ChangeDriveState();

  KUKA_RSI_DRIVER_LOCAL void ChangeCycleTime();

  const rclcpp::Logger logger_;
  std::unique_ptr<kuka::external::control::kss::eki::Robot> robot_ptr_;
  StatusManager status_manager_;

  std::vector<double> hw_states_;
  std::vector<double> hw_gpio_states_;
  std::vector<double> hw_commands_;
  std::vector<double> hw_gpio_commands_;

  std::vector<int> gpio_states_to_commands_map_;

  double hw_control_mode_command_;
  double server_state_;
  double drives_enabled_command_;
  double cycle_time_command_;

  std::mutex event_mutex_;

  kuka_drivers_core::ControlMode prev_control_mode_ =
    kuka_drivers_core::ControlMode::CONTROL_MODE_UNSPECIFIED;
  kuka_drivers_core::HardwareEvent last_event_ =
    kuka_drivers_core::HardwareEvent::HARDWARE_EVENT_UNSPECIFIED;
  RsiCycleTime prev_cycle_time_ = RsiCycleTime::RSI_12MS;
=======
  KUKA_RSI_DRIVER_LOCAL void CreateRobotInstance(
    const kuka::external::control::kss::Configuration &) override;
>>>>>>> acb5556 (Add mxAutomation support and refactor (#284))

  InitSequenceReport init_report_;
  std::mutex init_mtx_;
  std::condition_variable init_cv_;

<<<<<<< HEAD
  bool first_write_done_;
  bool is_active_;
  bool msg_received_;
  bool prev_drives_enabled_;
  bool drives_command_sent_;
=======
>>>>>>> acb5556 (Add mxAutomation support and refactor (#284))
  bool verify_robot_model_;
  std::atomic<bool> stop_requested_{false};
};
}  // namespace kuka_rsi_driver

#endif  // KUKA_RSI_DRIVER__HARDWARE_INTERFACE_EKI_RSI_HPP_
