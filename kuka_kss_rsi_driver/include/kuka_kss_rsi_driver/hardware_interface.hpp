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

#ifndef KUKA_KSS_RSI_DRIVER__HARDWARE_INTERFACE_HPP_
#define KUKA_KSS_RSI_DRIVER__HARDWARE_INTERFACE_HPP_

#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <hardware_interface/system_interface.hpp>
#include <rclcpp/macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include "kuka/external-control-sdk/kss/irobot.h"
#include "kuka_drivers_core/control_mode.hpp"
#include "kuka_kss_rsi_driver/initialization_handler.hpp"
#include "kuka_kss_rsi_driver/robot_status_manager.hpp"
#include "kuka_kss_rsi_driver/visibility_control.h"

namespace kuka_kss_rsi_driver
{

class HardwareInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(HardwareInterface)

  KUKA_KSS_RSI_DRIVER_PUBLIC HardwareInterface()
  : SystemInterface(), logger_(rclcpp::get_logger("HardwareInterface"))
  {
  }

  KUKA_KSS_RSI_DRIVER_PUBLIC
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo &) override;

  KUKA_KSS_RSI_DRIVER_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  KUKA_KSS_RSI_DRIVER_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  KUKA_KSS_RSI_DRIVER_PUBLIC
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State &) override;

  KUKA_KSS_RSI_DRIVER_PUBLIC
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State &) override;

  KUKA_KSS_RSI_DRIVER_PUBLIC
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State &) override;

  KUKA_KSS_RSI_DRIVER_PUBLIC
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State &) override;

  KUKA_KSS_RSI_DRIVER_PUBLIC
  hardware_interface::return_type read(const rclcpp::Time &, const rclcpp::Duration &) override;

  KUKA_KSS_RSI_DRIVER_PUBLIC
  hardware_interface::return_type write(const rclcpp::Time &, const rclcpp::Duration &) override;

  // KUKA driver specific methods
  KUKA_KSS_RSI_DRIVER_PUBLIC rclcpp::Logger GetLogger() const { return logger_; }

  KUKA_KSS_RSI_DRIVER_PUBLIC void HandleInitialization(
    const kuka::external::control::kss::InitializationData & init_data);

  KUKA_KSS_RSI_DRIVER_PUBLIC void InitializeCommandInterfaces(
    kuka_drivers_core::ControlMode control_mode,
    kuka::external::control::kss::CycleTime cycle_time);

  KUKA_KSS_RSI_DRIVER_PUBLIC void SetServerEvent(kuka_drivers_core::HardwareEvent event);

  KUKA_KSS_RSI_DRIVER_PUBLIC void SetStopFlag() { stop_requested_ = true; }

private:
  KUKA_KSS_RSI_DRIVER_LOCAL bool ConnectToController();

  KUKA_KSS_RSI_DRIVER_LOCAL bool ShouldWriteJointCommands() const;

  KUKA_KSS_RSI_DRIVER_LOCAL void Read(const std::chrono::milliseconds timeout);

  KUKA_KSS_RSI_DRIVER_LOCAL void Write();

  KUKA_KSS_RSI_DRIVER_LOCAL bool CheckJointInterfaces(
    const hardware_interface::ComponentInfo & joint) const;

  KUKA_KSS_RSI_DRIVER_LOCAL bool ChangeDriveState();

  KUKA_KSS_RSI_DRIVER_LOCAL bool ChangeCycleTime();

  const rclcpp::Logger logger_;

  std::unique_ptr<kuka::external::control::kss::IRobot> robot_ptr_;
  StatusManager status_manager_;

  std::vector<double> hw_states_;
  std::vector<double> hw_commands_;

  double control_mode_command_;
  double server_state_;
  double drives_enabled_command_;
  double cycle_time_command_;

  std::mutex event_mutex_;

  kuka_drivers_core::ControlMode prev_control_mode_ =
    kuka_drivers_core::ControlMode::CONTROL_MODE_UNSPECIFIED;
  kuka_drivers_core::HardwareEvent last_event_ =
    kuka_drivers_core::HardwareEvent::HARDWARE_EVENT_UNSPECIFIED;
  kuka::external::control::kss::CycleTime prev_cycle_time_ =
    kuka::external::control::kss::CycleTime::RSI_12MS;

  std::unique_ptr<IInitializationHandler> initialization_handler_;
  InitSequenceReport init_report_;
  std::mutex init_mtx_;
  std::condition_variable init_cv_;

  bool first_write_done_;
  bool is_active_;
  bool msg_received_;
  bool prev_drives_enabled_;
  std::atomic<bool> stop_requested_{false};

  static constexpr std::chrono::milliseconds IDLE_SLEEP_DURATION{2};
  static constexpr std::chrono::milliseconds INIT_WAIT_DURATION{100};
  static constexpr std::chrono::milliseconds FIRST_READ_TIMEOUT{5'000};
  static constexpr std::chrono::milliseconds READ_TIMEOUT{1'000};
};

}  // namespace kuka_kss_rsi_driver

#endif  // KUKA_KSS_RSI_DRIVER__HARDWARE_INTERFACE_HPP_
