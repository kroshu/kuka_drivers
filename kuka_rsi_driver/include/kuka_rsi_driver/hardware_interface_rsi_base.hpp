// Copyright 2023 KUKA Hungaria Kft.
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

#ifndef KUKA_RSI_DRIVER__HARDWARE_INTERFACE_RSI_BASE_HPP_
#define KUKA_RSI_DRIVER__HARDWARE_INTERFACE_RSI_BASE_HPP_

#include <memory>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "kuka/external-control-sdk/kss/configuration.h"
#include "kuka/external-control-sdk/kss/rsi/robot_interface.h"
#include "kuka_drivers_core/control_mode.hpp"
#include "kuka_rsi_driver/robot_status_manager.hpp"
#include "kuka_rsi_driver/visibility_control.h"

using hardware_interface::return_type;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
using RsiCycleTime = kuka::external::control::kss::CycleTime;

namespace kuka_rsi_driver
{

class KukaRSIHardwareInterfaceBase : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(KukaRSIHardwareInterfaceBase)

  explicit KUKA_RSI_DRIVER_PUBLIC KukaRSIHardwareInterfaceBase(const std::string & logger_name)
  : SystemInterface(), logger_(rclcpp::get_logger(logger_name))
  {
  }

  virtual KUKA_RSI_DRIVER_PUBLIC ~KukaRSIHardwareInterfaceBase() = default;

  KUKA_RSI_DRIVER_PUBLIC
  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
  // TODO: On init should be changed to this
  // KUKA_RSI_DRIVER_PUBLIC
  // CallbackReturn on_init(const hardware_interface::HardwareComponentInterfaceParams & params)
  // override;

  KUKA_RSI_DRIVER_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  KUKA_RSI_DRIVER_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  KUKA_RSI_DRIVER_PUBLIC CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override;

  KUKA_RSI_DRIVER_PUBLIC
  return_type read(const rclcpp::Time &, const rclcpp::Duration &) override;

  KUKA_RSI_DRIVER_PUBLIC
  return_type write(const rclcpp::Time &, const rclcpp::Duration &) override;

  KUKA_RSI_DRIVER_PUBLIC void set_server_event(kuka_drivers_core::HardwareEvent);
  KUKA_RSI_DRIVER_PUBLIC void initialize_command_interfaces(
    kuka_drivers_core::ControlMode control_mode, RsiCycleTime cycle_time);

protected:
  KUKA_RSI_DRIVER_LOCAL bool SetupRobot(
    kuka::external::control::kss::Configuration & config,
    std::unique_ptr<kuka::external::control::EventHandler> event_handler,
    std::unique_ptr<kuka::external::control::kss::IEventHandlerExtension> extension);

  virtual KUKA_RSI_DRIVER_LOCAL void Read(const int64_t request_timeout);

  KUKA_RSI_DRIVER_LOCAL void Write();

  KUKA_RSI_DRIVER_LOCAL bool CheckJointInterfaces(
    const hardware_interface::ComponentInfo & joint) const;

  KUKA_RSI_DRIVER_LOCAL void CopyGPIOStatesToCommands();

  virtual KUKA_RSI_DRIVER_LOCAL void CreateRobotInstance(
    const kuka::external::control::kss::Configuration &) = 0;

  KUKA_RSI_DRIVER_LOCAL kuka::external::control::kss::GPIOConfiguration ParseGPIOConfig(
    const hardware_interface::InterfaceInfo & info);

  struct InitSequenceReport
  {
    bool sequence_complete = false;
    bool ok = false;
    std::string reason = "";
  };

  // Methods common for EKI and MXA version
  CallbackReturn extended_activation(const rclcpp_lifecycle::State &);
  CallbackReturn extended_deactivation(const rclcpp_lifecycle::State &);
  kuka::external::control::Status ChangeCycleTime();

  const rclcpp::Logger logger_;

  std::unique_ptr<kuka::external::control::kss::rsi::Robot> robot_ptr_;

  std::vector<double> hw_states_;
  std::vector<double> hw_gpio_states_;
  std::vector<double> hw_commands_;
  std::vector<double> hw_gpio_commands_;

  double server_state_;
  kuka_drivers_core::HardwareEvent last_event_ =
    kuka_drivers_core::HardwareEvent::HARDWARE_EVENT_UNSPECIFIED;
  std::mutex event_mutex_;

  std::vector<int> gpio_states_to_commands_map_;

  bool is_active_;
  bool msg_received_;

  // EKI-MXA common variables
  StatusManager status_manager_;

  double hw_control_mode_command_;
  double cycle_time_command_;

  kuka_drivers_core::ControlMode prev_control_mode_ =
    kuka_drivers_core::ControlMode::CONTROL_MODE_UNSPECIFIED;
  RsiCycleTime prev_cycle_time_ = RsiCycleTime::RSI_12MS;

  static constexpr std::chrono::milliseconds IDLE_SLEEP_DURATION{2};
  static constexpr std::chrono::milliseconds INIT_WAIT_DURATION{100};
  static constexpr std::chrono::seconds DRIVES_POWERED_TIMEOUT{3};
  static constexpr std::chrono::milliseconds DRIVES_POWERED_CHECK_INTERVAL{100};
  static constexpr int64_t READ_TIMEOUT_MS = 1'000;
};
}  // namespace kuka_rsi_driver

#endif  // KUKA_RSI_DRIVER__HARDWARE_INTERFACE_RSI_BASE_HPP_
