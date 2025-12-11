// Copyright 2023 Aron Svastits
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

#include "kuka/external-control-sdk/common/irobot.h"
#include "kuka/external-control-sdk/kss/configuration.h"
#include "kuka_drivers_core/control_mode.hpp"
#include "kuka_rsi_driver/visibility_control.h"

using hardware_interface::return_type;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace kuka_rsi_driver
{

class KukaRSIHardwareInterfaceBase : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(KukaRSIHardwareInterfaceBase)

  KUKA_RSI_DRIVER_PUBLIC KukaRSIHardwareInterfaceBase()
  : SystemInterface(), logger_(rclcpp::get_logger("KukaRsiHardwareInterfaceBase"))
  {
  }

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

  KUKA_RSI_DRIVER_PUBLIC CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;

  KUKA_RSI_DRIVER_PUBLIC CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;

  KUKA_RSI_DRIVER_PUBLIC CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;

  KUKA_RSI_DRIVER_PUBLIC CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override;

  KUKA_RSI_DRIVER_PUBLIC
  return_type read(const rclcpp::Time &, const rclcpp::Duration &) override;

  KUKA_RSI_DRIVER_PUBLIC
  return_type write(const rclcpp::Time &, const rclcpp::Duration &) override;

  KUKA_RSI_DRIVER_PUBLIC void set_server_event(kuka_drivers_core::HardwareEvent);

protected:
  KUKA_RSI_DRIVER_LOCAL bool SetupRobot();

  KUKA_RSI_DRIVER_LOCAL void Read(const int64_t request_timeout);

  KUKA_RSI_DRIVER_LOCAL void Write();

  KUKA_RSI_DRIVER_LOCAL bool CheckJointInterfaces(
    const hardware_interface::ComponentInfo & joint) const;

  KUKA_RSI_DRIVER_LOCAL void CopyGPIOStatesToCommands();

  virtual KUKA_RSI_DRIVER_LOCAL void CreateRobotInstance(const kuka::external::control::kss::Configuration&) = 0;

  KUKA_RSI_DRIVER_LOCAL kuka::external::control::kss::GPIOConfiguration ParseGPIOConfig(
    const hardware_interface::InterfaceInfo & info);

  const rclcpp::Logger logger_;

  std::unique_ptr<kuka::external::control::IRobot> robot_ptr_;

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

  static constexpr int64_t READ_TIMEOUT_MS = 1'000;
};
}  // namespace kuka_rsi_driver

#endif  // KUKA_RSI_DRIVER__HARDWARE_INTERFACE_RSI_BASE_HPP_
