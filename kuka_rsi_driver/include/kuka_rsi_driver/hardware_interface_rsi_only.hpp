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

#ifndef KUKA_RSI_DRIVER__HARDWARE_INTERFACE_RSI_ONLY_HPP_
#define KUKA_RSI_DRIVER__HARDWARE_INTERFACE_RSI_ONLY_HPP_

#include <memory>
#include <vector>

#include "hardware_interface_rsi_base.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "kuka/external-control-sdk/kss/robot.h"
#include "kuka_drivers_core/control_mode.hpp"
#include "kuka_rsi_driver/visibility_control.h"

using hardware_interface::return_type;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace kuka_rsi_driver
{

class KukaRSIHardwareInterface : public kuka_rsi_driver::KukaRSIHardwareInterfaceBase
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(KukaRSIHardwareInterface)

  KUKA_RSI_DRIVER_PUBLIC KukaRSIHardwareInterface()
  : KukaRSIHardwareInterfaceBase("KukaMxaRsiHardwareInterface")
  {
  }

<<<<<<< HEAD
  KUKA_RSI_DRIVER_PUBLIC
  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  KUKA_RSI_DRIVER_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  KUKA_RSI_DRIVER_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

=======
>>>>>>> acb5556 (Add mxAutomation support and refactor (#284))
  KUKA_RSI_DRIVER_PUBLIC CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;

  KUKA_RSI_DRIVER_PUBLIC CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;

  KUKA_RSI_DRIVER_PUBLIC CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;

private:
<<<<<<< HEAD
  KUKA_RSI_DRIVER_LOCAL bool SetupRobot();

  KUKA_RSI_DRIVER_LOCAL void Read(const int64_t request_timeout);

  KUKA_RSI_DRIVER_LOCAL void Write();

  KUKA_RSI_DRIVER_LOCAL bool CheckJointInterfaces(
    const hardware_interface::ComponentInfo & joint) const;

  KUKA_RSI_DRIVER_LOCAL void CopyGPIOStatesToCommands();

  KUKA_RSI_DRIVER_LOCAL kuka::external::control::kss::GPIOConfiguration ParseGPIOConfig(
    const hardware_interface::InterfaceInfo & info);

  const rclcpp::Logger logger_;

  std::unique_ptr<kuka::external::control::kss::Robot> robot_ptr_;

  std::vector<double> hw_states_;
  std::vector<double> hw_gpio_states_;
  std::vector<double> hw_commands_;
  std::vector<double> hw_gpio_commands_;

  double server_state_;

  std::vector<int> gpio_states_to_commands_map_;

  bool first_write_done_;
  bool is_active_;
  bool msg_received_;
  bool stop_requested_;

  static constexpr int64_t READ_TIMEOUT_MS = 1'000;
=======
  KUKA_RSI_DRIVER_LOCAL void CreateRobotInstance(
    const kuka::external::control::kss::Configuration &) override;
>>>>>>> acb5556 (Add mxAutomation support and refactor (#284))
};
}  // namespace kuka_rsi_driver

#endif  // KUKA_RSI_DRIVER__HARDWARE_INTERFACE_RSI_ONLY_HPP_
