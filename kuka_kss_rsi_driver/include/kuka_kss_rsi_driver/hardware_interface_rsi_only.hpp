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

#ifndef KUKA_KSS_RSI_DRIVER__HARDWARE_INTERFACE_RSI_ONLY_HPP_
#define KUKA_KSS_RSI_DRIVER__HARDWARE_INTERFACE_RSI_ONLY_HPP_

#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "kuka/external-control-sdk/kss/robot.h"
#include "kuka_drivers_core/control_mode.hpp"
#include "kuka_kss_rsi_driver/visibility_control.h"

using hardware_interface::return_type;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace kuka_kss_rsi_driver
{

class KukaRSIHardwareInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(KukaRSIHardwareInterface)

  KUKA_KSS_RSI_DRIVER_PUBLIC KukaRSIHardwareInterface()
  : SystemInterface(), logger_{rclcpp::get_logger("KukaRSIHardwareInterface")}
  {
  }

  KUKA_KSS_RSI_DRIVER_PUBLIC
  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  KUKA_KSS_RSI_DRIVER_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  KUKA_KSS_RSI_DRIVER_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  KUKA_KSS_RSI_DRIVER_PUBLIC CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;

  KUKA_KSS_RSI_DRIVER_PUBLIC CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;

  KUKA_KSS_RSI_DRIVER_PUBLIC CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;

  KUKA_KSS_RSI_DRIVER_PUBLIC CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override;

  KUKA_KSS_RSI_DRIVER_PUBLIC
  return_type read(const rclcpp::Time &, const rclcpp::Duration &) override;

  KUKA_KSS_RSI_DRIVER_PUBLIC
  return_type write(const rclcpp::Time &, const rclcpp::Duration &) override;

private:
  KUKA_KSS_RSI_DRIVER_LOCAL bool SetupRobot();

  KUKA_KSS_RSI_DRIVER_LOCAL void Read(const int64_t request_timeout);

  KUKA_KSS_RSI_DRIVER_LOCAL void Write();

  KUKA_KSS_RSI_DRIVER_LOCAL bool CheckJointInterfaces(
    const hardware_interface::ComponentInfo & joint) const;

  const rclcpp::Logger logger_;

  std::unique_ptr<kuka::external::control::kss::Robot> robot_ptr_;

  std::vector<double> hw_states_;
  std::vector<double> hw_commands_;

  bool first_write_done_;
  bool is_active_;
  bool msg_received_;
  bool stop_requested_;
};
}  // namespace kuka_kss_rsi_driver

#endif  // KUKA_KSS_RSI_DRIVER__HARDWARE_INTERFACE_RSI_ONLY_HPP_
