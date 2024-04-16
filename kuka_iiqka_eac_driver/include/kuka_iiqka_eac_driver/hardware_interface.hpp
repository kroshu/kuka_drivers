// Copyright 2022 Aron Svastits
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

#ifndef KUKA_IIQKA_EAC_DRIVER__HARDWARE_INTERFACE_HPP_
#define KUKA_IIQKA_EAC_DRIVER__HARDWARE_INTERFACE_HPP_

#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/rclcpp.hpp"

#include "kuka/external-control-sdk/iiqka/sdk.h"
#include "rclcpp_lifecycle/state.hpp"

#include "kuka_drivers_core/hardware_event.hpp"

#include "kuka_iiqka_eac_driver/visibility_control.h"

using hardware_interface::return_type;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace kuka_eac
{
class KukaEACHardwareInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(KukaEACHardwareInterface)

  KUKA_IIQKA_EAC_DRIVER_PUBLIC CallbackReturn
  on_init(const hardware_interface::HardwareInfo & info) override;

  KUKA_IIQKA_EAC_DRIVER_PUBLIC std::vector<hardware_interface::StateInterface>
  export_state_interfaces() override;

  KUKA_IIQKA_EAC_DRIVER_PUBLIC std::vector<hardware_interface::CommandInterface>
  export_command_interfaces() override;

  KUKA_IIQKA_EAC_DRIVER_PUBLIC CallbackReturn
  on_configure(const rclcpp_lifecycle::State & previous_state) override;

  KUKA_IIQKA_EAC_DRIVER_PUBLIC CallbackReturn
  on_activate(const rclcpp_lifecycle::State & previous_state) override;

  KUKA_IIQKA_EAC_DRIVER_PUBLIC CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  KUKA_IIQKA_EAC_DRIVER_PUBLIC return_type
  read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  KUKA_IIQKA_EAC_DRIVER_PUBLIC return_type
  write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  KUKA_IIQKA_EAC_DRIVER_PUBLIC void set_server_event(kuka_drivers_core::HardwareEvent event);

  KUKA_IIQKA_EAC_DRIVER_PUBLIC void set_stop_flag() { stop_requested_ = true; }

  KUKA_IIQKA_EAC_DRIVER_PUBLIC void reset_cycle_count() { cycle_count_ = 0; }

private:
  KUKA_IIQKA_EAC_DRIVER_LOCAL bool SetupRobot();
  KUKA_IIQKA_EAC_DRIVER_LOCAL bool SetupQoS();

  std::unique_ptr<kuka::external::control::iiqka::Robot> robot_ptr_;

  std::vector<double> hw_position_commands_;
  std::vector<double> hw_torque_commands_;
  std::vector<double> hw_stiffness_commands_;
  std::vector<double> hw_damping_commands_;
  std::vector<double> hw_position_states_;
  std::vector<double> hw_torque_states_;

  double hw_control_mode_command_ = 0;
  double server_state_ = 0;
  int cycle_count_ = 0;

  std::mutex event_mutex_;

  kuka_drivers_core::ControlMode prev_control_mode_ =
    kuka_drivers_core::ControlMode::CONTROL_MODE_UNSPECIFIED;
  kuka_drivers_core::HardwareEvent last_event_ =
    kuka_drivers_core::HardwareEvent::HARDWARE_EVENT_UNSPECIFIED;

  bool msg_received_;
  std::atomic<bool> stop_requested_{false};
};
}  // namespace kuka_eac

#endif  // KUKA_IIQKA_EAC_DRIVER__HARDWARE_INTERFACE_HPP_
