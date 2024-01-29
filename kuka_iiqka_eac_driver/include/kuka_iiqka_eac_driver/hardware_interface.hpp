// Copyright 2022 Áron Svastits
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

#include "rclcpp_lifecycle/state.hpp"
#include "os-core-udp-communication/replier.h"
#include "kuka/external-control-sdk/iiqka/robot.h"
#include "kuka/external-control-sdk/iiqka/message_builder.h"

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

private:
  KUKA_IIQKA_EAC_DRIVER_LOCAL bool SetupRobot();
  KUKA_IIQKA_EAC_DRIVER_LOCAL bool SetupQoS();
  KUKA_IIQKA_EAC_DRIVER_LOCAL void ObserveControl();

  std::unique_ptr<kuka::external::control::iiqka::Robot> robot_ptr_;
  kuka::external::control::BaseControlSignal* hw_control_signal_ = nullptr;

  std::vector<double> hw_position_commands_;
  std::vector<double> hw_torque_commands_;
  std::vector<double> hw_stiffness_commands_;
  std::vector<double> hw_damping_commands_;
  std::vector<double> hw_position_states_;
  std::vector<double> hw_torque_states_;

  kuka::ecs::v1::CommandState command_state_;
  bool msg_received_;
  

  std::chrono::milliseconds receive_timeout_{1000};

};
}  // namespace kuka_eac

#endif  // KUKA_IIQKA_EAC_DRIVER__HARDWARE_INTERFACE_HPP_
