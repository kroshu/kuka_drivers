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

#ifndef KUKA_KSS_RSI_DRIVER__HARDWARE_INTERFACE_HPP_
#define KUKA_KSS_RSI_DRIVER__HARDWARE_INTERFACE_HPP_

#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "hardware_interface/system_interface.hpp"

#include "kuka_kss_rsi_driver/rsi_command.hpp"
#include "kuka_kss_rsi_driver/rsi_state.hpp"
#include "kuka_kss_rsi_driver/udp_server.hpp"
#include "kuka_kss_rsi_driver/visibility_control.h"

using hardware_interface::return_type;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace kuka_kss_rsi_driver
{

class KukaRSIHardwareInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(KukaRSIHardwareInterface)

  KUKA_KSS_RSI_DRIVER_PUBLIC
  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  KUKA_KSS_RSI_DRIVER_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  KUKA_KSS_RSI_DRIVER_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  KUKA_KSS_RSI_DRIVER_PUBLIC
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  KUKA_KSS_RSI_DRIVER_PUBLIC
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  KUKA_KSS_RSI_DRIVER_PUBLIC
  return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  KUKA_KSS_RSI_DRIVER_PUBLIC
  return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  bool stop_flag_ = false;
  bool is_active_ = false;
  std::string rsi_ip_address_ = "";
  int rsi_port_ = 0;

  std::vector<double> hw_commands_;
  std::vector<double> hw_states_;

  // RSI related joint positions
  std::vector<double> initial_joint_pos_;
  std::vector<double> joint_pos_correction_deg_;

  uint64_t ipoc_ = 0;
  RSIState rsi_state_;
  RSICommand rsi_command_;
  std::unique_ptr<UDPServer> server_;
  std::string in_buffer_;
  std::string out_buffer_;

  static constexpr double R2D = 180 / M_PI;
  static constexpr double D2R = M_PI / 180;
};
}  // namespace kuka_kss_rsi_driver

#endif  // KUKA_KSS_RSI_DRIVER__HARDWARE_INTERFACE_HPP_
