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

#ifndef KUKA_RSI_HW_INTERFACE__KUKA_HARDWARE_INTERFACE_HPP_
#define KUKA_RSI_HW_INTERFACE__KUKA_HARDWARE_INTERFACE_HPP_

#include <kuka_rsi_hw_interface/udp_server.h>
#include <kuka_rsi_hw_interface/rsi_state.h>
#include <kuka_rsi_hw_interface/rsi_command.h>

#include <vector>
#include <string>
#include <memory>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

namespace kuka_rsi_hw_interface
{
class KukaHardwareInterface
{
public:
  explicit KukaHardwareInterface(const std::string & rsi_ip_address, int rsi_port, uint8_t n_dof);

  void start(std::vector<double> & joint_state_msg_position);
  void stop();
  bool read(std::vector<double> & joint_state_msg_position);
  bool write(const std::vector<double> & joint_pos_correction_deg_);

  bool isActive() const;

private:
  bool is_active_ = false;
  const uint8_t n_dof_ = 6;
  std::string rsi_ip_address_ = "";
  int rsi_port_ = 0;

  std::vector<double> initial_joint_pos_ = std::vector<double>(n_dof_, 0.0);
  std::vector<double> joint_pos_correction_deg_ = std::vector<double>(n_dof_, 0.0);

  uint64_t ipoc_ = 0;
  RSIState rsi_state_;
  RSICommand rsi_command_;
  std::unique_ptr<UDPServer> server_;
  std::string in_buffer_;
  std::string out_buffer_;
  std::mutex m_;

  static constexpr double R2D = 180 / M_PI;
  static constexpr double D2R = M_PI / 180;
};
} // namespace kuka_rsi_hw_interface

#endif // KUKA_RSI_HW_INTERFACE__KUKA_HARDWARE_INTERFACE_HPP_
