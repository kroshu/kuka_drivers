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
  KukaHardwareInterface(rclcpp_lifecycle::LifecycleNode::SharedPtr robot_control_node);

  void start();
  void stop();
  void configure();
  void cleanup();
  bool read();
  void write();

private:
  unsigned int n_dof_ = 6;

  std::vector<std::string> joint_names_ = std::vector<std::string>(6);

  // RSI
  RSIState rsi_state_;
  RSICommand rsi_command_;
  std::vector<double> initial_joint_pos_ = std::vector<double>(6, 0.0);
  std::vector<double> joint_pos_correction_deg_ = std::vector<double>(6, 0.0);
  uint64_t ipoc_ = 0;

  std::unique_ptr<UDPServer> server_;
  std::string local_host_ = "";
  int local_port_ = 0;
  std::string in_buffer_;
  std::string out_buffer_;
  rclcpp_lifecycle::LifecycleNode::SharedPtr control_node_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::JointState>::SharedPtr
    joint_state_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_command_subscription_;

  rclcpp::CallbackGroup::SharedPtr cbg_;
  sensor_msgs::msg::JointState::SharedPtr joint_command_msg_;
  sensor_msgs::msg::JointState joint_state_msg_;

  std::mutex m_;
  std::condition_variable cv_;

  bool is_active_ = false;

  static constexpr double R2D = 180 / M_PI;
  static constexpr double D2R = M_PI / 180;

  void commandReceivedCallback(sensor_msgs::msg::JointState::SharedPtr msg);
};
}  // namespace kuka_rsi_hw_interface

#endif  // KUKA_RSI_HW_INTERFACE__KUKA_HARDWARE_INTERFACE_HPP_
