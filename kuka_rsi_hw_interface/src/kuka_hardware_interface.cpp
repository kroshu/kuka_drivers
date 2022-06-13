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
#include <stdexcept>
#include <string>
#include <memory>
#include <vector>

#include "kuka_rsi_hw_interface/kuka_hardware_interface.hpp"

namespace kuka_rsi_hw_interface
{

KukaHardwareInterface::KukaHardwareInterface(
  const std::string & rsi_ip_address, int rsi_port, unsigned int n_dof)
: rsi_ip_address_(rsi_ip_address),
  rsi_port_(rsi_port),
  n_dof_(n_dof)
{
  in_buffer_.resize(1024);
  out_buffer_.resize(1024);
}

void KukaHardwareInterface::start(
  std::vector<double> & joint_state_msg_position,
  std::vector<double> & joint_command_msg_position,
  std::vector<double> & initial_joint_position)
{
  std::unique_lock<std::mutex> lock(m_);
  // Wait for connection from robot
  server_.reset(new UDPServer(rsi_ip_address_, rsi_port_));
  // TODO(Marton): use any logger
  std::cout << "Waiting for robot connection\n";
  int bytes = server_->recv(in_buffer_);

  // Drop empty <rob> frame with RSI <= 2.3
  if (bytes < 100) {
    bytes = server_->recv(in_buffer_);
  }

  rsi_state_ = RSIState(in_buffer_);
  for (std::size_t i = 0; i < n_dof_; ++i) {
    joint_state_msg_position[i] = rsi_state_.positions[i] * KukaHardwareInterface::D2R;
    joint_command_msg_position[i] = joint_state_msg_position[i];
    initial_joint_position[i] = rsi_state_.initial_positions[i] * KukaHardwareInterface::D2R;
  }
  ipoc_ = rsi_state_.ipoc;
  out_buffer_ = RSICommand(std::vector<double>(n_dof_, 0), ipoc_).xml_doc;
  server_->send(out_buffer_);
  // Set receive timeout to 1 second
  server_->set_timeout(1000);
  // TODO(Marton): use any logger
  std::cout << "Got connection from robot\n";
  is_active_ = true;
}

bool KukaHardwareInterface::read(std::vector<double> & joint_state_msg_position)
{
  std::unique_lock<std::mutex> lock(m_);
  if (!is_active_) {
    return false;
  }
  in_buffer_.resize(1024);
  if (server_->recv(in_buffer_) == 0) {
    return false;
  }
  rsi_state_ = RSIState(in_buffer_);
  for (std::size_t i = 0; i < n_dof_; ++i) {
    joint_state_msg_position[i] = rsi_state_.positions[i] * KukaHardwareInterface::D2R;
  }
  ipoc_ = rsi_state_.ipoc;
  return true;
}

bool KukaHardwareInterface::write(const std::vector<double> & joint_pos_correction_deg_)
{
  out_buffer_.resize(1024);   // TODO(Svastits): is this necessary?
  std::unique_lock<std::mutex> lock(m_);
  if (!is_active_) {
    std::cout << "Controller deactivated\n";
    return false;
  }
  out_buffer_ = RSICommand(joint_pos_correction_deg_, ipoc_).xml_doc;
  server_->send(out_buffer_);
  return true;
}

void KukaHardwareInterface::stop(const std::vector<double> & joint_pos_correction_deg_)
{
  std::lock_guard<std::mutex> lock(m_);
  out_buffer_ = RSICommand(joint_pos_correction_deg_, ipoc_, true).xml_doc;
  server_->send(out_buffer_);
  server_.reset();
  is_active_ = false;
  std::cout << "Connection to robot terminated\n";
}

bool KukaHardwareInterface::isActive() const
{
  return is_active_;
}

}  // namespace namespace kuka_rsi_hw_interface
