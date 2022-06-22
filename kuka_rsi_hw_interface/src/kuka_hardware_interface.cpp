/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014 Norwegian University of Science and Technology
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Norwegian University of Science and
 *     Technology, nor the names of its contributors may be used to
 *     endorse or promote products derived from this software without
 *     specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/*
 * Author: Lars Tingelstad
 * Author: Svastits Aron
 */

#include <stdexcept>
#include <string>
#include <memory>
#include <vector>

#include "kuka_rsi_hw_interface/kuka_hardware_interface.hpp"

namespace kuka_rsi_hw_interface
{

KukaHardwareInterface::KukaHardwareInterface(
  const std::string & rsi_ip_address, int rsi_port, uint8_t n_dof)
: rsi_ip_address_(rsi_ip_address),
  rsi_port_(rsi_port),
  n_dof_(n_dof)
{
  in_buffer_.resize(1024);
  out_buffer_.resize(1024);
}

void KukaHardwareInterface::start(std::vector<double> & joint_state_msg_position)
{
  std::lock_guard<std::mutex> lock(m_);
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
    initial_joint_pos_[i] = rsi_state_.initial_positions[i] * KukaHardwareInterface::D2R;
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
  std::lock_guard<std::mutex> lock(m_);
  if (!is_active_) {
    return false;
  }

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

bool KukaHardwareInterface::write(const std::vector<double> & joint_command_position_msg_)
{
  std::lock_guard<std::mutex> lock(m_);
  if (!is_active_) {
    std::cout << "Controller deactivated\n";
    return false;
  }

  for (size_t i = 0; i < n_dof_; i++) {
    joint_pos_correction_deg_[i] = (joint_command_position_msg_[i] - initial_joint_pos_[i]) *
      KukaHardwareInterface::R2D;
  }

  out_buffer_ = RSICommand(joint_pos_correction_deg_, ipoc_).xml_doc;
  server_->send(out_buffer_);
  return true;
}

void KukaHardwareInterface::stop()
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
