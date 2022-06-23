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

#ifndef KUKA_RSI_HW_INTERFACE__KUKA_HARDWARE_INTERFACE_HPP_
#define KUKA_RSI_HW_INTERFACE__KUKA_HARDWARE_INTERFACE_HPP_

#include <kuka_rsi_hw_interface/udp_server.h>
#include <kuka_rsi_hw_interface/rsi_state.h>
#include <kuka_rsi_hw_interface/rsi_command.h>

#include <vector>
#include <string>
#include <memory>
#include <chrono>
#include <cmath>
#include <mutex>

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
}  // namespace kuka_rsi_hw_interface

#endif  // KUKA_RSI_HW_INTERFACE__KUKA_HARDWARE_INTERFACE_HPP_
