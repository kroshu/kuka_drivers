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

#ifndef KUKA_KSS_RSI_DRIVER__HARDWARE_INTERFACE_HPP_
#define KUKA_KSS_RSI_DRIVER__HARDWARE_INTERFACE_HPP_

#include <vector>
#include <string>
#include <memory>
#include <chrono>
#include <cmath>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/macros.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "hardware_interface/system_interface.hpp"

#include "kuka_kss_rsi_driver/udp_server.h"
#include "kuka_kss_rsi_driver/rsi_state.h"
#include "kuka_kss_rsi_driver/rsi_command.h"
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
