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
CallbackReturn KukaRSIHardwareInterface::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  command_handler_ = RSICommandHandler();

  hw_states_.resize(info_.joints.size(), 0.0);
  hw_commands_.resize(info_.joints.size(), 0.0);

  for (const hardware_interface::ComponentInfo & joint : info_.joints) {
    if (joint.command_interfaces.size() != 1) {
      RCLCPP_FATAL(
        rclcpp::get_logger(
          "KukaRSIHardwareInterface"), "expecting exactly 1 command interface");
      return CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
      RCLCPP_FATAL(
        rclcpp::get_logger(
          "KukaRSIHardwareInterface"), "expecting only POSITION command interface");
      return CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 1) {
      RCLCPP_FATAL(
        rclcpp::get_logger(
          "KukaRSIHardwareInterface"), "expecting exactly 1 state interface");
      return CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
      RCLCPP_FATAL(
        rclcpp::get_logger(
          "KukaRSIHardwareInterface"), "expecting only POSITION state interface");
      return CallbackReturn::ERROR;
    }
  }

  // RSI

  initial_joint_pos_.resize(info_.joints.size(), 0.0);
  joint_pos_correction_deg_.resize(info_.joints.size(), 0.0);
  ipoc_ = 0;

  rsi_ip_address_ = info_.hardware_parameters["client_ip"];
  rsi_port_ = std::stoi(info_.hardware_parameters["client_port"]);

  RCLCPP_INFO(
    rclcpp::get_logger("KukaRSIHardwareInterface"),
    "IP of client machine: %s:%d", rsi_ip_address_.c_str(), rsi_port_);

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> KukaRSIHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < info_.joints.size(); i++) {
    state_interfaces.emplace_back(
      info_.joints[i].name,
      hardware_interface::HW_IF_POSITION,
      &hw_states_[i]);
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> KukaRSIHardwareInterface::
export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < info_.joints.size(); i++) {
    command_interfaces.emplace_back(
      info_.joints[i].name,
      hardware_interface::HW_IF_POSITION,
      &hw_commands_[i]);
  }
  return command_interfaces;
}

CallbackReturn KukaRSIHardwareInterface::on_activate(const rclcpp_lifecycle::State &)
{
  stop_flag_ = false;
  // Wait for connection from robot
  server_.reset(new UDPServer(rsi_ip_address_, rsi_port_));
  server_->set_timeout(10000);  // Set receive timeout to 10 seconds for activation


  RCLCPP_INFO(rclcpp::get_logger("KukaRSIHardwareInterface"), "Connecting to robot . . .");

  int bytes = server_->recv(in_buffer_);
  if (bytes == 0) {
    RCLCPP_ERROR(rclcpp::get_logger("KukaRSIHardwareInterface"), "Connection timeout");
    return CallbackReturn::FAILURE;
  }

  RCLCPP_INFO(rclcpp::get_logger("KukaRSIHardwareInterface"), "Got data from robot");

  // Drop empty <rob> frame with RSI <= 2.3
  if (bytes < 100) {
    bytes = server_->recv(in_buffer_);
  }

  if (!command_handler_.Decode(in_buffer_, UDP_BUFFER_SIZE)) {
    RCLCPP_ERROR(rclcpp::get_logger("KukaRSIHardwareInterface"), "Decode failed");
    return CallbackReturn::FAILURE;
  }

  // Position data
  hw_states_[0] = command_handler_.GetState().GetElement("AIPos")->GetParam<double>("A1") *
    KukaRSIHardwareInterface::D2R;
  hw_states_[1] = command_handler_.GetState().GetElement("AIPos")->GetParam<double>("A2") *
    KukaRSIHardwareInterface::D2R;
  hw_states_[2] = command_handler_.GetState().GetElement("AIPos")->GetParam<double>("A3") *
    KukaRSIHardwareInterface::D2R;
  hw_states_[3] = command_handler_.GetState().GetElement("AIPos")->GetParam<double>("A4") *
    KukaRSIHardwareInterface::D2R;
  hw_states_[4] = command_handler_.GetState().GetElement("AIPos")->GetParam<double>("A5") *
    KukaRSIHardwareInterface::D2R;
  hw_states_[5] = command_handler_.GetState().GetElement("AIPos")->GetParam<double>("A6") *
    KukaRSIHardwareInterface::D2R;

  // Initial position data
  initial_joint_pos_[0] = command_handler_.GetState().GetElement("ASPos")->GetParam<double>("A1") *
    KukaRSIHardwareInterface::D2R;
  initial_joint_pos_[1] = command_handler_.GetState().GetElement("ASPos")->GetParam<double>("A2") *
    KukaRSIHardwareInterface::D2R;
  initial_joint_pos_[2] = command_handler_.GetState().GetElement("ASPos")->GetParam<double>("A3") *
    KukaRSIHardwareInterface::D2R;
  initial_joint_pos_[3] = command_handler_.GetState().GetElement("ASPos")->GetParam<double>("A4") *
    KukaRSIHardwareInterface::D2R;
  initial_joint_pos_[4] = command_handler_.GetState().GetElement("ASPos")->GetParam<double>("A5") *
    KukaRSIHardwareInterface::D2R;
  initial_joint_pos_[5] = command_handler_.GetState().GetElement("ASPos")->GetParam<double>("A6") *
    KukaRSIHardwareInterface::D2R;

  // Ipoc data
  ipoc_ = command_handler_.GetState().GetElement("IPOC")->GetParam<long>("IPOC");

  for (size_t i = 0; i < info_.joints.size(); ++i) {
    // hw_states_[i] = rsi_state_.positions[i] * KukaRSIHardwareInterface::D2R;
    hw_commands_[i] = hw_states_[i];
    // initial_joint_pos_[i] = rsi_state_.initial_positions[i] * KukaRSIHardwareInterface::D2R;
  }
  // ipoc_ = rsi_state_.ipoc;

  // Initial command pos data
  command_handler_.SetCommandParam<double>("AK", "A1", joint_pos_correction_deg_[0]);
  command_handler_.SetCommandParam<double>("AK", "A2", joint_pos_correction_deg_[1]);
  command_handler_.SetCommandParam<double>("AK", "A3", joint_pos_correction_deg_[2]);
  command_handler_.SetCommandParam<double>("AK", "A4", joint_pos_correction_deg_[3]);
  command_handler_.SetCommandParam<double>("AK", "A5", joint_pos_correction_deg_[4]);
  command_handler_.SetCommandParam<double>("AK", "A6", joint_pos_correction_deg_[5]);
  command_handler_.SetCommandParam<bool>("Stop", "Stop", stop_flag_);
  command_handler_.SetCommandParam<long>("IPOC", "IPOC", static_cast<long>(ipoc_));

  //TODO (Komáromi): construct and send msg
  auto out_buffer_it = out_buffer_;
  if (command_handler_.Encode(out_buffer_it, UDP_BUFFER_SIZE) < 0) {
    RCLCPP_ERROR(rclcpp::get_logger("KukaRSIHardwareInterface"), "Encode Failed");
    return CallbackReturn::FAILURE;
  }
  // out_buffer_ = RSICommand(joint_pos_correction_deg_, ipoc_, stop_flag_).xml_doc;
  server_->send(out_buffer_);
  server_->set_timeout(1000);  // Set receive timeout to 1 second

  RCLCPP_INFO(rclcpp::get_logger("KukaRSIHardwareInterface"), "System Successfully started!");
  is_active_ = true;

  return CallbackReturn::SUCCESS;
}

CallbackReturn KukaRSIHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  stop_flag_ = true;
  RCLCPP_INFO(rclcpp::get_logger("KukaRSIHardwareInterface"), "Stop flag was set!");
  return CallbackReturn::SUCCESS;
}

return_type KukaRSIHardwareInterface::read(
  const rclcpp::Time &,
  const rclcpp::Duration &)
{
  if (!is_active_) {
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
    return return_type::OK;
  }

  if (server_->recv(in_buffer_) == 0) {
    RCLCPP_ERROR(rclcpp::get_logger("KukaRSIHardwareInterface"), "No data received from robot");
    this->on_deactivate(this->get_state());
    return return_type::ERROR;
  }
  if (!command_handler_.Decode(in_buffer_, UDP_BUFFER_SIZE)) {
    this->on_deactivate(this->get_state());
    return return_type::ERROR;
  }

  // for (std::size_t i = 0; i < info_.joints.size(); ++i) {
  //   hw_states_[i] = rsi_state_.positions[i] * KukaRSIHardwareInterface::D2R;
  // }
  hw_states_[0] = command_handler_.GetState().GetElement("AIPos")->GetParam<double>("A1") *
    KukaRSIHardwareInterface::D2R;
  hw_states_[1] = command_handler_.GetState().GetElement("AIPos")->GetParam<double>("A2") *
    KukaRSIHardwareInterface::D2R;
  hw_states_[2] = command_handler_.GetState().GetElement("AIPos")->GetParam<double>("A3") *
    KukaRSIHardwareInterface::D2R;
  hw_states_[3] = command_handler_.GetState().GetElement("AIPos")->GetParam<double>("A4") *
    KukaRSIHardwareInterface::D2R;
  hw_states_[4] = command_handler_.GetState().GetElement("AIPos")->GetParam<double>("A5") *
    KukaRSIHardwareInterface::D2R;
  hw_states_[5] = command_handler_.GetState().GetElement("AIPos")->GetParam<double>("A6") *
    KukaRSIHardwareInterface::D2R;
  // ipoc_ = rsi_state_.ipoc;
  ipoc_ = command_handler_.GetState().GetElement("IPOC")->GetParam<long>("IPOC");
  return return_type::OK;
}

return_type KukaRSIHardwareInterface::write(
  const rclcpp::Time &,
  const rclcpp::Duration &)
{
  // It is possible, that write is called immediately after activation
  // In this case write in that tick should be skipped to be able to read state at first
  // First cycle (with 0 ipoc) is handled in the on_activate method, so 0 ipoc means
  //  read was not called yet
  if (!is_active_ || ipoc_ == 0) {
    return return_type::OK;
  }

  if (stop_flag_) {is_active_ = false;}

  for (size_t i = 0; i < info_.joints.size(); i++) {
    joint_pos_correction_deg_[i] = (hw_commands_[i] - initial_joint_pos_[i]) *
      KukaRSIHardwareInterface::R2D;
  }

  command_handler_.SetCommandParam<double>("AK", "A1", joint_pos_correction_deg_[0]);
  command_handler_.SetCommandParam<double>("AK", "A2", joint_pos_correction_deg_[1]);
  command_handler_.SetCommandParam<double>("AK", "A3", joint_pos_correction_deg_[2]);
  command_handler_.SetCommandParam<double>("AK", "A4", joint_pos_correction_deg_[3]);
  command_handler_.SetCommandParam<double>("AK", "A5", joint_pos_correction_deg_[4]);
  command_handler_.SetCommandParam<double>("AK", "A6", joint_pos_correction_deg_[5]);
  command_handler_.SetCommandParam<bool>("Stop", "Stop", stop_flag_);
  command_handler_.SetCommandParam<long>("IPOC", "IPOC", static_cast<long>(ipoc_));

  // out_buffer_ = RSICommand(joint_pos_correction_deg_, ipoc_, stop_flag_).xml_doc;
  auto out_buffer_it = out_buffer_;
  if (command_handler_.Encode(out_buffer_it, UDP_BUFFER_SIZE) < 0) {
    this->on_deactivate(this->get_state());
    return return_type::ERROR;
  }
  server_->send(out_buffer_);
  return return_type::OK;
}
}  // namespace namespace kuka_rsi_hw_interface

PLUGINLIB_EXPORT_CLASS(
  kuka_rsi_hw_interface::KukaRSIHardwareInterface,
  hardware_interface::SystemInterface
)
