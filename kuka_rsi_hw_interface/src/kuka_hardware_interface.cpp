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
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace kuka_rsi_hw_interface
{

// KukaRSIHardwareInterface::KukaRSIHardwareInterface(
//   const std::string & rsi_ip_address, int rsi_port, uint8_t n_dof)
// : rsi_ip_address_(rsi_ip_address),
//   rsi_port_(rsi_port),
//   n_dof_(n_dof)
// {
//   in_buffer_.resize(1024);
//   out_buffer_.resize(1024);
// }


CallbackReturn KukaRSIHardwareInterface::on_init(const hardware_interface::HardwareInfo & info)
{
	RCLCPP_INFO(rclcpp::get_logger("KukaRSIHardwareInterface"), "on_init()");

	if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
		return CallbackReturn::ERROR;
	}

	hw_states_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
	hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

	for (const hardware_interface::ComponentInfo &joint : info_.joints) {

		if(joint.command_interfaces.size() != 1) {
			RCLCPP_FATAL(rclcpp::get_logger("KukaRSIHardwareInterface"), "expecting exactly 1 command interface");
			return CallbackReturn::ERROR;
		}

		if(joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
			RCLCPP_FATAL(rclcpp::get_logger("KukaRSIHardwareInterface"), "expecting only POSITION command interface");
			return CallbackReturn::ERROR;
		}

		if(joint.state_interfaces.size() != 1) {
			RCLCPP_FATAL(rclcpp::get_logger("KukaRSIHardwareInterface"), "expecting exactly 1 state interface");
			return CallbackReturn::ERROR;
		}

		if(joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
			RCLCPP_FATAL(rclcpp::get_logger("KukaRSIHardwareInterface"), "expecting only POSITION state interface");
			return CallbackReturn::ERROR;
		}

	}

	//RSI
	in_buffer_.resize(1024); //udp_server.h --> #define BUFSIZE 1024
	out_buffer_.resize(1024);

	initial_joint_pos_.resize(n_dof_, 0.0);
	joint_pos_correction_deg_.resize(n_dof_, 0.0),
	ipoc_ = 0;

	rsi_ip_address_ = info_.hardware_parameters["rsi_ip_address"];
	rsi_port_ = std::stoi(info_.hardware_parameters["rsi_port"]);

	RCLCPP_INFO(rclcpp::get_logger("KukaRSIHardwareInterface"),
	            "robot location: %s:%d", rsi_ip_address_.c_str(), rsi_port_);

	//done
	// return return_type::OK;
	//lifecycle_state_ = rclcpp_lifecycle::State::CONFIGURED;
	return CallbackReturn::SUCCESS;
}


CallbackReturn KukaRSIHardwareInterface::on_configure(const rclcpp_lifecycle::State & previous_state)
{
	RCLCPP_INFO(rclcpp::get_logger("KukaRSIHardwareInterface"), "on_configure()");

	//just in case - not 100% sure this is the right thing to do . . .
	for (size_t i = 0; i < hw_states_.size(); ++i) {
		hw_states_[i] = std::numeric_limits<double>::quiet_NaN();
		hw_commands_[i] = std::numeric_limits<double>::quiet_NaN();
		initial_joint_pos_[i] = 0.0;
		joint_pos_correction_deg_[i] = 0.0;
	}

	return CallbackReturn::SUCCESS;
}


std::vector<hardware_interface::StateInterface> KukaRSIHardwareInterface::export_state_interfaces()
{
	RCLCPP_INFO(rclcpp::get_logger("KukaRSIHardwareInterface"), "export_state_interfaces()");

	std::vector<hardware_interface::StateInterface> state_interfaces;
	for(size_t i=0; i<info_.joints.size(); i++) {
		state_interfaces.emplace_back(
			hardware_interface::StateInterface(
				info_.joints[i].name,
				hardware_interface::HW_IF_POSITION,
				&hw_states_[i]));
	}
	return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> KukaRSIHardwareInterface::export_command_interfaces()
{
	RCLCPP_INFO(rclcpp::get_logger("KukaRSIHardwareInterface"), "export_command_interfaces()");

	std::vector<hardware_interface::CommandInterface> command_interfaces;
	for(size_t i=0; i<info_.joints.size(); i++) {
		command_interfaces.emplace_back(
			hardware_interface::CommandInterface(
				info_.joints[i].name,
				hardware_interface::HW_IF_POSITION,
				&hw_commands_[i]));
	}
	return command_interfaces;
}

CallbackReturn KukaRSIHardwareInterface::on_activate(const rclcpp_lifecycle::State & previous_state)
{
	RCLCPP_INFO(rclcpp::get_logger("KukaRSIHardwareInterface"),     "on_activate()");

	// Wait for connection from robot
	server_.reset(new UDPServer(rsi_ip_address_, rsi_port_));

	RCLCPP_INFO(rclcpp::get_logger("KukaRSIHardwareInterface"),     "Connecting to robot . . .");

	int bytes = server_->recv(in_buffer_);

	RCLCPP_INFO(rclcpp::get_logger("KukaRSIHardwareInterface"),     "got some bytes");

	// Drop empty <rob> frame with RSI <= 2.3
	if(bytes < 100) {
		bytes = server_->recv(in_buffer_);
	}

	if(bytes < 100) {
		RCLCPP_FATAL(rclcpp::get_logger("KukaRSIHardwareInterface"), "not enough data received");
		return CallbackReturn::ERROR;
	}

	rsi_state_ = RSIState(in_buffer_);

	for (size_t i = 0; i < n_dof_; ++i) {
		hw_states_[i] = rsi_state_.positions[i] * KukaRSIHardwareInterface::D2R;
		hw_commands_[i] = hw_states_[i];
		initial_joint_pos_[i] = rsi_state_.initial_positions[i] * KukaRSIHardwareInterface::D2R;
		// 		joint_state_msg_position[i] = rsi_state_.positions[i] * KukaRSIHardwareInterface::D2R;
	}
	ipoc_ = rsi_state_.ipoc;

	out_buffer_ = RSICommand(joint_pos_correction_deg_, ipoc_).xml_doc;
	server_->send(out_buffer_);
	server_->set_timeout(1000); // Set receive timeout to 1 second

	RCLCPP_INFO(rclcpp::get_logger("KukaRSIHardwareInterface"), "System Sucessfully started!");
	is_active_ = true;

	// status_ = hardware_interface::status::STARTED;
	// return return_type::OK;
	return CallbackReturn::SUCCESS;
}

CallbackReturn KukaRSIHardwareInterface::on_deactivate(const rclcpp_lifecycle::State & previous_state)
{
	RCLCPP_INFO(rclcpp::get_logger("KukaRSIHardwareInterface"),     "on_deactivate()");
	out_buffer_ = RSICommand(joint_pos_correction_deg_, ipoc_, true).xml_doc;
	server_->send(out_buffer_);
	server_.reset();
	is_active_ = false;
	RCLCPP_INFO(rclcpp::get_logger("KukaRSIHardwareInterface"),     "System sucessfully stopped!");
	return CallbackReturn::SUCCESS;
}


// void KukaRSIHardwareInterface::start(std::vector<double> & joint_state_msg_position)
// {
// 	std::lock_guard<std::mutex> lock(m_);
// 	// Wait for connection from robot
// 	server_.reset(new UDPServer(rsi_ip_address_, rsi_port_));
// 	// TODO(Marton): use any logger
// 	std::cout << "Waiting for robot connection\n";
// 	int bytes = server_->recv(in_buffer_);

// 	// Drop empty <rob> frame with RSI <= 2.3
// 	if (bytes < 100) {
// 		bytes = server_->recv(in_buffer_);
// 	}

// 	rsi_state_ = RSIState(in_buffer_);
// 	for (std::size_t i = 0; i < n_dof_; ++i) {
// 		joint_state_msg_position[i] = rsi_state_.positions[i] * KukaRSIHardwareInterface::D2R;
// 		initial_joint_pos_[i] = rsi_state_.initial_positions[i] * KukaRSIHardwareInterface::D2R;
// 	}
// 	ipoc_ = rsi_state_.ipoc;
// 	out_buffer_ = RSICommand(std::vector<double>(n_dof_, 0), ipoc_).xml_doc;
// 	server_->send(out_buffer_);
// 	// Set receive timeout to 1 second
// 	server_->set_timeout(1000);
// 	// TODO(Marton): use any logger
// 	std::cout << "Got connection from robot\n";
// 	is_active_ = true;
// }

return_type KukaRSIHardwareInterface::read(const rclcpp::Time & time, const rclcpp::Duration & period)
{
	std::lock_guard<std::mutex> lock(m_);
	if (!is_active_) {
		return return_type::ERROR;
	}

	if (server_->recv(in_buffer_) == 0) {
		return return_type::ERROR;
	}
	rsi_state_ = RSIState(in_buffer_);

	for (std::size_t i = 0; i < n_dof_; ++i) {
		hw_states_[i] = rsi_state_.positions[i] * KukaRSIHardwareInterface::D2R;
		RCLCPP_INFO(rclcpp::get_logger("KukaRSIHardwareInterface"), "Got state %.5f for joint %ld!", hw_states_[i], i);
		// joint_state_msg_position[i] = rsi_state_.positions[i] * KukaRSIHardwareInterface::D2R;
	}
	ipoc_ = rsi_state_.ipoc;
	return return_type::OK;
}

return_type KukaRSIHardwareInterface::write(const rclcpp::Time & time, const rclcpp::Duration & period)
{
	RCLCPP_INFO(rclcpp::get_logger("KukaRSIHardwareInterface"), "write()");

	std::lock_guard<std::mutex> lock(m_);
	if (!is_active_) {
		RCLCPP_INFO(rclcpp::get_logger("KukaRSIHardwareInterface"), "Controller deactivated");
		return return_type::ERROR;
	}

	for (size_t i = 0; i < n_dof_; i++) {
		RCLCPP_INFO(rclcpp::get_logger("KukaRSIHardwareInterface"), "Got command %.5f for joint %ld!",hw_commands_[i], i);
		joint_pos_correction_deg_[i] = (hw_commands_[i] - initial_joint_pos_[i]) * KukaRSIHardwareInterface::R2D;
	}

	out_buffer_ = RSICommand(joint_pos_correction_deg_, ipoc_).xml_doc;
	server_->send(out_buffer_);
	return return_type::OK;

}

// void KukaRSIHardwareInterface::stop()
// {
// 	std::lock_guard<std::mutex> lock(m_);
// 	out_buffer_ = RSICommand(joint_pos_correction_deg_, ipoc_, true).xml_doc;
// 	server_->send(out_buffer_);
// 	server_.reset();
// 	is_active_ = false;
// 	std::cout << "Connection to robot terminated\n";
// }

bool KukaRSIHardwareInterface::isActive() const
{
	return is_active_;
}

}  // namespace namespace kuka_rsi_hw_interface

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
	kuka_rsi_hw_interface::KukaRSIHardwareInterface,
	hardware_interface::SystemInterface
	)

