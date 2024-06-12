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

#include <grpcpp/create_channel.h>
#include <chrono>
#include <stdexcept>
#include <string>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "kuka_drivers_core/control_mode.hpp"
#include "kuka_drivers_core/hardware_interface_types.hpp"

#include "kuka_iiqka_drivers_core/event_observer.hpp"
#include "kuka_kmr_iisy_eac_driver/hardware_interface.hpp"

namespace kuka_eac
{
CallbackReturn KukaKMRiisyEACHardwareInterface::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  // Initialize control mode with 'undefined', which should be changed by the appropriate controller
  // during configuration
  hw_twist_commands_.resize(6, 0.0);
  hw_twist_state_.resize(6, 0.0);
  hw_pose_state_.resize(7, 0.0);
  
  RCLCPP_INFO(
    rclcpp::get_logger("KukaKMRiisyEACHardwareInterface"),
    "Init successful with controller ip: %s and client ip: %s",
    info_.hardware_parameters.at("controller_ip").c_str(),
    info_.hardware_parameters.at("client_ip").c_str());

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> KukaKMRiisyEACHardwareInterface::export_state_interfaces()
{
  RCLCPP_DEBUG(rclcpp::get_logger("KukaKMRiisyEACHardwareInterface"), "Export state interfaces");
  std::vector<hardware_interface::StateInterface> state_interfaces;

  state_interfaces.emplace_back(
    hardware_interface::STATE_PREFIX, hardware_interface::SERVER_STATE, &server_state_);
  state_interfaces.emplace_back(
      std::string(hardware_interface::TWIST_PREFIX), std::string(hardware_interface::LINEAR_PREFIX) + "/" + std::string(hardware_interface::HW_IF_X), &hw_twist_state_[0]);
  state_interfaces.emplace_back(
      std::string(hardware_interface::TWIST_PREFIX), std::string(hardware_interface::LINEAR_PREFIX) + "/" + std::string(hardware_interface::HW_IF_Y), &hw_twist_state_[1]);
  state_interfaces.emplace_back(
      std::string(hardware_interface::TWIST_PREFIX), std::string(hardware_interface::LINEAR_PREFIX) + "/" + std::string(hardware_interface::HW_IF_Z), &hw_twist_state_[2]);
  state_interfaces.emplace_back(
      std::string(hardware_interface::TWIST_PREFIX), std::string(hardware_interface::ANGULAR_PREFIX) + "/" +  std::string(hardware_interface::HW_IF_X), &hw_twist_state_[3]);
  state_interfaces.emplace_back(
      std::string(hardware_interface::TWIST_PREFIX), std::string(hardware_interface::ANGULAR_PREFIX) + "/" +  std::string(hardware_interface::HW_IF_Y), &hw_twist_state_[4]);
  state_interfaces.emplace_back(
      std::string(hardware_interface::TWIST_PREFIX), std::string(hardware_interface::ANGULAR_PREFIX) + "/" +  std::string(hardware_interface::HW_IF_Z), &hw_twist_state_[5]);

  state_interfaces.emplace_back(
      std::string(hardware_interface::POSE_PREFIX) + "/" + std::string(hardware_interface::POSITION_PREFIX),  hardware_interface::HW_IF_X, &hw_pose_state_[0]);
  state_interfaces.emplace_back(
      std::string(hardware_interface::POSE_PREFIX) + "/" + std::string(hardware_interface::POSITION_PREFIX),  hardware_interface::HW_IF_Y, &hw_pose_state_[1]);
  state_interfaces.emplace_back(
      std::string(hardware_interface::POSE_PREFIX) + "/" + std::string(hardware_interface::POSITION_PREFIX),  hardware_interface::HW_IF_Z, &hw_pose_state_[2]);
  state_interfaces.emplace_back(
      std::string(hardware_interface::POSE_PREFIX) + "/" + std::string(hardware_interface::ORIENTATION_PREFIX),  hardware_interface::HW_IF_X, &hw_pose_state_[3]);
  state_interfaces.emplace_back(
      std::string(hardware_interface::POSE_PREFIX) + "/" + std::string(hardware_interface::ORIENTATION_PREFIX),  hardware_interface::HW_IF_Y, &hw_pose_state_[4]);
  state_interfaces.emplace_back(
      std::string(hardware_interface::POSE_PREFIX) + "/" + std::string(hardware_interface::ORIENTATION_PREFIX),  hardware_interface::HW_IF_Z, &hw_pose_state_[5]);
  state_interfaces.emplace_back(
      std::string(hardware_interface::POSE_PREFIX) + "/" + std::string(hardware_interface::ORIENTATION_PREFIX),  hardware_interface::HW_IF_W, &hw_pose_state_[6]);
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
KukaKMRiisyEACHardwareInterface::export_command_interfaces()
{
  RCLCPP_DEBUG(rclcpp::get_logger("KukaKMRiisyEACHardwareInterface"), "Export command interfaces");

  std::vector<hardware_interface::CommandInterface> command_interfaces;
  command_interfaces.emplace_back(
    hardware_interface::CONFIG_PREFIX, hardware_interface::CONTROL_MODE, &hw_control_mode_command_);

  command_interfaces.emplace_back(
      std::string(hardware_interface::TWIST_PREFIX), std::string(hardware_interface::LINEAR_PREFIX) + "/" + std::string(hardware_interface::HW_IF_X), &hw_twist_commands_[0]);
  command_interfaces.emplace_back( 
      std::string(hardware_interface::TWIST_PREFIX), std::string(hardware_interface::LINEAR_PREFIX) + "/" + std::string(hardware_interface::HW_IF_Y), &hw_twist_commands_[1]);
  command_interfaces.emplace_back(
      std::string(hardware_interface::TWIST_PREFIX), std::string(hardware_interface::LINEAR_PREFIX) + "/" + std::string(hardware_interface::HW_IF_Z), &hw_twist_commands_[2]);
  command_interfaces.emplace_back(
      std::string(hardware_interface::TWIST_PREFIX), std::string(hardware_interface::ANGULAR_PREFIX) + "/" + std::string(hardware_interface::HW_IF_X), &hw_twist_commands_[3]);
  command_interfaces.emplace_back(
      std::string(hardware_interface::TWIST_PREFIX), std::string(hardware_interface::ANGULAR_PREFIX) + "/" + std::string(hardware_interface::HW_IF_Y), &hw_twist_commands_[4]);  
  command_interfaces.emplace_back(
      std::string(hardware_interface::TWIST_PREFIX), std::string(hardware_interface::ANGULAR_PREFIX) + "/" + std::string(hardware_interface::HW_IF_Z), &hw_twist_commands_[5]);
  return command_interfaces;
}


return_type KukaKMRiisyEACHardwareInterface::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  // Bigger timeout blocks controller configuration
  kuka::external::control::Status receive_state =
    robot_ptr_->ReceiveMotionState(std::chrono::milliseconds(10));

  if ((msg_received_ = receive_state.return_code == kuka::external::control::ReturnCode::OK))
  {
    auto & req_message = robot_ptr_->GetLastMotionState();

      std::copy(
      req_message.GetMeasuredTwist().begin(), req_message.GetMeasuredTwist().end(),
      hw_twist_state_.begin());
    
      std::copy(
      req_message.GetMeasuredCartesianPositions().begin(), req_message.GetMeasuredCartesianPositions().end(),
      hw_pose_state_.begin());
    
    

      cycle_count_++;
    }
  tf2::Quaternion q(
    hw_pose_state_[0],
    hw_pose_state_[1],
    hw_pose_state_[2],
    hw_pose_state_[3]);
  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll,pitch,yaw);
  
  RCLCPP_DEBUG( rclcpp::get_logger(
        "KukaRoXHardwareInterface"), "STATE position: %f, %f, %f",
            hw_pose_state_[0],
            hw_pose_state_[1],
            yaw);
  RCLCPP_DEBUG( rclcpp::get_logger(
        "KukaRoXHardwareInterface"), "STATE velocity: x:%f, y:%f, theta:%f",
            hw_twist_state_[0],
            hw_twist_state_[1],
            hw_twist_state_[5]);

  // Modify state interface only in read
  std::lock_guard<std::mutex> lk(event_mutex_);
  server_state_ = static_cast<double>(last_event_);
  
  return return_type::OK;
}

return_type KukaKMRiisyEACHardwareInterface::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  // If control is not started or a request is missed, do not send back anything
  if (!msg_received_)
  {
    return return_type::OK;
  }

  robot_ptr_->GetControlSignal().AddTwistValues(hw_twist_commands_.begin(),hw_twist_commands_.end());


  kuka::external::control::Status send_reply;
  if (stop_requested_)
  {
    RCLCPP_INFO(rclcpp::get_logger("KukaKMRiisyEACHardwareInterface"), "Sending stop signal");
    send_reply = robot_ptr_->StopControlling();
  }
  else if (
    static_cast<kuka_drivers_core::ControlMode>(hw_control_mode_command_) != prev_control_mode_)
  {
    RCLCPP_INFO(rclcpp::get_logger("KukaKMRiisyEACHardwareInterface"), "Requesting control mode switch");
    send_reply = robot_ptr_->SwitchControlMode(
      static_cast<kuka::external::control::ControlMode>(hw_control_mode_command_));
    prev_control_mode_ = static_cast<kuka_drivers_core::ControlMode>(hw_control_mode_command_);
  }
  else
  {
    send_reply = robot_ptr_->SendControlSignal();
  }
  if (send_reply.return_code != kuka::external::control::ReturnCode::OK)
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("KukaKMRiisyEACHardwareInterface"), "Send reply failed, error message: %s",
      send_reply.message);
    throw std::runtime_error("Error sending reply");
  }
  return return_type::OK;
}

bool KukaKMRiisyEACHardwareInterface::SetupRobot()
{
  kuka::external::control::iiqka::Configuration config;

  config.client_ip_address = info_.hardware_parameters.at("client_ip");
  config.koni_ip_address = info_.hardware_parameters.at("controller_ip");

  config.is_secure = false;
  config.dof = info_.joints.size();

  robot_ptr_ = std::make_unique<kuka::external::control::iiqka::Robot>(config);

  kuka::external::control::Status setup = robot_ptr_->Setup();

  if (setup.return_code != kuka::external::control::ReturnCode::OK)
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("KukaKMRiisyEACHardwareInterface"), "Setup failed, error message: %s",
      setup.message);
    return false;
  }

  return true;
}




}  // namespace kuka_eac

PLUGINLIB_EXPORT_CLASS(kuka_eac::KukaKMRiisyEACHardwareInterface, hardware_interface::SystemInterface)
