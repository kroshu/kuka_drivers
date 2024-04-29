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

#include "kuka_iiqka_eac_driver/event_observer.hpp"
#include "kuka_iiqka_eac_driver/hardware_interface.hpp"

namespace kuka_eac
{
CallbackReturn KukaEACHardwareInterface::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  // Initialize control mode with 'undefined', which should be changed by the appropriate controller
  // during configuration
  hw_position_states_.resize(info_.joints.size(), 0.0);
  hw_torque_states_.resize(info_.joints.size(), 0.0);
  hw_position_commands_.resize(info_.joints.size(), 0.0);
  hw_torque_commands_.resize(info_.joints.size(), 0.0);
  hw_stiffness_commands_.resize(info_.joints.size(), 30);
  hw_damping_commands_.resize(info_.joints.size(), 0.7);
  hw_twist_commands_vector_.resize(6, 0.0);
   
if (info_.name=="KUKA_MR" ){
  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    if (joint.command_interfaces.size() != 3)
        {
          RCLCPP_FATAL(
            rclcpp::get_logger("KukaEACHardwareInterface"), "expecting exactly 3 command interface");
          return CallbackReturn::ERROR;
        }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_X)
        {
          RCLCPP_FATAL(
            rclcpp::get_logger("KukaEACHardwareInterface"),
            "expecting 'X' command interface as first");
          return CallbackReturn::ERROR;
        }
    if (joint.command_interfaces[1].name != hardware_interface::HW_IF_Y)
        {
          RCLCPP_FATAL(
            rclcpp::get_logger("KukaEACHardwareInterface"),
            "expecting 'Y' command interface as second");
          return CallbackReturn::ERROR;
        }
    if (joint.command_interfaces[2].name != hardware_interface::HW_IF_Z)
        {
          RCLCPP_FATAL(
            rclcpp::get_logger("KukaEACHardwareInterface"),
            "expecting 'Z' command interface as third");
          return CallbackReturn::ERROR;
        }
    if (joint.state_interfaces.size() != 3)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("KukaEACHardwareInterface"), "expecting exactly 3 state interface");
      return CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_X)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("KukaEACHardwareInterface"),
        "expecting 'X' state interface as first");
      return CallbackReturn::ERROR;
    }
    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_Y)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("KukaEACHardwareInterface"),
        "expecting 'Y' state interface as second");
      return CallbackReturn::ERROR;
    }
    if (joint.state_interfaces[2].name != hardware_interface::HW_IF_Z)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("KukaEACHardwareInterface"),
        "expecting 'Z' state interface as third");
      return CallbackReturn::ERROR;
    }
  }
}
else {
  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    if (joint.command_interfaces.size() != 4)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("KukaEACHardwareInterface"), "expecting exactly 4 command interface");
      return CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("KukaEACHardwareInterface"),
        "expecting 'POSITION' command interface as first");
      return CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[1].name != hardware_interface::HW_IF_STIFFNESS)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("KukaEACHardwareInterface"),
        "expecting 'STIFFNESS' command interface as second");
      return CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[2].name != hardware_interface::HW_IF_DAMPING)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("KukaEACHardwareInterface"),
        "expecting 'DAMPING' command interface as third");
      return CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[3].name != hardware_interface::HW_IF_EFFORT)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("KukaEACHardwareInterface"),
        "expecting 'EFFORT' command interface as fourth");
      return CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 2)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("KukaEACHardwareInterface"), "expecting exactly 2 state interface");
      return CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("KukaEACHardwareInterface"),
        "expecting 'POSITION' state interface as first");
      return CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_EFFORT)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("KukaEACHardwareInterface"),
        "expecting 'EFFORT' state interface as second");
      return CallbackReturn::ERROR;
    }
  }
}
  RCLCPP_INFO(
    rclcpp::get_logger("KukaEACHardwareInterface"),
    "Init successful with controller ip: %s and client ip: %s",
    info_.hardware_parameters.at("controller_ip").c_str(),
    info_.hardware_parameters.at("client_ip").c_str());

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> KukaEACHardwareInterface::export_state_interfaces()
{
  RCLCPP_DEBUG(rclcpp::get_logger("KukaEACHardwareInterface"), "Export state interfaces");
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_position_states_[i]);

    state_interfaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_torque_states_[i]);
  }

  state_interfaces.emplace_back(
    hardware_interface::STATE_PREFIX, hardware_interface::SERVER_STATE, &server_state_);
  state_interfaces.emplace_back(
      info_.joints[0].name, "x", &hw_twist_state_.linear.x);
  state_interfaces.emplace_back(
      info_.joints[0].name, "y",  &hw_twist_state_.linear.y);
  state_interfaces.emplace_back(
      info_.joints[0].name, "z", &hw_twist_state_.linear.z);
  state_interfaces.emplace_back(
      info_.joints[1].name, "x", &hw_twist_state_.angular.x);
  state_interfaces.emplace_back(
      info_.joints[1].name, "y", &hw_twist_state_.angular.y);
  state_interfaces.emplace_back(
      info_.joints[1].name, "z", &hw_twist_state_.angular.z);

  state_interfaces.emplace_back(
      hardware_interface::POSE_PREFIX_P,  "x", &hw_pose_state_.position.x);
  state_interfaces.emplace_back(
      hardware_interface::POSE_PREFIX_P,  "y", &hw_pose_state_.position.y);
  state_interfaces.emplace_back(
      hardware_interface::POSE_PREFIX_P,  "z", &hw_pose_state_.position.z);
  state_interfaces.emplace_back(
      hardware_interface::POSE_PREFIX_O,  "x", &hw_pose_state_.orientation.x);
  state_interfaces.emplace_back(
      hardware_interface::POSE_PREFIX_O,  "y", &hw_pose_state_.orientation.y);
  state_interfaces.emplace_back(
      hardware_interface::POSE_PREFIX_O,  "z", &hw_pose_state_.orientation.z);
  state_interfaces.emplace_back(
      hardware_interface::POSE_PREFIX_O,  "w", &hw_pose_state_.orientation.w);
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
KukaEACHardwareInterface::export_command_interfaces()
{
  RCLCPP_DEBUG(rclcpp::get_logger("KukaEACHardwareInterface"), "Export command interfaces");

  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_position_commands_[i]);

    command_interfaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_torque_commands_[i]);

    command_interfaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_STIFFNESS, &hw_stiffness_commands_[i]);

    command_interfaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_DAMPING, &hw_damping_commands_[i]);
  }

  command_interfaces.emplace_back(
    hardware_interface::CONFIG_PREFIX, hardware_interface::CONTROL_MODE, &hw_control_mode_command_);

  command_interfaces.emplace_back(
      info_.joints[0].name, hardware_interface::HW_IF_X, &hw_twist_commands_.linear.x);
  command_interfaces.emplace_back(
      info_.joints[0].name, hardware_interface::HW_IF_Y, &hw_twist_commands_.linear.y);
  command_interfaces.emplace_back(
      info_.joints[0].name, hardware_interface::HW_IF_Z, &hw_twist_commands_.linear.z);
  command_interfaces.emplace_back(
      info_.joints[1].name, hardware_interface::HW_IF_X, &hw_twist_commands_.angular.x);
  command_interfaces.emplace_back(
      info_.joints[1].name, hardware_interface::HW_IF_Y, &hw_twist_commands_.angular.y);  
  command_interfaces.emplace_back(
      info_.joints[1].name, hardware_interface::HW_IF_Z, &hw_twist_commands_.angular.z);
  return command_interfaces;
}

CallbackReturn KukaEACHardwareInterface::on_configure(const rclcpp_lifecycle::State &)
{
  if (!SetupRobot())
  {
    return CallbackReturn::FAILURE;
  }

  if (!SetupQoS())
  {
    return CallbackReturn::FAILURE;
  }

  RCLCPP_INFO(
    rclcpp::get_logger("KukaEACHardwareInterface"),
    "Set QoS profile with %s consequent and %s packet losses allowed in %s milliseconds",
    info_.hardware_parameters.at("consequent_lost_packets").c_str(),
    info_.hardware_parameters.at("lost_packets_in_timeframe").c_str(),
    info_.hardware_parameters.at("timeframe_ms").c_str());

  return CallbackReturn::SUCCESS;
}

CallbackReturn KukaEACHardwareInterface::on_activate(const rclcpp_lifecycle::State &)
{
  kuka::external::control::Status create_event_observer =
    robot_ptr_->RegisterEventHandler(std::make_unique<KukaEACEventObserver>(this));
  if (create_event_observer.return_code == kuka::external::control::ReturnCode::ERROR)
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("KukaEACHardwareInterface"),
      "Creating event observer failed, error message: %s", create_event_observer.message);
  }

  kuka::external::control::Status start_control = robot_ptr_->StartControlling(
    static_cast<kuka::external::control::ControlMode>(hw_control_mode_command_));
  if (start_control.return_code == kuka::external::control::ReturnCode::ERROR)
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("KukaEACHardwareInterface"),
      "Starting external control failed, error message: %s", start_control.message);
    return CallbackReturn::FAILURE;
  }

  prev_control_mode_ = static_cast<kuka_drivers_core::ControlMode>(hw_control_mode_command_);

  RCLCPP_INFO(
    rclcpp::get_logger("KukaEACHardwareInterface"),
    "External control session started successfully");

  stop_requested_ = false;
  cycle_count_ = 0;
  return CallbackReturn::SUCCESS;
}

CallbackReturn KukaEACHardwareInterface::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger("KukaEACHardwareInterface"), "Deactivating hardware interface");

  stop_requested_ = true;

  return CallbackReturn::SUCCESS;
}

return_type KukaEACHardwareInterface::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  // Bigger timeout blocks controller configuration
  kuka::external::control::Status receive_state =
    robot_ptr_->ReceiveMotionState(std::chrono::milliseconds(10));

  if ((msg_received_ = receive_state.return_code == kuka::external::control::ReturnCode::OK))
  {
    auto & req_message = robot_ptr_->GetLastMotionState();
    if (req_message.GetMeasuredPositions()!=nullptr){
      std::copy(
        req_message.GetMeasuredPositions()->begin(), req_message.GetMeasuredPositions()->end(),
        hw_position_states_.begin());
      }
    if (req_message.GetMeasuredTorques()!=nullptr){
      std::copy(
        req_message.GetMeasuredTorques()->begin(), req_message.GetMeasuredTorques()->end(),
        hw_torque_states_.begin());
      }
    if (req_message.GetMeasuredTorques()!=nullptr){
        hw_twist_state_.linear.x=req_message.GetMeasuredTwist()->begin().operator[](0);
        hw_twist_state_.linear.y=req_message.GetMeasuredTwist()->begin().operator[](1);
        hw_twist_state_.linear.y=req_message.GetMeasuredTwist()->begin().operator[](2);
        hw_twist_state_.angular.x=req_message.GetMeasuredTwist()->begin().operator[](3);
        hw_twist_state_.angular.y=req_message.GetMeasuredTwist()->begin().operator[](4);
        hw_twist_state_.angular.z=req_message.GetMeasuredTwist()->begin().operator[](5);
      } 
    if (req_message.GetMeasuredCartesianPositions()!=nullptr){
      hw_pose_state_.position.x=req_message.GetMeasuredCartesianPositions()->begin().operator[](0);
      hw_pose_state_.position.y=req_message.GetMeasuredCartesianPositions()->begin().operator[](1);
      hw_pose_state_.position.z=req_message.GetMeasuredCartesianPositions()->begin().operator[](2);
      hw_pose_state_.orientation.x=req_message.GetMeasuredCartesianPositions()->begin().operator[](3);
      hw_pose_state_.orientation.y=req_message.GetMeasuredCartesianPositions()->begin().operator[](4);
      hw_pose_state_.orientation.z=req_message.GetMeasuredCartesianPositions()->begin().operator[](5);
      hw_pose_state_.orientation.w=req_message.GetMeasuredCartesianPositions()->begin().operator[](6);
    }
    if (cycle_count_ == 0)
      {
        std::copy(
          hw_position_states_.begin(), hw_position_states_.end(), hw_position_commands_.begin());
      }

      cycle_count_++;
    }

      // Modify state interface only in read
      std::lock_guard<std::mutex> lk(event_mutex_);
      server_state_ = static_cast<double>(last_event_);

      tf2::Quaternion q(
        hw_pose_state_.orientation.x,
        hw_pose_state_.orientation.y,
        hw_pose_state_.orientation.z,
        hw_pose_state_.orientation.w);
      tf2::Matrix3x3 m(q);
      double roll, pitch, yaw;
      m.getRPY(roll,pitch,yaw);
      
      RCLCPP_DEBUG( rclcpp::get_logger(
            "KukaRoXHardwareInterface"), "STATE position: %f, %f, %f",
                hw_pose_state_.position.x,
                hw_pose_state_.position.y,
                yaw);
      RCLCPP_DEBUG( rclcpp::get_logger(
            "KukaRoXHardwareInterface"), "STATE velocity: %f, %f, %f",
                hw_twist_state_.linear.x,
                hw_twist_state_.linear.y,
                hw_twist_state_.angular.z);
  
  return return_type::OK;
}

return_type KukaEACHardwareInterface::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  // If control is not started or a request is missed, do not send back anything
  if (!msg_received_)
  {
    return return_type::OK;
  }

  robot_ptr_->GetControlSignal().AddJointPositionValues(hw_position_commands_);
  robot_ptr_->GetControlSignal().AddTorqueValues(hw_torque_commands_);
  robot_ptr_->GetControlSignal().AddStiffnessAndDampingValues(
    hw_stiffness_commands_, hw_damping_commands_);
    
    hw_twist_commands_vector_[0]=hw_twist_commands_.linear.x;
    hw_twist_commands_vector_[1]=hw_twist_commands_.linear.y;
    hw_twist_commands_vector_[2]=hw_twist_commands_.linear.z;
    hw_twist_commands_vector_[3]=hw_twist_commands_.angular.x;
    hw_twist_commands_vector_[4]=hw_twist_commands_.angular.y;
    hw_twist_commands_vector_[5]=hw_twist_commands_.angular.z;
  robot_ptr_->GetControlSignal().AddTwistValues(hw_twist_commands_vector_);


  RCLCPP_DEBUG(rclcpp::get_logger(
      "KukaRoXHardwareInterface"), "COMMAND velocity: x:%f, theta:%f ",
          hw_twist_commands_.linear.x, hw_twist_commands_.angular.z);

  kuka::external::control::Status send_reply;
  if (stop_requested_)
  {
    RCLCPP_INFO(rclcpp::get_logger("KukaEACHardwareInterface"), "Sending stop signal");
    send_reply = robot_ptr_->StopControlling();
  }
  else if (
    static_cast<kuka_drivers_core::ControlMode>(hw_control_mode_command_) != prev_control_mode_)
  {
    RCLCPP_INFO(rclcpp::get_logger("KukaEACHardwareInterface"), "Requesting control mode switch");
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
      rclcpp::get_logger("KukaEACHardwareInterface"), "Send reply failed, error message: %s",
      send_reply.message);
    throw std::runtime_error("Error sending reply");
  }
  return return_type::OK;
}

bool KukaEACHardwareInterface::SetupRobot()
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
      rclcpp::get_logger("KukaEACHardwareInterface"), "Setup failed, error message: %s",
      setup.message);
    return false;
  }

  return true;
}

bool KukaEACHardwareInterface::SetupQoS()
{
  kuka::external::control::iiqka::QoS_Configuration qos_config;
  qos_config.packet_loss_in_timeframe_limit =
    std::stoi(info_.hardware_parameters.at("lost_packets_in_timeframe"));
  qos_config.consecutive_packet_loss_limit =
    std::stoi(info_.hardware_parameters.at("consequent_lost_packets"));
  qos_config.timeframe_ms = std::stoi(info_.hardware_parameters.at("timeframe_ms"));

  kuka::external::control::Status set_qos_status = robot_ptr_->SetQoSProfile(qos_config);

  if (set_qos_status.return_code != kuka::external::control::ReturnCode::OK)
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("KukaEACHardwareInterface"), "QoS configuration failed, error message: %s",
      set_qos_status.message);
    return false;
  }

  return true;
}

void KukaEACHardwareInterface::set_server_event(kuka_drivers_core::HardwareEvent event)
{
  std::lock_guard<std::mutex> lk(event_mutex_);
  last_event_ = event;
}
}  // namespace kuka_eac

PLUGINLIB_EXPORT_CLASS(kuka_eac::KukaEACHardwareInterface, hardware_interface::SystemInterface)
