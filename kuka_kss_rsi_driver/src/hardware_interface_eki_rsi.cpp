// Copyright 2025 Kristóf Pásztor
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

#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"

#include "kuka_drivers_core/hardware_interface_types.hpp"
#include "kuka_kss_rsi_driver/event_observer_eki_rsi.hpp"
#include "kuka_kss_rsi_driver/hardware_interface_eki_rsi.hpp"

namespace kuka_kss_rsi_driver
{
CallbackReturn KukaRSIHardwareInterface::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  hw_position_states_.resize(info_.joints.size(), 0.0);
  hw_position_commands_.resize(info_.joints.size(), 0.0);

  for (const auto & joint : info_.joints)
  {
    bool interfaces_ok = CheckJointInterfaces(joint);
    if (!interfaces_ok)
    {
      return CallbackReturn::ERROR;
    }
  }

  RCLCPP_INFO(logger_, "Client IP: %s", info_.hardware_parameters["client_ip"].c_str());
  RCLCPP_INFO(logger_, "Controller IP: %s", info_.hardware_parameters["controller_ip"].c_str());

  is_active_ = false;
  hw_control_mode_command_ = 0.0;
  server_state_ = 0.0;

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> KukaRSIHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_position_states_[i]);
  }

  state_interfaces.emplace_back(
    hardware_interface::STATE_PREFIX, hardware_interface::SERVER_STATE, &server_state_);

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
KukaRSIHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_position_commands_[i]);
  }

  command_interfaces.emplace_back(
    hardware_interface::CONFIG_PREFIX, hardware_interface::CONTROL_MODE, &hw_control_mode_command_);

  return command_interfaces;
}

CallbackReturn KukaRSIHardwareInterface::on_configure(const rclcpp_lifecycle::State &)
{
  const bool setup_success = SetupRobot();

  return setup_success ? CallbackReturn::SUCCESS : CallbackReturn::ERROR;
}

CallbackReturn KukaRSIHardwareInterface::on_cleanup(const rclcpp_lifecycle::State &)
{
  robot_ptr_.reset();
  return CallbackReturn::SUCCESS;
}

CallbackReturn KukaRSIHardwareInterface::on_activate(const rclcpp_lifecycle::State &)
{
  const auto control_mode =
    static_cast<kuka::external::control::ControlMode>(hw_control_mode_command_);

  kuka::external::control::Status control_status = robot_ptr_->StartControlling(control_mode);
  if (control_status.return_code == kuka::external::control::ReturnCode::ERROR)
  {
    RCLCPP_ERROR(logger_, "Starting external control failed: %s", control_status.message);
    return CallbackReturn::FAILURE;
  }

  prev_control_mode_ = static_cast<kuka_drivers_core::ControlMode>(hw_control_mode_command_);

  stop_requested_ = false;

  // We must first receive the initial position of the robot
  // We set a longer timeout, since the first message might not arrive all that fast
  Read(5'000);
  std::copy(
    hw_position_states_.cbegin(), hw_position_states_.cend(), hw_position_commands_.begin());
  is_active_ = true;
  RCLCPP_INFO(logger_, "Received position data from robot controller!");

  return CallbackReturn::SUCCESS;
}

CallbackReturn KukaRSIHardwareInterface::on_deactivate(const rclcpp_lifecycle::State &)
{
  set_stop_flag();
  return CallbackReturn::SUCCESS;
}

return_type KukaRSIHardwareInterface::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  if (!is_active_)
  {
    return return_type::OK;
  }

  Read(1'000);
  return return_type::OK;
}

return_type KukaRSIHardwareInterface::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  if (!is_active_ || !msg_received_)
  {
    return return_type::OK;
  }

  Write();

  return return_type::OK;
}

void KukaRSIHardwareInterface::set_server_event(kuka_drivers_core::HardwareEvent event)
{
  std::lock_guard<std::mutex> lk(event_mutex_);
  last_event_ = event;
}

bool KukaRSIHardwareInterface::SetupRobot()
{
  RCLCPP_INFO(logger_, "Initiating connection setup to the robot controller...");

  using Configuration = kuka::external::control::kss::Configuration;
  Configuration config;
  config.installed_interface = Configuration::InstalledInterface::EKI_RSI;
  config.client_ip_address = info_.hardware_parameters["client_ip"];
  config.kli_ip_address = info_.hardware_parameters["controller_ip"];
  config.dof = info_.joints.size();

  robot_ptr_ = std::make_unique<kuka::external::control::kss::Robot>(config);

  auto event_observer = std::make_unique<KukaRSIEventObserver>(this);
  const auto handler_registration_status =
    robot_ptr_->RegisterEventHandler(std::move(event_observer));
  if (handler_registration_status.return_code == kuka::external::control::ReturnCode::ERROR)
  {
    const auto message = handler_registration_status.message;
    RCLCPP_ERROR(logger_, "Creating event observer failed: %s", message);
  }

  const auto setup = robot_ptr_->Setup();
  if (setup.return_code != kuka::external::control::ReturnCode::OK)
  {
    RCLCPP_ERROR(logger_, "Setup failed: %s", setup.message);
    return false;
  }

  RCLCPP_INFO(logger_, "Successfully established connection to the robot controller!");

  return true;
}

void KukaRSIHardwareInterface::Read(const int64_t request_timeout)
{
  std::chrono::milliseconds timeout(request_timeout);
  const auto motion_state_status = robot_ptr_->ReceiveMotionState(timeout);

  msg_received_ = motion_state_status.return_code == kuka::external::control::ReturnCode::OK;
  if (msg_received_)
  {
    const auto & req_message = robot_ptr_->GetLastMotionState();
    const auto & positions = req_message.GetMeasuredPositions();
    std::copy(positions.cbegin(), positions.cend(), hw_position_states_.begin());
  }

  std::lock_guard<std::mutex> lk(event_mutex_);
  server_state_ = static_cast<double>(last_event_);
}

void KukaRSIHardwareInterface::Write()
{
  // Write values to hardware interface
  auto & control_signal = robot_ptr_->GetControlSignal();
  control_signal.AddJointPositionValues(
    hw_position_commands_.cbegin(), hw_position_commands_.cend());

  const auto control_mode = static_cast<kuka_drivers_core::ControlMode>(hw_control_mode_command_);
  const bool control_mode_change_requested = control_mode != prev_control_mode_;
  kuka::external::control::Status send_reply_status;
  if (stop_requested_)
  {
    RCLCPP_INFO(logger_, "Sending stop signal");
    is_active_ = false;
    send_reply_status = robot_ptr_->StopControlling();
  }
  else if (control_mode_change_requested)
  {
    // TODO(pasztork): Test this branch once other control modes become available.
    RCLCPP_INFO(logger_, "Requesting control mode change");
    send_reply_status = robot_ptr_->SwitchControlMode(
      static_cast<kuka::external::control::ControlMode>(hw_control_mode_command_));
    prev_control_mode_ = control_mode;
  }
  else
  {
    send_reply_status = robot_ptr_->SendControlSignal();
  }

  if (send_reply_status.return_code != kuka::external::control::ReturnCode::OK)
  {
    RCLCPP_ERROR(logger_, "Sending reply failed: %s", send_reply_status.message);
    throw std::runtime_error("Error sending reply");
  }
}

bool KukaRSIHardwareInterface::CheckJointInterfaces(
  const hardware_interface::ComponentInfo & joint) const
{
  if (joint.command_interfaces.size() != 1)
  {
    RCLCPP_FATAL(logger_, "Expecting exactly 1 command interface");
    return false;
  }

  if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
  {
    RCLCPP_FATAL(logger_, "Expecting only POSITION command interface");
    return false;
  }

  if (joint.state_interfaces.size() != 1)
  {
    RCLCPP_FATAL(logger_, "Expecting exactly 1 state interface");
    return false;
  }

  if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
  {
    RCLCPP_FATAL(logger_, "Expecting only POSITION state interface");
    return false;
  }

  return true;
}
}  // namespace kuka_kss_rsi_driver

PLUGINLIB_EXPORT_CLASS(
  kuka_kss_rsi_driver::KukaRSIHardwareInterface, hardware_interface::SystemInterface)
