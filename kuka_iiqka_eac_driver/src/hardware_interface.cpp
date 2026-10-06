// Copyright 2022 KUKA Hungaria Kft.
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
#include <limits>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "kuka_drivers_core/control_mode.hpp"
#include "kuka_drivers_core/hardware_interface_types.hpp"
#include "kuka_drivers_core/hardware_interface_utils.hpp"
#include "kuka_drivers_core/joint_interface_validator.hpp"

#include "kuka_iiqka_eac_driver/event_observer.hpp"
#include "kuka_iiqka_eac_driver/hardware_interface.hpp"

namespace kuka_eac
{
CallbackReturn KukaEACHardwareInterface::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (hardware_interface::SystemInterface::on_init(params) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  // Initialize control mode with 'undefined', which should be changed by the appropriate controller
  // during configuration
  hw_position_states_.resize(info_.joints.size(), 0.0);
  hw_commanded_position_states_.resize(info_.joints.size(), 0.0);
  hw_torque_states_.resize(info_.joints.size(), 0.0);
  hw_position_commands_.resize(info_.joints.size(), 0.0);
  hw_torque_commands_.resize(info_.joints.size(), 0.0);
  hw_stiffness_commands_.resize(info_.joints.size(), 30);
  hw_damping_commands_.resize(info_.joints.size(), 0.7);

  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    if (!CheckJointInterfaces(joint))
    {
      return CallbackReturn::ERROR;
    }
  }

  RCLCPP_INFO(
    rclcpp::get_logger("KukaEACHardwareInterface"),
    "Init successful with controller ip: %s and client ip: %s",
    info_.hardware_parameters.at("controller_ip").c_str(),
    info_.hardware_parameters.at("client_ip").c_str());

  auto info = get_hardware_info();
  is_async_hardware_ = info.is_async;
  interface_prefix_ = info.name + "/";
  auto it = info.hardware_parameters.find("interface_prefix");
  if (it != info.hardware_parameters.end())
  {
    interface_prefix_ = it->second;
  }

  joint_interface_names_.resize(info_.joints.size());
  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    const std::string & name = info_.joints[i].name;
    auto & names = joint_interface_names_[i];
    names.position_state = name + "/" + hardware_interface::HW_IF_POSITION;
    names.effort_state = name + "/" + hardware_interface::HW_IF_EFFORT;
    names.commanded_position_state = name + "/" + hardware_interface::HW_IF_COMMANDED_POSITION;
    names.position_command = name + "/" + hardware_interface::HW_IF_POSITION;
    names.effort_command = name + "/" + hardware_interface::HW_IF_EFFORT;
    names.stiffness_command = name + "/" + hardware_interface::HW_IF_STIFFNESS;
    names.damping_command = name + "/" + hardware_interface::HW_IF_DAMPING;
  }
  server_state_name_ =
    interface_prefix_ + hardware_interface::STATE_PREFIX + "/" + hardware_interface::SERVER_STATE;
  control_mode_name_ =
    interface_prefix_ + hardware_interface::CONFIG_PREFIX + "/" + hardware_interface::CONTROL_MODE;
  interpolation_count_name_ = interface_prefix_ + hardware_interface::CONFIG_PREFIX + "/" +
                              hardware_interface::INTERPOLATION_COUNT;

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::InterfaceDescription>
KukaEACHardwareInterface::export_unlisted_state_interface_descriptions()
{
  hardware_interface::InterfaceInfo server_state_info{};
  server_state_info.name = hardware_interface::SERVER_STATE;
  server_state_info.initial_value = "0";
  return {hardware_interface::InterfaceDescription(
    interface_prefix_ + hardware_interface::STATE_PREFIX, server_state_info)};
}

std::vector<hardware_interface::InterfaceDescription>
KukaEACHardwareInterface::export_unlisted_command_interface_descriptions()
{
  hardware_interface::InterfaceInfo control_mode_info{};
  control_mode_info.name = hardware_interface::CONTROL_MODE;
  control_mode_info.initial_value = "0";

  hardware_interface::InterfaceInfo interpolation_count_info{};
  interpolation_count_info.name = hardware_interface::INTERPOLATION_COUNT;
  interpolation_count_info.initial_value = "0";

  return {
    hardware_interface::InterfaceDescription(
      interface_prefix_ + hardware_interface::CONFIG_PREFIX, control_mode_info),
    hardware_interface::InterfaceDescription(
      interface_prefix_ + hardware_interface::CONFIG_PREFIX, interpolation_count_info)};
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
  hw_control_mode_command_ = get_command<double>(control_mode_name_);

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

  cycle_count_ = 0;
  interpolation_count_initialized_ = false;
  return CallbackReturn::SUCCESS;
}

CallbackReturn KukaEACHardwareInterface::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(
    rclcpp::get_logger("KukaEACHardwareInterface"),
    "Deactivating hardware interface by sending stop signal");

  // StopControlling sometimes calls a blocking read, which could conflict with the read() method,
  // but resource manager handles locking (resources_lock_), so is not necessary here
  robot_ptr_->StopControlling();
  interpolation_count_initialized_ = false;

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

    std::copy(
      req_message.GetMeasuredPositions().begin(), req_message.GetMeasuredPositions().end(),
      hw_position_states_.begin());
    std::copy(
      req_message.GetMeasuredTorques().begin(), req_message.GetMeasuredTorques().end(),
      hw_torque_states_.begin());

    if (cycle_count_ == 0)
    {
      std::copy(
        hw_position_states_.begin(), hw_position_states_.end(), hw_position_commands_.begin());
      // Push the bootstrap position command now, before any controller has had a chance to
      // claim/write the interface this cycle, so write() does not pull an unset value.
      for (size_t i = 0; i < info_.joints.size(); i++)
      {
        set_command(joint_interface_names_[i].position_command, hw_position_commands_[i]);
      }
    }

    std::copy(
      hw_position_commands_.begin(), hw_position_commands_.end(),
      hw_commanded_position_states_.begin());

    for (size_t i = 0; i < info_.joints.size(); i++)
    {
      set_state(joint_interface_names_[i].position_state, hw_position_states_[i]);
      set_state(joint_interface_names_[i].effort_state, hw_torque_states_[i]);
      set_state(
        joint_interface_names_[i].commanded_position_state, hw_commanded_position_states_[i]);
    }

    cycle_count_++;
  }

  // Modify state interface only in read
  std::lock_guard<std::mutex> lk(event_mutex_);
  server_state_ = static_cast<double>(last_event_);
  set_state(server_state_name_, server_state_);
  return return_type::OK;
}

return_type KukaEACHardwareInterface::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  // If control is not started or a request is missed, do not send back anything
  if (!msg_received_)
  {
    return return_type::OK;
  }

  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    hw_position_commands_[i] = get_command<double>(joint_interface_names_[i].position_command);
    hw_torque_commands_[i] = get_command<double>(joint_interface_names_[i].effort_command);
    hw_stiffness_commands_[i] = get_command<double>(joint_interface_names_[i].stiffness_command);
    hw_damping_commands_[i] = get_command<double>(joint_interface_names_[i].damping_command);
  }
  hw_control_mode_command_ = get_command<double>(control_mode_name_);
  interpolation_count_command_ = get_command<double>(interpolation_count_name_);

  uint32_t current_count = static_cast<uint32_t>(interpolation_count_command_);
  // Skip validation while count is 0: EventBroadcaster only increments after all HW interfaces
  // report CONTROL_STARTED
  if (current_count > 0 && interpolation_count_initialized_)
  {
    const uint32_t expected_count =
      (last_interpolation_count_command_ == std::numeric_limits<uint32_t>::max())
        ? 0
        : last_interpolation_count_command_ + 1;

    if (current_count != expected_count)
    {
      // WaitForInterpolationCount polls this callback repeatedly while it waits, so it must
      // re-read the live command value each time, not the snapshot taken above.
      current_count = kuka_drivers_core::hardware_interface_utils::WaitForInterpolationCount(
        expected_count, current_count, is_async_hardware_,
        [this]()
        { return static_cast<uint32_t>(get_command<double>(interpolation_count_name_)); });

      if (current_count != expected_count)
      {
        RCLCPP_WARN(
          rclcpp::get_logger("KukaEACHardwareInterface"),
          "interpolation_count mismatch before write: expected %u, got %u, hardware is %s",
          expected_count, current_count, is_async_hardware_ ? "async" : "sync");
      }
    }
  }
  if (current_count > 0)
  {
    interpolation_count_initialized_ = true;
    last_interpolation_count_command_ = current_count;
  }

  robot_ptr_->GetControlSignal().AddJointPositionValues(
    hw_position_commands_.begin(), hw_position_commands_.end());
  robot_ptr_->GetControlSignal().AddTorqueValues(
    hw_torque_commands_.begin(), hw_torque_commands_.end());
  robot_ptr_->GetControlSignal().AddStiffnessAndDampingValues(
    hw_stiffness_commands_.begin(), hw_stiffness_commands_.end(), hw_damping_commands_.begin(),
    hw_damping_commands_.end());

  kuka::external::control::Status send_reply;
  if (static_cast<kuka_drivers_core::ControlMode>(hw_control_mode_command_) != prev_control_mode_)
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

bool KukaEACHardwareInterface::CheckJointInterfaces(
  const hardware_interface::ComponentInfo & joint) const
{
  return CheckJointCommandInterfaces(joint) && CheckJointStateInterfaces(joint);
}

bool KukaEACHardwareInterface::CheckJointCommandInterfaces(
  const hardware_interface::ComponentInfo & joint) const
{
  const std::vector<std::string> expected_interfaces = {
    hardware_interface::HW_IF_POSITION, hardware_interface::HW_IF_STIFFNESS,
    hardware_interface::HW_IF_DAMPING, hardware_interface::HW_IF_EFFORT};
  return kuka_drivers_core::urdf_validator::ValidateJointCommandInterfaces(
    joint, expected_interfaces, rclcpp::get_logger("KukaEACHardwareInterface"));
}

bool KukaEACHardwareInterface::CheckJointStateInterfaces(
  const hardware_interface::ComponentInfo & joint) const
{
  const std::vector<std::string> expected_interfaces = {
    hardware_interface::HW_IF_POSITION, hardware_interface::HW_IF_EFFORT,
    hardware_interface::HW_IF_COMMANDED_POSITION};
  return kuka_drivers_core::urdf_validator::ValidateJointStateInterfaces(
    joint, expected_interfaces, rclcpp::get_logger("KukaEACHardwareInterface"));
}
}  // namespace kuka_eac

PLUGINLIB_EXPORT_CLASS(kuka_eac::KukaEACHardwareInterface, hardware_interface::SystemInterface)
