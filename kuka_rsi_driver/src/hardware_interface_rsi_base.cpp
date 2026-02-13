// Copyright 2023 KUKA Hungaria Kft.
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

#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

#include "kuka_drivers_core/hardware_event.hpp"
#include "kuka_drivers_core/hardware_interface_types.hpp"
#include "kuka_rsi_driver/hardware_interface_rsi_base.hpp"

namespace kuka_rsi_driver
{
CallbackReturn KukaRSIHardwareInterfaceBase::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  hw_states_.resize(info_.joints.size(), 0.0);
  hw_commands_.resize(info_.joints.size(), 0.0);

  for (const auto & joint : info_.joints)
  {
    bool interfaces_ok = CheckJointInterfaces(joint);
    if (!interfaces_ok)
    {
      return CallbackReturn::ERROR;
    }
  }

  // Check gpio components size
  if (info_.gpios.size() != 1)
  {
    RCLCPP_FATAL(logger_, "expecting exactly 1 gpio component");
    return CallbackReturn::ERROR;
  }
  const auto & gpio = info_.gpios[0];
  // Check gpio component name
  if (gpio.name != hardware_interface::IO_PREFIX)
  {
    RCLCPP_FATAL(logger_, "expecting gpio component called \"gpio\" first");
    return CallbackReturn::ERROR;
  }

  // Save the mapping of GPIO states to commands
  for (const auto & command_interface : gpio.command_interfaces)
  {
    // Find the corresponding state interface for each command interface and connect them based on
    // their names
    auto it = std::find_if(
      gpio.state_interfaces.begin(), gpio.state_interfaces.end(),
      [&command_interface](const hardware_interface::InterfaceInfo & state_interface)
      { return state_interface.name == command_interface.name; });
    if (it != gpio.state_interfaces.end())
    {
      gpio_states_to_commands_map_.push_back(std::distance(gpio.state_interfaces.begin(), it));
    }
    else
    {
      gpio_states_to_commands_map_.push_back(-1);  // Not found, use -1 as a placeholder
    }
  }

  hw_gpio_states_.resize(gpio.state_interfaces.size(), 0.0);
  hw_gpio_commands_.resize(gpio.command_interfaces.size(), 0.0);

  is_active_ = false;
  msg_received_ = false;

  // For plain RSI setup, there is no event broadcaster from the server, server events are published
  // based on HWIF logic to enable reactivation after an error
  // Locking is taken care of in resource manager (read, write, on_activate, on_deactivate)
  server_state_ = static_cast<double>(kuka_drivers_core::HardwareEvent::HARDWARE_EVENT_UNSPECIFIED);

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
KukaRSIHardwareInterfaceBase::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_[i]);
  }

  for (size_t i = 0; i < info_.gpios[0].state_interfaces.size(); i++)
  {
    state_interfaces.emplace_back(
      hardware_interface::IO_PREFIX, info_.gpios[0].state_interfaces[i].name, &hw_gpio_states_[i]);
  }

  state_interfaces.emplace_back(
    hardware_interface::STATE_PREFIX, hardware_interface::SERVER_STATE, &server_state_);

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
KukaRSIHardwareInterfaceBase::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_[i]);
  }

  for (size_t i = 0; i < info_.gpios[0].command_interfaces.size(); i++)
  {
    command_interfaces.emplace_back(
      hardware_interface::IO_PREFIX, info_.gpios[0].command_interfaces[i].name,
      &hw_gpio_commands_[i]);
  }

  return command_interfaces;
}

CallbackReturn KukaRSIHardwareInterfaceBase::on_cleanup(const rclcpp_lifecycle::State &)
{
  robot_ptr_.reset();
  return CallbackReturn::SUCCESS;
}

return_type KukaRSIHardwareInterfaceBase::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  // The first packet is received at activation, Read() should not be called before
  // Add short sleep to avoid RT thread eating CPU
  if (!is_active_)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
    return return_type::OK;
  }

  Read(READ_TIMEOUT_MS);
  return return_type::OK;
}

return_type KukaRSIHardwareInterfaceBase::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  // If control is not started or a request is missed, do not send back anything
  if (!msg_received_)
  {
    return return_type::OK;
  }

  Write();

  return return_type::OK;
}

bool KukaRSIHardwareInterfaceBase::SetupRobot(
  kuka::external::control::kss::Configuration & config,
  std::unique_ptr<kuka::external::control::EventHandler> event_handler,
  std::unique_ptr<kuka::external::control::kss::IEventHandlerExtension> extension)
{
  RCLCPP_INFO(logger_, "Setting up robot...");

  ConfigureJoints(config);

  RCLCPP_INFO(
    logger_, info_.gpios[0].command_interfaces.empty() ? "No GPIO command interfaces configured"
                                                       : "Configured GPIO commands:");
  for (const auto & gpio_command : info_.gpios[0].command_interfaces)
  {
    RCLCPP_INFO(
      logger_, "Name: %s, Data type: %s, Initial value: %s, Enable limits: %s, Min: %s, Max: %s",
      gpio_command.name.c_str(), gpio_command.data_type.c_str(), gpio_command.initial_value.c_str(),
      gpio_command.enable_limits ? "true" : "false", gpio_command.min.c_str(),
      gpio_command.max.c_str());

    // TODO (Komaromi): Add size and parameters
    config.gpio_command_configs.emplace_back(ParseGPIOConfig(gpio_command));
  }

  RCLCPP_INFO(
    logger_, info_.gpios[0].state_interfaces.empty() ? "No GPIO state interfaces configured"
                                                     : "Configured GPIO states:");
  for (const auto & gpio_state : info_.gpios[0].state_interfaces)
  {
    RCLCPP_INFO(
      logger_, "Name: %s, Data type: %s, Initial value: %s, Enable limits: %s, Min: %s, Max: %s",
      gpio_state.name.c_str(), gpio_state.data_type.c_str(), gpio_state.initial_value.c_str(),
      gpio_state.enable_limits ? "true" : "false", gpio_state.min.c_str(), gpio_state.max.c_str());

    // TODO (Komaromi): Add size, and parameters
    config.gpio_state_configs.emplace_back(ParseGPIOConfig(gpio_state));
  }

  CreateRobotInstance(config);

  if (event_handler != nullptr)
  {
    auto status = robot_ptr_->RegisterEventHandler(std::move(event_handler));
    if (status.return_code == kuka::external::control::ReturnCode::ERROR)
    {
      RCLCPP_ERROR(logger_, "Creating event observer failed: %s", status.message);
    }
  }

  if (extension != nullptr)
  {
    auto status = robot_ptr_->RegisterEventHandlerExtension(std::move(extension));
    if (status.return_code == kuka::external::control::ReturnCode::ERROR)
    {
      RCLCPP_INFO(logger_, "Creating event handler extension failed: %s", status.message);
    }
  }
  const auto setup = robot_ptr_->Setup();
  if (setup.return_code != kuka::external::control::ReturnCode::OK)
  {
    RCLCPP_ERROR(logger_, "Setup failed: %s", setup.message);
    return false;
  }

  RCLCPP_INFO(logger_, "Robot setup successful!");

  return true;
}

void KukaRSIHardwareInterfaceBase::Read(const int64_t request_timeout)
{
  auto motion_state_status =
    robot_ptr_->ReceiveMotionState(std::chrono::milliseconds(request_timeout));
  msg_received_ = motion_state_status.return_code == kuka::external::control::ReturnCode::OK;
  if (msg_received_)
  {
    const auto & req_message = robot_ptr_->GetLastMotionState();
    const auto & positions = req_message.GetMeasuredPositions();
    const auto & gpio_values = req_message.GetGPIOValues();

    std::copy(positions.cbegin(), positions.cend(), hw_states_.begin());
    // Save IO states
    for (size_t i = 0; i < hw_gpio_states_.size(); i++)
    {
      auto value = gpio_values.at(i)->GetValue();
      if (value.has_value())
      {
        hw_gpio_states_[i] = value.value();
      }
      else
      {
        RCLCPP_ERROR(
          logger_, "GPIO value not set. No value type found for GPIO %s (Should be dead code)",
          gpio_values.at(i)->GetGPIOConfig()->GetName().c_str());
      }
    }
  }
  else
  {
    RCLCPP_ERROR(logger_, "Failed to receive motion state %s", motion_state_status.message);
    set_server_event(kuka_drivers_core::HardwareEvent::ERROR);
  }

  std::lock_guard<std::mutex> lk(event_mutex_);
  server_state_ = static_cast<double>(last_event_);
}

void KukaRSIHardwareInterfaceBase::set_server_event(kuka_drivers_core::HardwareEvent event)
{
  std::lock_guard<std::mutex> lk(event_mutex_);
  last_event_ = event;
}

bool KukaRSIHardwareInterfaceBase::CheckJointInterfaces(
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

void KukaRSIHardwareInterfaceBase::CopyGPIOStatesToCommands()
{
  for (size_t i = 0; i < gpio_states_to_commands_map_.size(); i++)
  {
    if (gpio_states_to_commands_map_[i] != -1)
    {
      hw_gpio_commands_[i] = hw_gpio_states_[gpio_states_to_commands_map_[i]];
    }
  }
}

kuka::external::control::kss::GPIOConfiguration KukaRSIHardwareInterfaceBase::ParseGPIOConfig(
  const hardware_interface::InterfaceInfo & info)
{
  kuka::external::control::kss::GPIOConfiguration gpio_config;
  gpio_config.name = info.name;
  gpio_config.enable_limits = info.enable_limits;
  // TODO (komaromi): This might not work from Kilted kaiju onward the get_optional function in the
  // handle since it is only accepting double and bool
  if (info.data_type == "BOOL" || info.data_type == "bool")
  {
    gpio_config.value_type = kuka::external::control::GPIOValueType::BOOL;
  }
  else if (info.data_type == "DOUBLE" || info.data_type == "double")
  {
    gpio_config.value_type = kuka::external::control::GPIOValueType::DOUBLE;
  }
  else if (info.data_type == "LONG")
  {
    gpio_config.value_type = kuka::external::control::GPIOValueType::LONG;
  }
  else
  {
    gpio_config.value_type = kuka::external::control::GPIOValueType::UNSPECIFIED;
  }

  if (!info.initial_value.empty())
  {
    try
    {
      gpio_config.initial_value = std::stod(info.initial_value);
    }
    catch (const std::exception & ex)
    {
      RCLCPP_WARN(
        logger_, "Initial_value is not valid number, it is set to 0. Exception: %s", ex.what());
      gpio_config.initial_value = 0.0;  // If initial_value is not a valid number, set to 0.0
    }
  }
  else
  {
    // TODO (Komaromi): Should this be set to 0?
    gpio_config.initial_value = 0.0;  // If initial_value is empty, set to 0.0
  }
  if (!info.min.empty())
  {
    try
    {
      gpio_config.min_value = std::stod(info.min);
    }
    catch (const std::exception & ex)
    {
      RCLCPP_WARN(
        logger_, "Min_value is not valid number, limits not used. Exception: %s", ex.what());
      gpio_config.enable_limits = false;  // If min_value is not a valid number, disable limits
    }
  }
  else
  {
    gpio_config.enable_limits = false;  // If min_value is empty, disable limits
  }
  if (!info.max.empty())
  {
    try
    {
      gpio_config.max_value = std::stod(info.max);
    }
    catch (const std::exception & ex)
    {
      RCLCPP_WARN(
        logger_, "Max_value is not valid number, limits not used. Exception: %s", ex.what());
      gpio_config.enable_limits = false;  // If max_value is not a valid number, disable limits
    }
  }
  else
  {
    gpio_config.enable_limits = false;  // If max_value is empty, disable limits
  }
  if (gpio_config.enable_limits && (gpio_config.min_value > gpio_config.max_value))
  {
    RCLCPP_WARN(
      logger_, "Min value is greater than max value, limits not used. Min: %f, Max: %f",
      gpio_config.min_value, gpio_config.max_value);
    gpio_config.enable_limits = false;  // If min_value is greater than or equal to max_value,
                                        // disable limits
  }
  return gpio_config;
}

CallbackReturn KukaRSIHardwareInterfaceBase::extended_activation(const rclcpp_lifecycle::State &)
{
  if (status_manager_.IsEmergencyStopActive())
  {
    RCLCPP_ERROR(logger_, "Emergency stop is active. Cannot activate hardware interface.");
    return CallbackReturn::FAILURE;
  }

  if (!status_manager_.IsKrcInExtMode())
  {
    RCLCPP_ERROR(logger_, "KRC not in EXT. Switch to EXT to activate.");
    return CallbackReturn::FAILURE;
  }

  if (!status_manager_.DrivesPowered())
  {
    RCLCPP_INFO(logger_, "Turning on drives");
    robot_ptr_->TurnOnDrives();

    // Wait for drives to be powered up
    auto start_time = std::chrono::steady_clock::now();
    while (!status_manager_.DrivesPowered())
    {
      if (
        std::chrono::steady_clock::now() - start_time >
        KukaRSIHardwareInterfaceBase::DRIVES_POWERED_TIMEOUT)
      {
        RCLCPP_ERROR(logger_, "Timeout waiting for drives to power on. Check robot state.");
        return CallbackReturn::FAILURE;
      }
      status_manager_.UpdateStateInterfaces();
      std::this_thread::sleep_for(KukaRSIHardwareInterfaceBase::DRIVES_POWERED_CHECK_INTERVAL);
    }
    RCLCPP_INFO(logger_, "Drives successfully powered on");
  }

  // Set control mode and cycle time before sending Start request
  ChangeCycleTime();

  const auto control_mode =
    static_cast<kuka::external::control::ControlMode>(hw_control_mode_command_);

  kuka::external::control::Status control_status = robot_ptr_->StartControlling(control_mode);
  if (control_status.return_code == kuka::external::control::ReturnCode::ERROR)
  {
    RCLCPP_ERROR(logger_, "Starting external control failed: %s", control_status.message);
    return CallbackReturn::FAILURE;
  }

  prev_control_mode_ = static_cast<kuka_drivers_core::ControlMode>(hw_control_mode_command_);

  // We must first receive the initial position of the robot
  // We set a longer timeout, since the first message might not arrive all that fast
  Read(5 * READ_TIMEOUT_MS);
  std::copy(hw_states_.cbegin(), hw_states_.cend(), hw_commands_.begin());
  CopyGPIOStatesToCommands();
  Write();

  msg_received_ = false;
  is_active_ = true;

  RCLCPP_INFO(logger_, "Received position data from robot controller!");

  return CallbackReturn::SUCCESS;
}

CallbackReturn KukaRSIHardwareInterfaceBase::extended_deactivation(const rclcpp_lifecycle::State &)
{
  if (msg_received_)
  {
    RCLCPP_INFO(logger_, "Deactivating hardware interface by sending stop signal");

    // StopControlling sometimes calls a blocking read, which could conflict with the read() method,
    // but resource manager handles locking (resources_lock_), so is not necessary here
    robot_ptr_->StopControlling();
  }
  else
  {
    RCLCPP_INFO(logger_, "Message not received, but stop requested. Cancelling RSI program.");
    robot_ptr_->CancelRsiProgram();
  }
  is_active_ = false;
  msg_received_ = false;

  if (status_manager_.DrivesPowered())
  {
    RCLCPP_INFO(logger_, "Turning off drives");
    robot_ptr_->TurnOffDrives();

    // Wait for drives to be powered off
    auto start_time = std::chrono::steady_clock::now();
    while (status_manager_.DrivesPowered())
    {
      if (
        std::chrono::steady_clock::now() - start_time >
        KukaRSIHardwareInterfaceBase::DRIVES_POWERED_TIMEOUT)
      {
        RCLCPP_ERROR(logger_, "Timeout waiting for drives to power off. Check robot state.");
        // Return success, as drives off signal is not received in Office mode for iiQKA.OS2
        status_manager_.UpdateStateInterfaces();
        return CallbackReturn::SUCCESS;
      }
      status_manager_.UpdateStateInterfaces();
      std::this_thread::sleep_for(KukaRSIHardwareInterfaceBase::DRIVES_POWERED_CHECK_INTERVAL);
    }
    RCLCPP_INFO(logger_, "Drives successfully powered off");
  }
  return CallbackReturn::SUCCESS;
}

void KukaRSIHardwareInterfaceBase::Write()
{
  // Write values to hardware interface
  auto & control_signal = robot_ptr_->GetControlSignal();
  control_signal.AddJointPositionValues(hw_commands_.cbegin(), hw_commands_.cend());
  control_signal.AddGPIOValues(hw_gpio_commands_.cbegin(), hw_gpio_commands_.cend());

  auto send_reply_status = robot_ptr_->SendControlSignal();

  if (send_reply_status.return_code != kuka::external::control::ReturnCode::OK)
  {
    RCLCPP_ERROR(logger_, "Sending reply failed: %s", send_reply_status.message);
    throw std::runtime_error("Error sending reply");
  }
}

kuka::external::control::Status KukaRSIHardwareInterfaceBase::ChangeCycleTime()
{
  const RsiCycleTime cycle_time = static_cast<RsiCycleTime>(cycle_time_command_);

  if (prev_cycle_time_ != cycle_time)
  {
    RCLCPP_INFO(logger_, "Changing RSI cycle time to %d", static_cast<int>(cycle_time));
    auto status = robot_ptr_->SetCycleTime(cycle_time);
    if (status.return_code != kuka::external::control::ReturnCode::OK)
    {
      return status;
    }
    prev_cycle_time_ = cycle_time;
  }

  return kuka::external::control::Status(kuka::external::control::ReturnCode::OK);
}

void KukaRSIHardwareInterfaceBase::initialize_command_interfaces(
  kuka_drivers_core::ControlMode control_mode, RsiCycleTime cycle_time)
{
  prev_control_mode_ = control_mode;
  prev_cycle_time_ = cycle_time;
  hw_control_mode_command_ = static_cast<double>(control_mode);
  cycle_time_command_ = static_cast<double>(cycle_time);
}

void KukaRSIHardwareInterfaceBase::ConfigureJoints(
  kuka::external::control::kss::Configuration & config)
{
  using JC = kuka::external::control::kss::JointConfiguration;

  config.dof = info_.joints.size();
  config.joint_configs.reserve(info_.joints.size());

  for (const auto & joint : info_.joints)
  {
    // Default to revolute joints
    const auto type_it = joint.parameters.find(std::string(kTypeParamValue));
    const auto type =
      (type_it == joint.parameters.end()) ? JC::Type::REVOLUTE : JC::ToType(type_it->second);

    // Default to internal joints
    const auto external_it = joint.parameters.find(std::string(kIsExternalParamValue));
    const bool is_external =
      (external_it == joint.parameters.end()) ? false : external_it->second == "true";

    config.joint_configs.emplace_back(joint.name, type, is_external);

    RCLCPP_INFO_STREAM(
      logger_, "Configured joint \"" << joint.name << "\": type=" << JC::TypeToString(type)
                                     << ", external=" << (is_external ? "true" : "false"));
  }
}

}  // namespace kuka_rsi_driver
