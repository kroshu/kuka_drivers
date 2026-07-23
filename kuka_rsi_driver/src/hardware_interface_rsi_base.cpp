// Copyright 2026 KUKA Hungaria Kft.
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

#include <limits>
#include <vector>
#include <limits>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

#include "kuka_drivers_core/hardware_event.hpp"
#include "kuka_drivers_core/hardware_interface_types.hpp"
#include "kuka_rsi_driver/hardware_interface_rsi_base.hpp"
#include "kuka_rsi_driver/rsi_xml_configuration_parser.hpp"

namespace kuka_rsi_driver
{
CallbackReturn KukaRSIHardwareInterfaceBase::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (hardware_interface::SystemInterface::on_init(params) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  interface_data_.position_states.resize(
    info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  interface_data_.velocity_states.resize(
    info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  interface_data_.torque_states.resize(
    info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  interface_data_.position_commands.resize(
    info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  interface_data_.velocity_commands.resize(
    info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  interface_data_.torque_commands.resize(
    info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  for (const auto & joint : info_.joints)
  {
    if (!CheckJointInterfaces(joint))
    {
      return CallbackReturn::ERROR;
    }
  }

  if (const auto xml_config_it =
        info_.hardware_parameters.find(std::string(kRsiXmlConfigFileParam));
      xml_config_it != info_.hardware_parameters.end() && !xml_config_it->second.empty())
  {
    kuka::external::control::kss::Configuration temp_config;
    if (!LoadXmlConfig(xml_config_it->second, temp_config))
    {
      RCLCPP_ERROR(
        logger_, "Aborting initialization due to invalid RSI XML config file: '%s'",
        xml_config_it->second.c_str());
      return CallbackReturn::ERROR;
    }
    motion_state_xml_config_ = temp_config.motion_state_xml_config;
    control_signal_xml_config_ = temp_config.control_signal_xml_config;
  }

  // Derive optional interface flags from the XML config.
  if (motion_state_xml_config_.has_value())
  {
    using MST = kuka::external::control::kss::MotionStateSignalType;
    const auto & joint_fields = motion_state_xml_config_.value().joint_fields;
    optional_interface_flags_.has_velocity_state_interface = std::any_of(
      joint_fields.cbegin(), joint_fields.cend(),
      [](const kuka::external::control::kss::MotionStateJointFieldConfiguration & field)
      { return field.signal_type == MST::VELOCITY; });
    optional_interface_flags_.has_torque_state_interface = std::any_of(
      joint_fields.cbegin(), joint_fields.cend(),
      [](const kuka::external::control::kss::MotionStateJointFieldConfiguration & field)
      { return field.signal_type == MST::TORQUE; });
  }

  if (control_signal_xml_config_.has_value())
  {
    const auto & ctrl_cfg = control_signal_xml_config_.value();
    optional_interface_flags_.has_velocity_command_interface =
      ctrl_cfg.include_velocity_values || ctrl_cfg.include_ext_velocity_values;
    optional_interface_flags_.has_torque_command_interface =
      ctrl_cfg.include_torque_values || ctrl_cfg.include_ext_torque_values;
  }

  // Warn if velocity/torque interfaces are exported but not configured in XML
  if (!optional_interface_flags_.has_velocity_state_interface)
  {
    RCLCPP_WARN(
      logger_,
      "Velocity state interfaces will be exported to ROS 2 Control, but "
      "motion_state.joints.velocities is not configured in RSI XML. Velocity state values will "
      "remain at their default (NaN) and will not be updated with actual measurements from the "
      "robot.");
  }

  if (!optional_interface_flags_.has_torque_state_interface)
  {
    RCLCPP_WARN(
      logger_,
      "Effort state interfaces will be exported to ROS 2 Control, but motion_state.joints.torques "
      "is not configured in RSI XML. Effort state values will remain at their default (NaN) and "
      "will not be updated with actual measurements from the robot.");
  }

  if (!optional_interface_flags_.has_velocity_command_interface)
  {
    RCLCPP_WARN(
      logger_,
      "Velocity command interfaces will be exported to ROS 2 Control, but "
      "control_signal.velocities (or control_signal.ext_velocities) is not enabled in RSI XML. "
      "Velocity commands will not be transmitted to the robot even if they are written to ROS 2 "
      "Control.");
  }

  if (!optional_interface_flags_.has_torque_command_interface)
  {
    RCLCPP_WARN(
      logger_,
      "Effort command interfaces will be exported to ROS 2 Control, but control_signal.torques "
      "(or control_signal.ext_torques) is not enabled in RSI XML. Effort commands will not be "
      "transmitted to the robot even if they are written to ROS 2 Control.");
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
      runtime_state_.gpio_states_to_commands_map.push_back(
        std::distance(gpio.state_interfaces.begin(), it));
    }
    else
    {
      runtime_state_.gpio_states_to_commands_map.push_back(
        -1);  // Not found, use -1 as a placeholder
    }
  }

  interface_data_.gpio_states.resize(gpio.state_interfaces.size(), 0.0);
  interface_data_.gpio_commands.resize(gpio.command_interfaces.size(), 0.0);

  // For plain RSI setup, there is no event broadcaster from the server, server events are published
  // based on HWIF logic to enable reactivation after an error
  // Locking is taken care of in resource manager (read, write, on_activate, on_deactivate)
  event_state_.server_state =
    static_cast<double>(kuka_drivers_core::HardwareEvent::HARDWARE_EVENT_UNSPECIFIED);

  auto info = get_hardware_info();
  is_async_hardware_ = info.is_async;
  interface_prefix_ = info.name + "/";
  auto it = info.hardware_parameters.find("interface_prefix");
  if (it != info.hardware_parameters.end())
  {
    interface_prefix_ = it->second;
  }

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
KukaRSIHardwareInterfaceBase::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION,
      &interface_data_.position_states[i]);
    state_interfaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY,
      &interface_data_.velocity_states[i]);
    state_interfaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &interface_data_.torque_states[i]);
  }

  for (size_t i = 0; i < info_.gpios[0].state_interfaces.size(); i++)
  {
    state_interfaces.emplace_back(
      hardware_interface::IO_PREFIX, info_.gpios[0].state_interfaces[i].name,
      &interface_data_.gpio_states[i]);
  }

  state_interfaces.emplace_back(
    interface_prefix_ + hardware_interface::STATE_PREFIX, hardware_interface::SERVER_STATE, &event_state_.server_state);

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
KukaRSIHardwareInterfaceBase::export_command_interfaces()
{
  RCLCPP_INFO(logger_, "Exporting command interfaces");
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION,
      &interface_data_.position_commands[i]);
    command_interfaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY,
      &interface_data_.velocity_commands[i]);
    command_interfaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &interface_data_.torque_commands[i]);
  }

  for (size_t i = 0; i < info_.gpios[0].command_interfaces.size(); i++)
  {
    command_interfaces.emplace_back(
      hardware_interface::IO_PREFIX, info_.gpios[0].command_interfaces[i].name,
      &interface_data_.gpio_commands[i]);
  }

  command_interfaces.emplace_back(
    interface_prefix_ + hardware_interface::CONFIG_PREFIX,
    hardware_interface::INTERPOLATION_COUNT, &interpolation_count_command_);

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
  if (!runtime_state_.is_active)
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
  if (!runtime_state_.msg_received)
  {
    return return_type::OK;
  }

  uint32_t current_count = static_cast<uint32_t>(interpolation_count_command_);
  if (interpolation_count_initialized_)
  {
    const uint32_t expected_count =
      (last_interpolation_count_command_ == std::numeric_limits<uint32_t>::max())
      ? 0
      : last_interpolation_count_command_ + 1;

    if (current_count != expected_count)
    {
      // Async components may lag one cycle behind controller updates; retry up to 1 ms.
      if (is_async_hardware_)
      {
        RCLCPP_DEBUG(
          logger_,
          "interpolation_count mismatch before write: expected %u, got %u",
          expected_count, current_count);
        const auto retry_deadline =
          std::chrono::steady_clock::now() + std::chrono::milliseconds(1);
        const auto retry_step =
          std::chrono::duration_cast<std::chrono::steady_clock::duration>(
          std::chrono::microseconds(200));

        while (current_count != expected_count)
        {
          const auto now = std::chrono::steady_clock::now();
          if (now >= retry_deadline)
          {
            break;
          }

          auto sleep_time = retry_step;
          const auto remaining = retry_deadline - now;
          if (remaining < sleep_time)
          {
            sleep_time = remaining;
          }

          std::this_thread::sleep_for(sleep_time);
          current_count = static_cast<uint32_t>(interpolation_count_command_);
        }
      }

      if (current_count != expected_count)
      {
        RCLCPP_WARN(
          logger_,
          "interpolation_count mismatch before write: expected %u, got %u",
          expected_count, current_count);
      }
    }
  }
  interpolation_count_initialized_ = true;
  last_interpolation_count_command_ = current_count;

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

  // Apply the XML config parsed during on_init (may be nullopt if no config file was provided).
  config.motion_state_xml_config = motion_state_xml_config_;
  config.control_signal_xml_config = control_signal_xml_config_;

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

    // TODO(Komaromi): Add size and parameters
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

    // TODO(Komaromi): Add size, and parameters
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
  runtime_state_.msg_received =
    motion_state_status.return_code == kuka::external::control::ReturnCode::OK;
  if (runtime_state_.msg_received)
  {
    // record timestamp immediately after the motion state is received
    auto now = std::chrono::steady_clock::now();

    // measure interval since previous packet if available
    if (diagnostics_state_.last_msg_received_time != std::chrono::steady_clock::time_point{})
    {
      auto interval = now - diagnostics_state_.last_msg_received_time;
      auto interval_ms =
        std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(interval);
      // determine expected cycle time enum
      double dt_ms =
        (control_state_.prev_cycle_time == RsiCycleTime::RSI_12MS) ? 12.0 : 4.0;  // default to 4ms
      double low_thresh = dt_ms - 0.5;
      double high_thresh = dt_ms + 0.5;
      if (interval_ms.count() < low_thresh || interval_ms.count() > high_thresh)
      {
        RCLCPP_WARN(
          logger_,
          "Unexpected RSI state interval: %.3f ms (expected %.3f±0.5 ms), change in interpolation "
          "count %lu",
          interval_ms.count(), dt_ms, robot_ptr_->getIpoc() - diagnostics_state_.last_ipoc);
      }
    }
    // update stored time for both interval and control-latency calculations
    diagnostics_state_.last_msg_received_time = now;

    const auto & req_message = robot_ptr_->GetLastMotionState();
    const auto & positions = req_message.GetMeasuredPositions();
    const auto & gpio_values = req_message.GetGPIOValues();

    std::copy(positions.cbegin(), positions.cend(), interface_data_.position_states.begin());
    if (optional_interface_flags_.has_velocity_state_interface)
    {
      const auto & velocities = req_message.GetMeasuredVelocities();
      std::copy(velocities.cbegin(), velocities.cend(), interface_data_.velocity_states.begin());
    }
    if (optional_interface_flags_.has_torque_state_interface)
    {
      const auto & torques = req_message.GetMeasuredTorques();
      std::copy(torques.cbegin(), torques.cend(), interface_data_.torque_states.begin());
    }
    // Save IO states
    for (size_t i = 0; i < interface_data_.gpio_states.size(); i++)
    {
      auto value = gpio_values.at(i)->GetValue();
      if (value.has_value())
      {
        interface_data_.gpio_states[i] = value.value();
      }
      else
      {
        RCLCPP_ERROR(
          logger_, "GPIO value not set. No value type found for GPIO %s (Should be dead code)",
          gpio_values.at(i)->GetGPIOConfig()->GetName().c_str());
      }
    }

    if (robot_ptr_->getDelay() != 0)
    {
      diagnostics_state_.packet_loss_count++;
      RCLCPP_WARN(
        logger_,
        "Packet loss registered, number of lost packets: %lu, continuous packet losses: %lu",
        diagnostics_state_.packet_loss_count, robot_ptr_->getDelay());
    }

    diagnostics_state_.last_ipoc = robot_ptr_->getIpoc();
  }
  else
  {
    RCLCPP_ERROR(logger_, "Failed to receive motion state %s", motion_state_status.message);
    set_server_event(kuka_drivers_core::HardwareEvent::ERROR);
  }

  std::lock_guard<std::mutex> lk(event_state_.event_mutex);
  event_state_.server_state = static_cast<double>(event_state_.last_event);
}

void KukaRSIHardwareInterfaceBase::set_server_event(kuka_drivers_core::HardwareEvent event)
{
  std::lock_guard<std::mutex> lk(event_state_.event_mutex);
  event_state_.last_event = event;
}

bool KukaRSIHardwareInterfaceBase::CheckJointInterfaces(
  const hardware_interface::ComponentInfo & joint) const
{
  return CheckJointCommandInterfaces(joint) && CheckJointStateInterfaces(joint);
}

bool KukaRSIHardwareInterfaceBase::CheckJointCommandInterfaces(
  const hardware_interface::ComponentInfo & joint) const
{
  bool has_position_command = false;
  bool has_velocity_command = false;
  bool has_effort_command = false;

  for (const auto & interface_info : joint.command_interfaces)
  {
    if (interface_info.name == hardware_interface::HW_IF_POSITION)
    {
      if (has_position_command)
      {
        RCLCPP_FATAL(
          logger_, "Duplicate POSITION command interface for joint %s", joint.name.c_str());
        return false;
      }
      has_position_command = true;
    }
    else if (interface_info.name == hardware_interface::HW_IF_VELOCITY)
    {
      if (has_velocity_command)
      {
        RCLCPP_FATAL(
          logger_, "Duplicate VELOCITY command interface for joint %s", joint.name.c_str());
        return false;
      }
      has_velocity_command = true;
    }
    else if (interface_info.name == hardware_interface::HW_IF_EFFORT)
    {
      if (has_effort_command)
      {
        RCLCPP_FATAL(
          logger_, "Duplicate EFFORT command interface for joint %s", joint.name.c_str());
        return false;
      }
      has_effort_command = true;
    }
    else
    {
      RCLCPP_FATAL(
        logger_, "Unsupported command interface '%s' for joint %s", interface_info.name.c_str(),
        joint.name.c_str());
      return false;
    }
  }

  return true;
}

bool KukaRSIHardwareInterfaceBase::CheckJointStateInterfaces(
  const hardware_interface::ComponentInfo & joint) const
{
  bool has_position_state = false;
  bool has_velocity_state = false;
  bool has_effort_state = false;

  for (const auto & interface_info : joint.state_interfaces)
  {
    if (interface_info.name == hardware_interface::HW_IF_POSITION)
    {
      if (has_position_state)
      {
        RCLCPP_FATAL(
          logger_, "Duplicate POSITION state interface for joint %s", joint.name.c_str());
        return false;
      }
      has_position_state = true;
    }
    else if (interface_info.name == hardware_interface::HW_IF_VELOCITY)
    {
      if (has_velocity_state)
      {
        RCLCPP_FATAL(
          logger_, "Duplicate VELOCITY state interface for joint %s", joint.name.c_str());
        return false;
      }
      has_velocity_state = true;
    }
    else if (interface_info.name == hardware_interface::HW_IF_EFFORT)
    {
      if (has_effort_state)
      {
        RCLCPP_FATAL(logger_, "Duplicate EFFORT state interface for joint %s", joint.name.c_str());
        return false;
      }
      has_effort_state = true;
    }
    else
    {
      RCLCPP_FATAL(
        logger_, "Unsupported state interface '%s' for joint %s", interface_info.name.c_str(),
        joint.name.c_str());
      return false;
    }
  }

  if (!has_position_state)
  {
    RCLCPP_FATAL(logger_, "POSITION state interface is required for joint %s", joint.name.c_str());
    return false;
  }

  return true;
}

void KukaRSIHardwareInterfaceBase::CopyGPIOStatesToCommands()
{
  for (size_t i = 0; i < runtime_state_.gpio_states_to_commands_map.size(); i++)
  {
    if (runtime_state_.gpio_states_to_commands_map[i] != -1)
    {
      interface_data_.gpio_commands[i] =
        interface_data_.gpio_states[runtime_state_.gpio_states_to_commands_map[i]];
    }
  }
}

kuka::external::control::kss::GPIOConfiguration KukaRSIHardwareInterfaceBase::ParseGPIOConfig(
  const hardware_interface::InterfaceInfo & info)
{
  kuka::external::control::kss::GPIOConfiguration gpio_config;
  gpio_config.name = info.name;
  gpio_config.enable_limits = info.enable_limits;
  // TODO(komaromi): This might not work from Kilted kaiju onward the get_optional function in the
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
    // TODO(Komaromi): Should this be set to 0?
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
  ResetDiagnostics();

  if (control_state_.status_manager.IsEmergencyStopActive())
  {
    RCLCPP_ERROR(logger_, "Emergency stop is active. Cannot activate hardware interface.");
    return CallbackReturn::FAILURE;
  }

  if (!control_state_.status_manager.IsKrcInExtMode())
  {
    RCLCPP_ERROR(logger_, "KRC not in EXT. Switch to EXT to activate.");
    return CallbackReturn::FAILURE;
  }

  if (!control_state_.status_manager.DrivesPowered())
  {
    RCLCPP_INFO(logger_, "Turning on drives");
    robot_ptr_->TurnOnDrives();

    // Wait for drives to be powered up
    auto start_time = std::chrono::steady_clock::now();
    while (!control_state_.status_manager.DrivesPowered())
    {
      if (
        std::chrono::steady_clock::now() - start_time >
        KukaRSIHardwareInterfaceBase::DRIVES_POWERED_TIMEOUT)
      {
        RCLCPP_ERROR(logger_, "Timeout waiting for drives to power on. Check robot state.");
        return CallbackReturn::FAILURE;
      }
      control_state_.status_manager.UpdateStateInterfaces();
      std::this_thread::sleep_for(KukaRSIHardwareInterfaceBase::DRIVES_POWERED_CHECK_INTERVAL);
    }
    RCLCPP_INFO(logger_, "Drives successfully powered on");
  }

  // Set control mode and cycle time before sending Start request
  ChangeCycleTime();

  const auto control_mode =
    static_cast<kuka::external::control::ControlMode>(control_state_.hw_control_mode_command);

  kuka::external::control::Status control_status = robot_ptr_->StartControlling(control_mode);
  if (control_status.return_code == kuka::external::control::ReturnCode::ERROR)
  {
    RCLCPP_ERROR(logger_, "Starting external control failed: %s", control_status.message);
    return CallbackReturn::FAILURE;
  }

  control_state_.prev_control_mode =
    static_cast<kuka_drivers_core::ControlMode>(control_state_.hw_control_mode_command);

  // We must first receive the initial position of the robot
  // We set a longer timeout, since the first message might not arrive all that fast
  Read(5 * READ_TIMEOUT_MS);
  std::copy(
    interface_data_.position_states.cbegin(), interface_data_.position_states.cend(),
    interface_data_.position_commands.begin());
  if (optional_interface_flags_.has_velocity_command_interface)
  {
    std::fill(
      interface_data_.velocity_commands.begin(), interface_data_.velocity_commands.end(), 0.0);
  }
  if (optional_interface_flags_.has_torque_command_interface)
  {
    std::fill(interface_data_.torque_commands.begin(), interface_data_.torque_commands.end(), 0.0);
  }
  CopyGPIOStatesToCommands();
  Write();

  runtime_state_.msg_received = false;
  runtime_state_.is_active = true;
  interpolation_count_initialized_ = false;

  RCLCPP_INFO(logger_, "Received position data from robot controller!");

  return CallbackReturn::SUCCESS;
}

CallbackReturn KukaRSIHardwareInterfaceBase::extended_deactivation(const rclcpp_lifecycle::State &)
{
  if (runtime_state_.msg_received)
  {
    RCLCPP_INFO(logger_, "Deactivating hardware interface by sending stop signal");

    // StopControlling sometimes calls a blocking read, which could conflict with the read() method,
    // but resource manager handles locking (resources_lock_), so is not necessary here
    robot_ptr_->StopControlling();
  }
  else
  {
    RCLCPP_INFO(logger_, "Message not received, but stop requested. Cancelling RSI program.");
    if (auto reset_status = robot_ptr_->ResetControlSignal();
        reset_status.return_code != kuka::external::control::ReturnCode::OK)
    {
      RCLCPP_WARN(logger_, "Failed to reset control signal.");
    }
    else
    {
      RCLCPP_INFO(logger_, "%s", reset_status.message);
    }
    robot_ptr_->CancelRsiProgram();
  }
  runtime_state_.is_active = false;
  runtime_state_.msg_received = false;
  interpolation_count_initialized_ = false;
  if (control_state_.status_manager.DrivesPowered())
  {
    RCLCPP_INFO(logger_, "Turning off drives");
    robot_ptr_->TurnOffDrives();

    // Wait for drives to be powered off
    auto start_time = std::chrono::steady_clock::now();
    while (control_state_.status_manager.DrivesPowered())
    {
      if (
        std::chrono::steady_clock::now() - start_time >
        KukaRSIHardwareInterfaceBase::DRIVES_POWERED_TIMEOUT)
      {
        RCLCPP_ERROR(logger_, "Timeout waiting for drives to power off. Check robot state.");
        // Return success, as drives off signal is not received in Office mode for iiQKA.OS2
        control_state_.status_manager.UpdateStateInterfaces();
        return CallbackReturn::SUCCESS;
      }
      control_state_.status_manager.UpdateStateInterfaces();
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
  control_signal.AddJointPositionValues(
    interface_data_.position_commands.cbegin(), interface_data_.position_commands.cend());
  if (optional_interface_flags_.has_velocity_command_interface)
  {
    control_signal.AddVelocityValues(
      interface_data_.velocity_commands.cbegin(), interface_data_.velocity_commands.cend());
  }
  if (optional_interface_flags_.has_torque_command_interface)
  {
    control_signal.AddTorqueValues(
      interface_data_.torque_commands.cbegin(), interface_data_.torque_commands.cend());
  }
  control_signal.AddGPIOValues(
    interface_data_.gpio_commands.cbegin(), interface_data_.gpio_commands.cend());

  // measure elapsed time since last motion state message
  // No need to check msg_received_ here, as Write() is only called when msg_received_ is true
  auto now = std::chrono::steady_clock::now();
  auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(
    now - diagnostics_state_.last_msg_received_time);
  // if the delay exceeds threshold, flag as warning
  if (elapsed > KukaRSIHardwareInterfaceBase::kWarningThreshold)
  {
    RCLCPP_WARN(
      logger_,
      "Slow response: %ld us elapsed between motion state was received and control signal sent",
      static_cast<uint64_t>(elapsed.count()));
  }

  auto send_reply_status = robot_ptr_->SendControlSignal();

  if (send_reply_status.return_code != kuka::external::control::ReturnCode::OK)
  {
    RCLCPP_ERROR(logger_, "Sending reply failed: %s", send_reply_status.message);
    throw std::runtime_error("Error sending reply");
  }
}

void KukaRSIHardwareInterfaceBase::ResetDiagnostics()
{
  // Reset diagnostics related variables
  diagnostics_state_.packet_loss_count = 0;
  diagnostics_state_.last_ipoc = 0;
  diagnostics_state_.last_msg_received_time = std::chrono::steady_clock::time_point{};
}

kuka::external::control::Status KukaRSIHardwareInterfaceBase::ChangeCycleTime()
{
  const RsiCycleTime cycle_time = static_cast<RsiCycleTime>(control_state_.cycle_time_command);

  if (control_state_.prev_cycle_time != cycle_time)
  {
    RCLCPP_INFO(
      logger_, "Changing RSI cycle time to %s",
      kuka::external::control::kss::CycleTimeToString(cycle_time));
    auto status = robot_ptr_->SetCycleTime(cycle_time);
    if (status.return_code != kuka::external::control::ReturnCode::OK)
    {
      return status;
    }
    control_state_.prev_cycle_time = cycle_time;
  }

  return kuka::external::control::Status(kuka::external::control::ReturnCode::OK);
}

void KukaRSIHardwareInterfaceBase::initialize_command_interfaces(
  kuka_drivers_core::ControlMode control_mode, RsiCycleTime cycle_time)
{
  control_state_.prev_control_mode = control_mode;
  control_state_.prev_cycle_time = cycle_time;
  control_state_.hw_control_mode_command = static_cast<double>(control_mode);
  control_state_.cycle_time_command = static_cast<double>(cycle_time);
}

bool KukaRSIHardwareInterfaceBase::LoadXmlConfig(
  const std::string & path, kuka::external::control::kss::Configuration & config) const
{
  RsiXmlConfigurationParser parser(logger_);
  return parser.Load(path, info_.joints.size(), info_.gpios[0].state_interfaces.size(), config);
}

void KukaRSIHardwareInterfaceBase::ConfigureJoints(
  kuka::external::control::kss::Configuration & config) const
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
