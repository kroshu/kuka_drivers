// Copyright 2023 Aron Svastits
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
#include "kuka_rsi_driver/hardware_interface_rsi_only.hpp"

namespace kuka_rsi_driver
{
CallbackReturn KukaRSIHardwareInterface::on_configure(const rclcpp_lifecycle::State &)
{
  const bool setup_success = SetupRobot();
  return setup_success ? CallbackReturn::SUCCESS : CallbackReturn::ERROR;
}

CallbackReturn KukaRSIHardwareInterface::on_activate(const rclcpp_lifecycle::State &)
{
  Read(10 * READ_TIMEOUT_MS);

  std::copy(hw_states_.cbegin(), hw_states_.cend(), hw_commands_.begin());
  CopyGPIOStatesToCommands();

  Write();

  msg_received_ = false;
  is_active_ = true;

  RCLCPP_INFO(logger_, "Received position data from robot controller!");
  server_state_ = static_cast<double>(kuka_drivers_core::HardwareEvent::CONTROL_STARTED);

  return CallbackReturn::SUCCESS;
}

CallbackReturn KukaRSIHardwareInterface::on_deactivate(const rclcpp_lifecycle::State &)
{
  // If control is active, send stop signal
  if (msg_received_)
  {
    RCLCPP_INFO(logger_, "Deactivating hardware interface by sending stop signal");

    // StopControlling sometimes calls a blocking read, which could conflict with the read() method,
    // but resource manager handles locking (resources_lock_), so is not necessary here
    robot_ptr_->StopControlling();
  }

  is_active_ = false;
  msg_received_ = false;

  RCLCPP_INFO(logger_, "Stop requested!");
  return CallbackReturn::SUCCESS;
}

CallbackReturn KukaRSIHardwareInterface::on_cleanup(const rclcpp_lifecycle::State &)
{
  robot_ptr_.reset();
  return CallbackReturn::SUCCESS;
}

return_type KukaRSIHardwareInterface::read(const rclcpp::Time &, const rclcpp::Duration &)
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

return_type KukaRSIHardwareInterface::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  // If control is not started or a request is missed, do not send back anything
  if (!msg_received_)
  {
    return return_type::OK;
  }

  Write();

  return return_type::OK;
}

bool KukaRSIHardwareInterface::SetupRobot()
{
  RCLCPP_INFO(logger_, "Initiating network setup...");

  kuka::external::control::kss::Configuration config;
  config.installed_interface =
    kuka::external::control::kss::Configuration::InstalledInterface::RSI_ONLY;
  config.dof = info_.joints.size();
  RCLCPP_INFO(logger_, "Configured GPIO commands:");
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

  RCLCPP_INFO(logger_, "Configured GPIO states:");
  for (const auto & gpio_state : info_.gpios[0].state_interfaces)
  {
    RCLCPP_INFO(
      logger_, "Name: %s, Data type: %s, Initial value: %s, Enable limits: %s, Min: %s, Max: %s",
      gpio_state.name.c_str(), gpio_state.data_type.c_str(), gpio_state.initial_value.c_str(),
      gpio_state.enable_limits ? "true" : "false", gpio_state.min.c_str(), gpio_state.max.c_str());

    // TODO (Komaromi): Add size, and parameters
    config.gpio_state_configs.emplace_back(ParseGPIOConfig(gpio_state));
  }

  robot_ptr_ = std::make_unique<kuka::external::control::kss::Robot>(config);

  const auto setup = robot_ptr_->Setup();
  if (setup.return_code != kuka::external::control::ReturnCode::OK)
  {
    RCLCPP_ERROR(logger_, "Setup failed: %s", setup.message);
    return false;
  }

  RCLCPP_INFO(logger_, "Network setup successful!");

  return true;
}

void KukaRSIHardwareInterface::Read(const int64_t request_timeout)
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
    server_state_ = static_cast<double>(kuka_drivers_core::HardwareEvent::ERROR);
    on_deactivate(get_lifecycle_state());
  }
}

void KukaRSIHardwareInterface::Write()
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
void KukaRSIHardwareInterface::CopyGPIOStatesToCommands()
{
  for (size_t i = 0; i < gpio_states_to_commands_map_.size(); i++)
  {
    if (gpio_states_to_commands_map_[i] != -1)
    {
      hw_gpio_commands_[i] = hw_gpio_states_[gpio_states_to_commands_map_[i]];
    }
  }
}

kuka::external::control::kss::GPIOConfiguration KukaRSIHardwareInterface::ParseGPIOConfig(
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

void KukaRSIHardwareInterface::CreateRobotInstance(const kuka::external::control::kss::Configuration& config)
{
  robot_ptr_ = std::make_unique<kuka::external::control::kss::Robot>(config);
}
}  // namespace kuka_rsi_driver

PLUGINLIB_EXPORT_CLASS(
  kuka_rsi_driver::KukaRSIHardwareInterface, hardware_interface::SystemInterface)
