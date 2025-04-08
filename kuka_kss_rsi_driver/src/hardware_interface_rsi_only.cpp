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

#include "kuka_drivers_core/hardware_interface_types.hpp"
#include "kuka_kss_rsi_driver/hardware_interface_rsi_only.hpp"

namespace kuka_kss_rsi_driver
{
CallbackReturn KukaRSIHardwareInterface::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  hw_states_.resize(info_.joints.size(), 0.0);
  hw_commands_.resize(info_.joints.size(), 0.0);

  // Checking joint config structure
  for (const auto & joint : info_.joints)
  {
    if (!CheckJointInterfaces(joint))
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
  auto & gpio = info_.gpios[0];
  // Check gpio component name
  if (gpio.name != hardware_interface::IO_PREFIX)
  {
    RCLCPP_FATAL(logger_, "expecting gpio component called 'GPIO' first");
    return CallbackReturn::ERROR;
  }
  // TODO (komaromi): Somehow check how many IOs are in the interfaces. RSI can receive and send
  // 8192 bits, but its not equal with 8192 IOs.

  hw_gpio_states_.resize(gpio.state_interfaces.size(), 0.0);
  hw_gpio_commands_.resize(gpio.command_interfaces.size(), 0.0);

  RCLCPP_INFO(logger_, "Client IP: %s", info_.hardware_parameters["client_ip"].c_str());

  first_write_done_ = false;
  is_active_ = false;
  msg_received_ = false;
  stop_requested_ = false;

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> KukaRSIHardwareInterface::export_state_interfaces()
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

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
KukaRSIHardwareInterface::export_command_interfaces()
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

CallbackReturn KukaRSIHardwareInterface::on_configure(const rclcpp_lifecycle::State &)
{
  const bool setup_success = SetupRobot();
  return setup_success ? CallbackReturn::SUCCESS : CallbackReturn::ERROR;
}

CallbackReturn KukaRSIHardwareInterface::on_activate(const rclcpp_lifecycle::State &)
{
  stop_requested_ = false;

  Read(10 * REQUEST_TIMEOUT_MS);
  std::copy(hw_states_.cbegin(), hw_states_.cend(), hw_commands_.begin());
  Write();

  msg_received_ = false;
  first_write_done_ = true;
  is_active_ = true;

  RCLCPP_INFO(logger_, "Received position data from robot controller!");

  return CallbackReturn::SUCCESS;
}

CallbackReturn KukaRSIHardwareInterface::on_deactivate(const rclcpp_lifecycle::State &)
{
  stop_requested_ = true;
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
  if (!is_active_)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
    return return_type::OK;
  }

  Read(REQUEST_TIMEOUT_MS);
  return return_type::OK;
}

return_type KukaRSIHardwareInterface::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  if (!is_active_ || !msg_received_ || !first_write_done_)
  {
    return return_type::OK;
  }

  Write();

  return return_type::OK;
}

bool KukaRSIHardwareInterface::SetupRobot()
{
  RCLCPP_INFO(logger_, "Initiating network setup...");

  using Configuration = kuka::external::control::kss::Configuration;
  Configuration config;
  config.installed_interface = Configuration::InstalledInterface::RSI_ONLY;
  config.client_ip_address = info_.hardware_parameters["client_ip"];
  config.dof = info_.joints.size();
  config.gpio_command_size = info_.gpios[0].command_interfaces.size();
  config.gpio_state_size = info_.gpios[0].state_interfaces.size();

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
  std::chrono::milliseconds timeout(request_timeout);
  const auto motion_state_status = robot_ptr_->ReceiveMotionState(timeout);

  msg_received_ = motion_state_status.return_code == kuka::external::control::ReturnCode::OK;
  if (msg_received_)
  {
    const auto & req_message = robot_ptr_->GetLastMotionState();
    const auto & positions = req_message.GetMeasuredPositions();
    const auto & gpio_values = req_message.GetGPIOValues();

    // TODO (Komaromi): Delete later
    for (std::size_t i = 0; i < req_message.GetGPIOValues().size(); i++)
    {
      RCLCPP_INFO(
        rclcpp::get_logger("KukaEACHardwareInterface"), "Signal_%ld - Value: %d, type: %d",
        req_message.GetGPIOValues().at(i)->GetGPIOConfig()->GetGPIOId(),
        req_message.GetGPIOValues().at(i)->GetBoolValue(),
        req_message.GetGPIOValues().at(i)->GetGPIOConfig()->GetValueType());
    }

    std::copy(positions.cbegin(), positions.cend(), hw_states_.begin());
    // Save IO states
    for (size_t i = 0; i < hw_gpio_states_.size(); i++)
    {
      switch (gpio_values.at(i)->GetGPIOConfig()->GetValueType())
      {
        case kuka::external::control::GPIOValueType::BOOL_VALUE:
          hw_gpio_states_[i] = static_cast<double>(gpio_values[i]->GetBoolValue());
          break;
        case kuka::external::control::GPIOValueType::DOUBLE_VALUE:
          hw_gpio_states_[i] = static_cast<double>(gpio_values[i]->GetDoubleValue());
          break;
        case kuka::external::control::GPIOValueType::RAW_VALUE:
          hw_gpio_states_[i] = static_cast<double>(gpio_values[i]->GetRawValue());
          break;
        case kuka::external::control::GPIOValueType::LONG_VALUE:
          hw_gpio_states_[i] = static_cast<double>(gpio_values[i]->GetLongValue());
          break;
        default:
          RCLCPP_ERROR(
            rclcpp::get_logger("KukaEACHardwareInterface"),
            "No signal value type found. (Should be dead code)");
      }
    }
    // TODO (Komaromi): Delete later
    for (auto && gpio : hw_gpio_states_)
    {
      RCLCPP_INFO(rclcpp::get_logger("KukaEACHardwareInterface"), "Signal value: %f", gpio);
    }
  }
}

void KukaRSIHardwareInterface::Write()
{
  // Write values to hardware interface
  auto & control_signal = robot_ptr_->GetControlSignal();
  control_signal.AddJointPositionValues(hw_commands_.cbegin(), hw_commands_.cend());
  control_signal.AddGPIOValues(hw_gpio_commands_.cbegin(), hw_gpio_commands_.cend());

  for (auto && gpio : control_signal.GetGPIOValues())
  {
    RCLCPP_INFO(
      logger_, "Signal_%ld - Value: %d", gpio->GetGPIOConfig()->GetGPIOId(), gpio->GetBoolValue());
  }

  kuka::external::control::Status send_reply_status;
  if (stop_requested_)
  {
    RCLCPP_INFO(logger_, "Sending stop signal");
    first_write_done_ = false;
    is_active_ = false;
    msg_received_ = false;
    send_reply_status = robot_ptr_->StopControlling();
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