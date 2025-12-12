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
  kuka::external::control::kss::Configuration rsi_config;
  rsi_config.installed_interface =
    kuka::external::control::kss::Configuration::InstalledInterface::RSI_ONLY;
  const bool setup_success = SetupRobot(rsi_config);
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

void KukaRSIHardwareInterface::CreateRobotInstance(
  const kuka::external::control::kss::Configuration & config)
{
  robot_ptr_ = std::make_unique<kuka::external::control::kss::Robot>(config);
}
}  // namespace kuka_rsi_driver

PLUGINLIB_EXPORT_CLASS(
  kuka_rsi_driver::KukaRSIHardwareInterface, hardware_interface::SystemInterface)
