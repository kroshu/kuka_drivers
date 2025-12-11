// Copyright 2025 KUKA Hungaria Kft.
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

#include <chrono>
#include <regex>
#include <thread>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

#include "kuka_drivers_core/hardware_interface_types.hpp"
#include "kuka_rsi_driver/event_observer_mxa_rsi.hpp"
#include "kuka_rsi_driver/hardware_interface_mxa_rsi.hpp"

namespace kuka_rsi_driver
{

CallbackReturn KukaMxaRsiHardwareInterface::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  if (KukaRSIHardwareInterfaceBase::on_init(info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  RCLCPP_INFO(logger_, "Controller IP: %s", info_.hardware_parameters["controller_ip"].c_str());

  auto it = info_.hardware_parameters.find("verify_robot_model");
  verify_robot_model_ = (it != info_.hardware_parameters.end() && it->second == "True");
  RCLCPP_INFO(
    logger_, "Robot model verification: %s", verify_robot_model_ ? "enabled" : "disabled");

  cycle_time_command_ = 0.0;
  hw_control_mode_command_ = 0.0;

  return CallbackReturn::SUCCESS;
}


std::vector<hardware_interface::CommandInterface>
KukaMxaRsiHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  command_interfaces = KukaRSIHardwareInterfaceBase::export_command_interfaces();

  command_interfaces.emplace_back(
    hardware_interface::CONFIG_PREFIX, hardware_interface::CONTROL_MODE, &hw_control_mode_command_);

  command_interfaces.emplace_back(
    hardware_interface::CONFIG_PREFIX, hardware_interface::CYCLE_TIME, &cycle_time_command_);

  return command_interfaces;
}

CallbackReturn KukaMxaRsiHardwareInterface::on_configure(const rclcpp_lifecycle::State &)
{
  kuka::external::control::kss::Configuration mxa_config;
  mxa_config.kli_ip_address = info_.hardware_parameters["controller_ip"];;
  if (!SetupRobot(mxa_config))
  {
    return CallbackReturn::ERROR;
  }

  auto status = robot_ptr_->RegisterEventHandler(std::move(std::make_unique<EventObserverMxa>(this)));
  if (status.return_code == kuka::external::control::ReturnCode::ERROR)
  {
    RCLCPP_ERROR(logger_, "Creating event observer failed: %s", status.message);
  }

  status = robot_ptr_->RegisterEventHandlerExtension(std::make_unique<EventHandlerExtensionMxa>(this));
  if (status.return_code == kuka::external::control::ReturnCode::ERROR)
  {
    RCLCPP_INFO(logger_, "Creating event handler extension failed: %s", status.message);
  }

  status = robot_ptr_->RegisterStatusResponseHandler(
    std::make_unique<StatusUpdateHandler>(this, &status_manager_));
  if (status.return_code != kuka::external::control::ReturnCode::OK)
  {
    RCLCPP_ERROR(logger_, "Registering status response handler failed: %s", status.message);
    return CallbackReturn::ERROR;
  }

  // Wait for the response to arrive from the controller
  std::unique_lock<std::mutex> lk{init_mtx_};
  init_cv_.wait(lk, [this] { return init_report_.sequence_complete; });

  if (verify_robot_model_)
  {
    if (!init_report_.ok)
    {
      RCLCPP_ERROR(
        logger_, "The driver is incompatible with the current hardware and software setup: %s",
        init_report_.reason.c_str());
      robot_ptr_.reset();
      return CallbackReturn::ERROR;
    }
    RCLCPP_INFO(logger_, "The driver is compatible with the current hardware and software setup");
  }
  else
  {
    RCLCPP_INFO(logger_, "Robot model verification is disabled, proceeding with the connection.");
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn KukaMxaRsiHardwareInterface::on_activate(const rclcpp_lifecycle::State &state)
{
  return KukaRSIHardwareInterfaceBase::extended_activation(state);
}

CallbackReturn KukaMxaRsiHardwareInterface::on_deactivate(const rclcpp_lifecycle::State &state)
{
  return KukaRSIHardwareInterfaceBase::extended_deactivation(state);
}

return_type KukaMxaRsiHardwareInterface::read(const rclcpp::Time & time, const rclcpp::Duration & duration)
{
  status_manager_.UpdateStateInterfaces();

  return KukaRSIHardwareInterfaceBase::read(time, duration);
}

void KukaMxaRsiHardwareInterface::mxa_init(const InitializationData & init_data)
{
  if (init_data.GetTotalAxisCount() == 0)
  {
    RCLCPP_WARN(
      logger_,
      "Skipping robot model verification, as it is not supported for mxA versions below 4.0");
    init_report_ = {true, true, ""};
  }
  else
  {
    std::lock_guard<std::mutex> lk{init_mtx_};
    if (info_.joints.size() != init_data.GetTotalAxisCount())
    {
      std::ostringstream oss;
      oss << "Mismatch in axis count: Driver expects " << info_.joints.size()
          << ", but mxAutomation server reported " << init_data.GetTotalAxisCount();
      init_report_ = {true, false, oss.str()};
    }
    else
    {
      init_report_ = {true, true, ""};
    }
  }
  init_cv_.notify_one();
}

void KukaMxaRsiHardwareInterface::Read(const int64_t request_timeout)
{
  if (status_manager_.IsEmergencyStopActive())
  {
    RCLCPP_ERROR(logger_, "Emergency stop detected!");
    set_server_event(kuka_drivers_core::HardwareEvent::ERROR);
  }
  else if (!status_manager_.DrivesPowered())
  {
    RCLCPP_ERROR(logger_, "Drives are not powered!");
    set_server_event(kuka_drivers_core::HardwareEvent::ERROR);
  }

  KukaRSIHardwareInterfaceBase::Read(request_timeout);
}

void KukaMxaRsiHardwareInterface::Write()
{
  KukaRSIHardwareInterfaceBase::extended_write();
}


void KukaMxaRsiHardwareInterface::CreateRobotInstance(const kuka::external::control::kss::Configuration& config)
{
  robot_ptr_ = std::make_unique<kuka::external::control::kss::mxa::Robot>(config);
}

}  // namespace kuka_rsi_driver

PLUGINLIB_EXPORT_CLASS(
  kuka_rsi_driver::KukaMxaRsiHardwareInterface, hardware_interface::SystemInterface)
