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

#include "kuka_drivers_core/control_mode.hpp"
#include "kuka_drivers_core/hardware_interface_types.hpp"

#include "kuka_iiqka_drivers_core/event_observer.hpp"
#include "kuka_iiqka_drivers_core/hardware_interface_base.hpp"

namespace kuka_eac
{

CallbackReturn KukaEACHardwareInterfaceBase::on_configure(const rclcpp_lifecycle::State &)
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

CallbackReturn KukaEACHardwareInterfaceBase::on_activate(const rclcpp_lifecycle::State &)
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

CallbackReturn KukaEACHardwareInterfaceBase::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger("KukaEACHardwareInterface"), "Deactivating hardware interface");

  stop_requested_ = true;

  return CallbackReturn::SUCCESS;
}

bool KukaEACHardwareInterfaceBase::SetupQoS()
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

void KukaEACHardwareInterfaceBase::set_server_event(kuka_drivers_core::HardwareEvent event)
{
  std::lock_guard<std::mutex> lk(event_mutex_);
  last_event_ = event;
}
}  // namespace kuka_eac
