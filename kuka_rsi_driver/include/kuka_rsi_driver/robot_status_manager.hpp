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

#ifndef KUKA_RSI_DRIVER__ROBOT_STATUS_MANAGER_HPP_
#define KUKA_RSI_DRIVER__ROBOT_STATUS_MANAGER_HPP_

#include <atomic>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "hardware_interface/hardware_info.hpp"
#include "kuka_drivers_core/hardware_interface_types.hpp"

#include "kuka/external-control-sdk/kss/status_update.h"

namespace kuka_rsi_driver
{

class StatusInterfaces
{
public:
  StatusInterfaces & operator=(const kuka::external::control::kss::StatusUpdate & update)
  {
    control_mode_ = static_cast<double>(update.control_mode_);
    cycle_time_ = static_cast<double>(update.cycle_time_);
    drives_powered_ = static_cast<double>(update.drives_powered_);
    emergency_stop_ = static_cast<double>(update.emergency_stop_);
    guard_stop_ = static_cast<double>(update.guard_stop_);
    in_motion_ = static_cast<double>(update.in_motion_);
    motion_possible_ = static_cast<double>(update.motion_possible_);
    operation_mode_ = static_cast<double>(update.operation_mode_);
    robot_stopped_ = static_cast<double>(update.robot_stopped_);
    return *this;
  }

  // Names (without the state/ prefix) and current values of the interfaces this class backs.
  // Kept as one list so ExportUnlistedStateInterfaceDescriptions() (declaring the interfaces)
  // and GetStateValues() (reading them back each cycle) can't drift apart.
  std::vector<std::pair<std::string, double>> GetNamedValues() const
  {
    return {
      {hardware_interface::CONTROL_MODE, control_mode_},
      {hardware_interface::CYCLE_TIME, cycle_time_},
      {hardware_interface::DRIVES_POWERED, drives_powered_},
      {hardware_interface::EMERGENCY_STOP, emergency_stop_},
      {hardware_interface::GUARD_STOP, guard_stop_},
      {hardware_interface::IN_MOTION, in_motion_},
      {hardware_interface::MOTION_POSSIBLE, motion_possible_},
      {hardware_interface::OPERATION_MODE, operation_mode_},
      {hardware_interface::ROBOT_STOPPED, robot_stopped_}};
  }

  std::vector<hardware_interface::InterfaceDescription> ExportUnlistedStateInterfaceDescriptions(
    const std::string & interface_prefix) const
  {
    std::vector<hardware_interface::InterfaceDescription> descriptions;
    for (const auto & [name, value] : GetNamedValues())
    {
      hardware_interface::InterfaceInfo info{};
      info.name = name;
      info.initial_value = "0";
      descriptions.emplace_back(interface_prefix + hardware_interface::STATE_PREFIX, info);
    }
    return descriptions;
  }

  std::vector<std::pair<std::string, double>> GetStateValues(
    const std::string & interface_prefix) const
  {
    std::vector<std::pair<std::string, double>> values;
    for (auto & [name, value] : GetNamedValues())
    {
      values.emplace_back(interface_prefix + hardware_interface::STATE_PREFIX + "/" + name, value);
    }
    return values;
  }

  bool IsOperationModeExt()
  {
    return static_cast<uint8_t>(operation_mode_) ==
           static_cast<uint8_t>(kuka::external::control::OperationMode::EXT);
  }

  bool DrivesPowered() { return static_cast<bool>(drives_powered_); }

  bool IsEmergencyStopActive() { return static_cast<bool>(emergency_stop_); }

  bool IsMotionPossible() { return static_cast<bool>(motion_possible_); }

private:
  double control_mode_ = 0.0;
  double cycle_time_ = 0.0;
  double drives_powered_ = 0.0;
  double emergency_stop_ = 0.0;
  double guard_stop_ = 0.0;
  double in_motion_ = 0.0;
  double motion_possible_ = 0.0;
  double operation_mode_ = 0.0;
  double robot_stopped_ = 0.0;
};

class StatusManager
{
public:
  std::vector<hardware_interface::InterfaceDescription> ExportUnlistedStateInterfaceDescriptions(
    const std::string & interface_prefix) const
  {
    return status_interfaces_.ExportUnlistedStateInterfaceDescriptions(interface_prefix);
  }

  std::vector<std::pair<std::string, double>> GetStateValues(const std::string & interface_prefix)
  {
    std::lock_guard<std::mutex> lck{status_mtx_};
    return status_interfaces_.GetStateValues(interface_prefix);
  }

  void SetStatusInterfaces(const kuka::external::control::kss::StatusUpdate & update)
  {
    std::lock_guard<std::mutex> lck{status_mtx_};
    actual_status_interfaces_ = update;
  }

  void UpdateStateInterfaces()
  {
    std::lock_guard<std::mutex> lck{status_mtx_};
    status_interfaces_ = actual_status_interfaces_;
  }

  bool IsKrcInExtMode()
  {
    std::lock_guard<std::mutex> lck{status_mtx_};
    return status_interfaces_.IsOperationModeExt();
  }

  bool DrivesPowered()
  {
    std::lock_guard<std::mutex> lck{status_mtx_};
    return status_interfaces_.DrivesPowered();
  }

  bool IsEmergencyStopActive()
  {
    std::lock_guard<std::mutex> lck{status_mtx_};
    return status_interfaces_.IsEmergencyStopActive();
  }

  bool IsMotionPossible()
  {
    std::lock_guard<std::mutex> lck{status_mtx_};
    return status_interfaces_.IsMotionPossible();
  }

private:
  StatusInterfaces status_interfaces_;         // Used as ROS 2 state interface
  StatusInterfaces actual_status_interfaces_;  // Stores actual state
  std::mutex status_mtx_;
};

}  // namespace kuka_rsi_driver

#endif  // KUKA_RSI_DRIVER__ROBOT_STATUS_MANAGER_HPP_
