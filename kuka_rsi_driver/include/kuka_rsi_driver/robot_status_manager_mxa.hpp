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

#ifndef KUKA_RSI_DRIVER__ROBOT_STATUS_MANAGER_MXA_HPP_
#define KUKA_RSI_DRIVER__ROBOT_STATUS_MANAGER_MXA_HPP_

#include <atomic>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "kuka/external-control-sdk/kss/mxa/client.h"
#include "kuka/external-control-sdk/kss/mxa/extension.h"
#include "kuka_rsi_driver/hardware_interface_mxa_rsi.hpp"

namespace kuka_rsi_driver
{

class StatusInterfaces
{
public:
  StatusInterfaces & operator=(const kuka::external::control::kss::mxa::StatusUpdate & update)
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

  void RegisterStateInterfaces(std::vector<hardware_interface::StateInterface> & state_interfaces)
  {
    const std::vector<std::pair<std::string, double *>> interface_data = {
      {hardware_interface::CONTROL_MODE, &control_mode_},
      {hardware_interface::CYCLE_TIME, &cycle_time_},
      {hardware_interface::DRIVES_POWERED, &drives_powered_},
      {hardware_interface::EMERGENCY_STOP, &emergency_stop_},
      {hardware_interface::GUARD_STOP, &guard_stop_},
      {hardware_interface::IN_MOTION, &in_motion_},
      {hardware_interface::MOTION_POSSIBLE, &motion_possible_},
      {hardware_interface::OPERATION_MODE, &operation_mode_},
      {hardware_interface::ROBOT_STOPPED, &robot_stopped_}};

    for (const auto & [name, value_ptr] : interface_data)
    {
      state_interfaces.emplace_back(hardware_interface::STATE_PREFIX, name, value_ptr);
    }
  }

  bool IsOperationModeExt()
  {
    return static_cast<uint8_t>(operation_mode_) ==
           static_cast<uint8_t>(kuka::external::control::OperationMode::EXT);
  }

  bool DrivesPowered() { return static_cast<bool>(drives_powered_); }

  bool IsEmergencyStopActive() { return static_cast<bool>(emergency_stop_); }

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
  void RegisterStateInterfaces(std::vector<hardware_interface::StateInterface> & state_interfaces)
  {
    status_interfaces_.RegisterStateInterfaces(state_interfaces);
  }

  void SetStatusInterfaces(const kuka::external::control::kss::mxa::StatusUpdate & update)
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

private:
  StatusInterfaces status_interfaces_;         // Used as ROS 2 state interface
  StatusInterfaces actual_status_interfaces_;  // Stores actual state
  std::mutex status_mtx_;
};

}  // namespace kuka_rsi_driver

#endif  // KUKA_RSI_DRIVER__ROBOT_STATUS_MANAGER_MXA_HPP_
