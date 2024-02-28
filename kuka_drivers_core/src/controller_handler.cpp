// Copyright 2023 Komáromi Sándor
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
#include <string>
#include <utility>
#include <vector>

#include "kuka_drivers_core/controller_handler.hpp"

namespace kuka_drivers_core
{
ControllerHandler::ControllerHandler(std::vector<std::string> fixed_controllers)
: fixed_controllers_(fixed_controllers.begin(), fixed_controllers.end())
{
}

bool ControllerHandler::UpdateControllerName(
  const ControllerType controller_type, const std::string & controller_name)
{
  switch (controller_type)
  {
    case ControllerType::JOINT_POSITION_CONTROLLER_TYPE:
      control_mode_map_[ControlMode::JOINT_POSITION_CONTROL].standard_controller = controller_name;
      control_mode_map_[ControlMode::JOINT_IMPEDANCE_CONTROL].standard_controller = controller_name;
      break;
    case ControllerType::CARTESIAN_POSITION_CONTROLLER_TYPE:
      control_mode_map_[ControlMode::CARTESIAN_POSITION_CONTROL].standard_controller =
        controller_name;
      control_mode_map_[ControlMode::CARTESIAN_IMPEDANCE_CONTROL].standard_controller =
        controller_name;
      break;
    case ControllerType::JOINT_VELOCITY_CONTROLLER_TYPE:
      control_mode_map_[ControlMode::JOINT_VELOCITY_CONTROL].standard_controller = controller_name;
      break;
    case ControllerType::TWIST_CONTROLLER_TYPE:
      control_mode_map_[ControlMode::CARTESIAN_VELOCITY_CONTROL].standard_controller =
        controller_name;
      break;
    case ControllerType::JOINT_IMPEDANCE_CONTROLLER_TYPE:
      control_mode_map_[ControlMode::JOINT_IMPEDANCE_CONTROL].impedance_controller =
        controller_name;
      break;
    case ControllerType::CARTESIAN_IMPEDANCE_CONTROLLER_TYPE:
      control_mode_map_[ControlMode::CARTESIAN_IMPEDANCE_CONTROL].impedance_controller =
        controller_name;
      break;
    case ControllerType::TORQUE_CONTROLLER_TYPE:
      control_mode_map_[ControlMode::JOINT_TORQUE_CONTROL].standard_controller = controller_name;
      break;
    case ControllerType::WRENCH_CONTROLLER_TYPE:
      control_mode_map_[ControlMode::WRENCH_CONTROL].standard_controller = controller_name;
      break;
    default:
      RCLCPP_INFO(rclcpp::get_logger("ControllerHandler"), "Invalid Controller type");
      return false;
  }
  return true;
}

std::pair<std::vector<std::string>, std::vector<std::string>>
ControllerHandler::GetControllersForSwitch(ControlMode new_control_mode)
{
  if (control_mode_map_.find(new_control_mode) == control_mode_map_.end())
  {
    // Not valid control mode, throw exception
    throw std::out_of_range("Attribute new_control_mode is out of range");
  }

  if (new_control_mode == ControlMode::CONTROL_MODE_UNSPECIFIED)
  {
    throw std::logic_error("CONTROL_MODE_UNSPECIFIED is not valid control mode");
  }

  // Set controllers which should be activated and deactivated
  activate_controllers_.clear();
  auto control_mode_controllers = control_mode_map_.at(new_control_mode);
  activate_controllers_.insert(control_mode_controllers.standard_controller);
  if (!control_mode_controllers.impedance_controller.empty())
  {
    activate_controllers_.insert(control_mode_controllers.impedance_controller);
  }

  activate_controllers_.insert(fixed_controllers_.begin(), fixed_controllers_.end());

  deactivate_controllers_ = active_controllers_;

  // Goes through every controllers that should be activated
  for (auto activate_controllers_it = activate_controllers_.begin();
       activate_controllers_it != activate_controllers_.end();)
  {
    // Finds the controller in the deactivate controllers
    auto deactivate_controllers_it = deactivate_controllers_.find(*activate_controllers_it);
    if (deactivate_controllers_it != deactivate_controllers_.end())
    {
      // Delete those controllers which not need to be activated or deactivated.
      activate_controllers_it = activate_controllers_.erase(activate_controllers_it);
      deactivate_controllers_.erase(deactivate_controllers_it);
    }
    else
    {
      ++activate_controllers_it;
    }
  }

  return std::make_pair(
    std::vector<std::string>(activate_controllers_.begin(), activate_controllers_.end()),
    std::vector<std::string>(deactivate_controllers_.begin(), deactivate_controllers_.end()));
}

std::vector<std::string> ControllerHandler::GetControllersForDeactivation()
{
  deactivate_controllers_ = active_controllers_;
  return std::vector<std::string>(deactivate_controllers_.begin(), deactivate_controllers_.end());
}

void ControllerHandler::ApproveControllerActivation()
{
  if (!activate_controllers_.empty())
  {
    active_controllers_.insert(activate_controllers_.begin(), activate_controllers_.end());
    activate_controllers_.clear();
  }
}

bool ControllerHandler::ApproveControllerDeactivation()
{
  for (auto && controller : deactivate_controllers_)
  {
    auto active_controller_it = active_controllers_.find(controller);
    if (active_controller_it == active_controllers_.end())
    {
      // We should not reach this, active controllers should always contain the ones to deactivate
      return false;
    }
    active_controllers_.erase(active_controller_it);
  }
  deactivate_controllers_.clear();

  return true;
}

std::vector<std::string> ControllerHandler::GetControllersForMode(ControlMode control_mode)
{
  std::vector<std::string> controllers;

  auto controller_types = control_mode_map_.at(control_mode);
  controllers.push_back(controller_types.standard_controller);
  if (!controller_types.impedance_controller.empty())
  {
    controllers.push_back(controller_types.impedance_controller);
  }
  return controllers;
}

}  // namespace kuka_drivers_core
