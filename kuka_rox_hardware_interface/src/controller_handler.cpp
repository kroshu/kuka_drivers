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

#include "kuka_rox_hw_interface/controller_handler.hpp"

using namespace kuka::motion::external;  // NOLINT

namespace kuka_rox
{
controller_handler::controller_handler(std::vector<std::string> fixed_controllers)
: fixed_controllers_(fixed_controllers)
{
  control_mode_map_.emplace(
    std::make_pair(
      ExternalControlMode::POSITION_CONTROL,
      std::vector<std::string>(POSITION_CONTROLLERS_SIZE)));
  control_mode_map_.emplace(
    std::make_pair(
      ExternalControlMode::JOINT_IMPEDANCE_CONTROL,
      std::vector<std::string>(IMPEDANCE_CONTROLLERS_SIZE)));
  control_mode_map_.emplace(
    std::make_pair(
      ExternalControlMode::TORQUE_CONTROL,
      std::vector<std::string>(TORQUE_CONTROLLERS_SIZE)));

}

bool controller_handler::Update_controller_name(
  const controller_handler::controller_type controller_type,
  const std::string & controller_name)
{
  switch (controller_type) {
    case POSITION_CONTROLLER:
      control_mode_map_.at(ExternalControlMode::POSITION_CONTROL).at(NORMAL_CONROLLERS_POS) =
        controller_name;
      control_mode_map_.at(ExternalControlMode::JOINT_IMPEDANCE_CONTROL).at(NORMAL_CONROLLERS_POS) =
        controller_name;
      return true;
      break;
    case IMPEDANCE_CONTROLLER:
      control_mode_map_.at(ExternalControlMode::JOINT_IMPEDANCE_CONTROL).at(
        IMPEDANCE_CONTROLLERS_POS) = controller_name;
      return true;
      break;
    case TORQUE_CONTROLLER:
      control_mode_map_.at(ExternalControlMode::TORQUE_CONTROL).at(NORMAL_CONROLLERS_POS) =
        controller_name;
      return true;
      break;
    default:
      RCLCPP_INFO(rclcpp::get_logger("ControllerHandler"), "Invalid Controller type");
      return false;
      break;
  }

}

std::pair<std::vector<std::string>, std::vector<std::string>>
controller_handler::Get_activate_deactivate_controllers(ExternalControlMode new_control_mode)
{
  if (control_mode_map_.find(new_control_mode) == control_mode_map_.end()) {
    // Not valid control mode, threw error

  }

  // Set controllers wich should be activated and deactivated
  activate_controllers_ = control_mode_map_.at(new_control_mode);
  deactivate_controllers_ = get_active_controllers();

  // Goes through every controllers that should be deactivated
  for (auto deactivate_controllers_it = deactivate_controllers_.begin();
    deactivate_controllers_it != deactivate_controllers_.end(); ++deactivate_controllers_it)
  {
    // Goes through every controllers that should be activated
    for (auto activate_controllers_it = activate_controllers_.begin();
      activate_controllers_it != activate_controllers_.end(); ++activate_controllers_it)
    {
      if (*activate_controllers_it == *deactivate_controllers_it) {
        // Delete those controllers wich not need to be activated or deactivated.
        activate_controllers_.erase(activate_controllers_it);
        deactivate_controllers_.erase(deactivate_controllers_it);
        // Decrement iterators so it will not lose track;
        --activate_controllers_it;
        --deactivate_controllers_it;
        break;
      }
    }
  }
  // TODO (komaromi): Somewhere the active_controllers have to be updated, unless it wont work.
  return std::make_pair(activate_controllers_, deactivate_controllers_);
}

std::pair<std::vector<std::string>,
  std::vector<std::string>> controller_handler::Get_active_controllers_on_deactivation()
{
  activate_controllers_ = {};
  deactivate_controllers_ = get_active_controllers();
  return std::make_pair(activate_controllers_, deactivate_controllers_);
}

std::vector<std::string> controller_handler::get_active_controllers()
{
  return std::vector<std::string>(active_controllers_.begin(), active_controllers_.end());
}

void controller_handler::ApproveControllerActivation()
{
  if (activate_controllers_.size() > 0) {
    std::copy(
      activate_controllers_.begin(), activate_controllers_.end(),
      std::inserter(active_controllers_, active_controllers_.end()));
  }
}

void controller_handler::ApproveControllerDeactivation()
{
  if (deactivate_controllers_.size() > 0) {
    for (auto && controller : deactivate_controllers_) {
      active_controllers_.erase(active_controllers_.find(controller));
    }
  }
}
}   // namespace kuka_rox
