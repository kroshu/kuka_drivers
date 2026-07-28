// Copyright 2020 Aron Svastits
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

#ifndef COMMUNICATION_HELPERS__ROS2_CONTROL_TOOLS_HPP_
#define COMMUNICATION_HELPERS__ROS2_CONTROL_TOOLS_HPP_

#include <memory>
#include <string>
#include <vector>

#include "communication_helpers/service_tools.hpp"
#include "controller_manager_msgs/srv/set_hardware_component_state.hpp"
#include "controller_manager_msgs/srv/switch_controller.hpp"
#include "rclcpp/rclcpp.hpp"

namespace kuka_drivers_core
{

inline bool changeHardwareState(
  rclcpp::Client<controller_manager_msgs::srv::SetHardwareComponentState>::SharedPtr client,
  const std::string & hardware_name, uint8_t state, int timeout_ms = 2000)
{
  auto hw_request =
    std::make_shared<controller_manager_msgs::srv::SetHardwareComponentState::Request>();
  hw_request->name = hardware_name;
  hw_request->target_state.id = state;
  auto hw_response = sendRequest<controller_manager_msgs::srv::SetHardwareComponentState::Response>(
    client, hw_request, 0, timeout_ms);
  if (!hw_response || !hw_response->ok)
  {
    return false;
  }
  return true;
}

inline bool changeControllerState(
  rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr client,
  const std::vector<std::string> & activate_controllers,
  const std::vector<std::string> & deactivate_controllers,
  int32_t strictness = controller_manager_msgs::srv::SwitchController::Request::STRICT)
{
  auto controller_request =
    std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
  controller_request->strictness = strictness;
  controller_request->activate_controllers = activate_controllers;
  controller_request->deactivate_controllers = deactivate_controllers;

  auto controller_response = sendRequest<controller_manager_msgs::srv::SwitchController::Response>(
    client, controller_request, 0, 2000);
  if (!controller_response || !controller_response->ok)
  {
    return false;
  }
  return true;
}

/**
 * @brief Rollback hardware interface states after a failure.
 * @param client The SetHardwareComponentState service client.
 * @param robot_models Vector of robot model names.
 * @param failed_idx Index of the robot that failed (rollback robots 0 to failed_idx-1).
 * @param target_state The state to rollback to.
 * @param logger Logger to use for error messages.
 * @param operation Description of the failed operation (for error messages).
 * @param timeout_ms Timeout for state change in milliseconds (default matches changeHardwareState).
 */
inline void rollbackHardwareStates(
  rclcpp::Client<controller_manager_msgs::srv::SetHardwareComponentState>::SharedPtr client,
  const std::vector<std::string> & robot_models, size_t failed_idx, uint8_t target_state,
  const rclcpp::Logger & logger, const std::string & operation, int timeout_ms = 2000)
{
  for (size_t rollback_idx = 0; rollback_idx < failed_idx; ++rollback_idx)
  {
    const auto & rollback_model = robot_models[rollback_idx];
    if (!changeHardwareState(client, rollback_model, target_state, timeout_ms))
    {
      RCLCPP_ERROR(
        logger, "Could not roll back hardware interface '%s' after %s failure",
        rollback_model.c_str(), operation.c_str());
    }
  }
}
}  // namespace kuka_drivers_core

#endif  // COMMUNICATION_HELPERS__ROS2_CONTROL_TOOLS_HPP_
