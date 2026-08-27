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

#ifndef KUKA_DRIVERS_CORE__CONTROLLER_HANDLER_HPP_
#define KUKA_DRIVERS_CORE__CONTROLLER_HANDLER_HPP_

#include <map>
#include <string>
#include <vector>

#include "control_mode.hpp"
#include "rclcpp/rclcpp.hpp"

namespace kuka_drivers_core
{
/**
 * @brief This class provides controller name lookup for a given control mode, making control mode
 * changes easier to handle.
 */
class ControllerHandler
{
private:
  struct ControllerTypes
  {
    std::string standard_controller;
    std::string impedance_controller;
  };

  /**
   * @brief Look up table for which controllers are needed for each control mode
   */
  std::map<ControlMode, ControllerTypes> control_mode_map_;

public:
  /**
   * @brief Construct a new control mode handler object
   */
  ControllerHandler() = default;

  /**
   * @brief Destroy the control mode handler object
   */
  ~ControllerHandler() = default;

  /**
   * @brief Updates the controllers' name for a specific controller type.
   *
   * @param controller_type: The type of the controller which will be updated.
   * @param controller_name: The new controller's name. From now on this controller will be
   * activated on controller activation.
   * @return True, if update was successful.
   * @return False, if update failed.
   */
  bool UpdateControllerName(
    const ControllerType controller_type, const std::string & controller_name);

  std::vector<std::string> GetControllersForMode(ControlMode control_mode);
};
}  // namespace kuka_drivers_core

#endif  // KUKA_DRIVERS_CORE__CONTROLLER_HANDLER_HPP_
