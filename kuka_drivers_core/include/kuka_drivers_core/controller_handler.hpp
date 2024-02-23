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
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "control_mode.hpp"
#include "rclcpp/rclcpp.hpp"

namespace kuka_drivers_core
{
/**
 * @brief This class is responsible for tracking the active controllers
 *  and on control mode change offer the controllers name that
 *  need to be activated and deactivated.
 * The following three public function has to be called in the following order:
 *  - GetControllersForSwitch() or GetControllersForDeactivation()
 *  - ApproveControllerActivation()
 *  - ApproveControllerDeactivation()
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
   * @brief Controller names that's have to be active in all control modes
   */
  std::set<std::string> fixed_controllers_;

  /**
   * @brief The currently active controllers that not include the fixed controllers
   * The controllers are stored in an std::set type
   */
  std::set<std::string> active_controllers_;

  /**
   * @brief These controllers will be activated after they get approved
   */
  std::set<std::string> activate_controllers_;

  /**
   * @brief These controllers will be deactivated after they get approved
   */
  std::set<std::string> deactivate_controllers_;

  /**
   * @brief Look up table for which controllers are needed for each control mode
   */
  std::map<ControlMode, ControllerTypes> control_mode_map_;

public:
  /**
   * @brief Construct a new control mode handler object
   *
   * @param fixed_controllers: Controllers that have to be active in all control modes
   */
  explicit ControllerHandler(std::vector<std::string> fixed_controllers = {});

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

  /**
   * @brief Calculates the controllers that have to be activated and deactivated for the control
   * mode change
   *
   * @param new_control_mode: The new control mode. It is based on Controller_handler::control_mode
   * enum.
   * @return std::pair<std::vector<std::string>, std::vector<std::string>>:
   * Two vectors, first has the controllers to activate, second has the controllers to deactivate
   * @exception std::out_of_range: new_control_mode attribute is invalid
   */
  std::pair<std::vector<std::string>, std::vector<std::string>> GetControllersForSwitch(
    ControlMode new_control_mode);

  /**
   * @brief Returns all controllers that has active state (used for driver deactivation)
   *
   * @return std::vector<std::string>: Vector that contains controllers for deactivation
   */
  std::vector<std::string> GetControllersForDeactivation();

  /**
   * @brief Approves that the controller activation was successful
   *
   */
  void ApproveControllerActivation();

  /**
   * @brief Approves that the controller deactivation was successful
   *
   */
  bool ApproveControllerDeactivation();

  std::vector<std::string> GetControllersForMode(ControlMode control_mode);
};
}  // namespace kuka_drivers_core

#endif  // KUKA_DRIVERS_CORE__CONTROLLER_HANDLER_HPP_
