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

#ifndef KUKA_ROX_HW_INTERFACE__CONTROLLER_HANDLER_HPP_
#define KUKA_ROX_HW_INTERFACE__CONTROLLER_HANDLER_HPP_

#include <string>
#include <vector>

#include "kuka/ecs/v1/motion_services_ecs.grpc.pb.h"
#include "rclcpp/rclcpp.hpp"

namespace kuka_rox
{
/**
 * @brief This class is responsible for tracking the active controllers
 *  and on control mode change offer the controllers name that
 *  need to be activated and deactivated.
 */
class controller_handler
{
private:
  /**
   * @brief Controller names thats have to active at any control mode
   */
  std::vector<std::string> fixed_controllers_;
  /**
   * @brief The currently active controllers that not include the fixed controllers
   */
  std::vector<std::string> active_controllers_;

  /**
   * @brief Look up table for wich controllers needed for each control mode
   */
  std::map<kuka::motion::external::ExternalControlMode, std::vector<std::string>> control_mode_map_;

  /**
   * @brief Size of every controller for each control_mode
   */
  static constexpr int POSITION_CONTROLLERS_SIZE = 1;
  static constexpr int IMPEDANCE_CONTROLLERS_SIZE = 2;
  static constexpr int TORQUE_CONTROLLERS_SIZE = 1;

  /**
   * @brief Controllers possition in the controllers vector
   */
  static constexpr int NORMAL_CONROLLERS_POS = 0;
  static constexpr int IMPEDANCE_CONTROLLERS_POS = 1;

public:
  /**
   * @brief Enum for identify every type of controllers
   */
  typedef enum   //controller_type
  {
    POSITION_CONTROLLER = 0,
    IMPEDANCE_CONTROLLER = 1,
    TORQUE_CONTROLLER = 2,
  } controller_type;

  /**
   * @brief Construct a new control mode handler object
   *
   * @param fixed_controllers: Controller names thats have to active at any control mode
   */
  controller_handler(std::vector<std::string> fixed_controllers);

  /**
   * @brief Destroy the control mode handler object
   */
  ~controller_handler() = default;

  /**
   * @brief Updates the controllers name for a specific controller type.
   *
   * @param controller_type: The type of the cotroller wich will be updated.
   * @param controller_name: The new controllers name. From now on this controller will be activated on controller acivation.
   * @return true, if update was succesfull.
   * @return false, if update failed.
   */
  bool Update_controller_name(
    const controller_type controller_type,
    const std::string & controller_name);

  /**
   * @brief Calculates the controllers that have to be activated and deactivated for the control mode change
   *
   * @param new_control_mode: The new control mode
   * @return std::pair<std::vector<std::string>, std::vector<std::string>>:
   * Two vectors, first has the contrllers to activate, second has the controllers to deactivate
   */
  std::pair<std::vector<std::string>, std::vector<std::string>> Get_new_controllers(
    kuka::motion::external::ExternalControlMode new_control_mode);

  /**
   * @brief Retruns the currently active controllers for the user.
   *
   * @return std::vector<std::string>: The currently active controllers
   */
  std::vector<std::string> Get_active_controllers();
};
} // namespace kuka_rox


#endif  // KUKA_ROX_HW_INTERFACE__CONTROLLER_HANDLER_HPP_
