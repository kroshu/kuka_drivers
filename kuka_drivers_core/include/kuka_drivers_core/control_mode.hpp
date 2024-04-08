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

#ifndef KUKA_DRIVERS_CORE__CONTROL_MODE_HPP_
#define KUKA_DRIVERS_CORE__CONTROL_MODE_HPP_

namespace kuka_drivers_core
{
/**
 * @brief Enum to identify every control mode
 */
enum class ControlMode : std::uint8_t
{
  CONTROL_MODE_UNSPECIFIED = 0,
  JOINT_POSITION_CONTROL = 1,
  JOINT_IMPEDANCE_CONTROL = 2,
  JOINT_VELOCITY_CONTROL = 3,
  JOINT_TORQUE_CONTROL = 4,
  CARTESIAN_POSITION_CONTROL = 5,
  CARTESIAN_IMPEDANCE_CONTROL = 6,
  CARTESIAN_VELOCITY_CONTROL = 7,
  WRENCH_CONTROL = 8,
};

/**
 * @brief Enum for identify every type of controllers
 */
enum class ControllerType : std::uint8_t
{
  JOINT_POSITION_CONTROLLER_TYPE = 0,
  CARTESIAN_POSITION_CONTROLLER_TYPE = 1,
  JOINT_IMPEDANCE_CONTROLLER_TYPE = 2,
  CARTESIAN_IMPEDANCE_CONTROLLER_TYPE = 3,
  TORQUE_CONTROLLER_TYPE = 4,
  WRENCH_CONTROLLER_TYPE = 5,
  JOINT_VELOCITY_CONTROLLER_TYPE = 6,
  TWIST_CONTROLLER_TYPE = 7
};
}  // namespace kuka_drivers_core

#endif  // KUKA_DRIVERS_CORE__CONTROL_MODE_HPP_
