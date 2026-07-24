// Copyright 2026 KUKA Hungaria Kft.
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

#ifndef KUKA_DRIVERS_CORE__JOINT_INTERFACE_VALIDATOR_HPP_
#define KUKA_DRIVERS_CORE__JOINT_INTERFACE_VALIDATOR_HPP_

#include <string>
#include <vector>

#include "hardware_interface/hardware_info.hpp"
#include "rclcpp/logger.hpp"

namespace kuka_drivers_core
{
namespace urdf_validator
{

bool ValidateJointCommandInterfaces(
  const hardware_interface::ComponentInfo & joint,
  const std::vector<std::string> & expected_interfaces,
  const rclcpp::Logger & logger);

bool ValidateJointStateInterfaces(
  const hardware_interface::ComponentInfo & joint,
  const std::vector<std::string> & expected_interfaces,
  const rclcpp::Logger & logger);

}  // namespace urdf_validator

}  // namespace kuka_drivers_core

#endif  // KUKA_DRIVERS_CORE__JOINT_INTERFACE_VALIDATOR_HPP_
