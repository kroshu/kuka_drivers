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

#include "kuka_drivers_core/joint_interface_validator.hpp"

#include <algorithm>
#include <unordered_set>

#include "rclcpp/rclcpp.hpp"

namespace kuka_drivers_core
{
namespace urdf_validator
{
namespace
{

bool ValidateInterfaces(
  const hardware_interface::ComponentInfo & joint,
  const std::vector<hardware_interface::InterfaceInfo> & interfaces_to_check,
  const std::vector<std::string> & expected_interfaces, const char * interface_type,
  const rclcpp::Logger & logger)
{
  if (interfaces_to_check.size() != expected_interfaces.size())
  {
    RCLCPP_FATAL(
      logger, "expecting exactly %zu %s interface", expected_interfaces.size(), interface_type);
    return false;
  }

  std::unordered_set<std::string> seen_interfaces;
  seen_interfaces.reserve(interfaces_to_check.size());

  for (const auto & interface_info : interfaces_to_check)
  {
    const auto & interface_name = interface_info.name;

    const auto is_expected =
      std::find(expected_interfaces.begin(), expected_interfaces.end(), interface_name) !=
      expected_interfaces.end();
    if (!is_expected)
    {
      RCLCPP_FATAL(
        logger, "Unsupported %s interface '%s' for joint %s", interface_type,
        interface_name.c_str(), joint.name.c_str());
      return false;
    }

    if (!seen_interfaces.insert(interface_name).second)
    {
      RCLCPP_FATAL(
        logger, "Duplicate %s %s interface for joint %s", interface_name.c_str(), interface_type,
        joint.name.c_str());
      return false;
    }
  }

  for (const auto & expected_interface : expected_interfaces)
  {
    if (seen_interfaces.find(expected_interface) == seen_interfaces.end())
    {
      RCLCPP_FATAL(
        logger, "%s %s interface is required for joint %s", expected_interface.c_str(),
        interface_type, joint.name.c_str());
      return false;
    }
  }

  return true;
}

}  // namespace

bool ValidateJointCommandInterfaces(
  const hardware_interface::ComponentInfo & joint,
  const std::vector<std::string> & expected_interfaces, const rclcpp::Logger & logger)
{
  return ValidateInterfaces(
    joint, joint.command_interfaces, expected_interfaces, "command", logger);
}

bool ValidateJointStateInterfaces(
  const hardware_interface::ComponentInfo & joint,
  const std::vector<std::string> & expected_interfaces, const rclcpp::Logger & logger)
{
  return ValidateInterfaces(joint, joint.state_interfaces, expected_interfaces, "state", logger);
}

}  // namespace urdf_validator
}  // namespace kuka_drivers_core
