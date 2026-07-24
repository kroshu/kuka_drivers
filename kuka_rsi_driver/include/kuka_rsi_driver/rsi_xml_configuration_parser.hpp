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

#ifndef KUKA_RSI_DRIVER__RSI_XML_CONFIGURATION_PARSER_HPP_
#define KUKA_RSI_DRIVER__RSI_XML_CONFIGURATION_PARSER_HPP_

#include <cstddef>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "kuka/external-control-sdk/kss/configuration.h"

namespace YAML
{
class Node;
}  // namespace YAML

namespace kuka_rsi_driver
{

class RsiXmlConfigurationParser
{
public:
  explicit RsiXmlConfigurationParser(const rclcpp::Logger & logger);

  bool Load(
    const std::string & path, std::size_t joint_count, std::size_t gpio_state_interface_count,
    kuka::external::control::kss::Configuration & config) const;

private:
  bool ParseMotionState(
    const YAML::Node & motion_state_node, const std::string & path, std::size_t joint_count,
    std::size_t gpio_state_interface_count,
    kuka::external::control::kss::Configuration & config) const;

  bool ParseControlSignal(
    const YAML::Node & control_signal_node, const std::string & path,
    kuka::external::control::kss::Configuration & config) const;

  bool ParseCartesianMotionState(
    const YAML::Node & cartesian_node,
    kuka::external::control::kss::MotionStateXmlConfiguration & motion_state_xml) const;

  bool ParseJointMotionState(
    const YAML::Node & joints_node, std::size_t joint_count,
    kuka::external::control::kss::MotionStateXmlConfiguration & motion_state_xml) const;

  bool ParseMotionStateGpio(
    const YAML::Node & gpio_node, std::size_t gpio_state_interface_count,
    kuka::external::control::kss::MotionStateXmlConfiguration & motion_state_xml) const;

  bool AppendJointFields(
    const YAML::Node & node, kuka::external::control::kss::MotionStateSignalType signal_type,
    std::vector<kuka::external::control::kss::MotionStateJointFieldConfiguration> & joint_fields)
    const;

  bool AppendOptionalJointFields(
    const YAML::Node & node, kuka::external::control::kss::MotionStateSignalType signal_type,
    const char * field_name, std::size_t joint_count,
    std::vector<kuka::external::control::kss::MotionStateJointFieldConfiguration> & joint_fields)
    const;

  void ParseXmlElement(const YAML::Node & node, std::string & xml_element) const;

  void ParseXmlAttributes(const YAML::Node & node, std::vector<std::string> & xml_attributes) const;

  void ParseControlSignalJointGroup(
    const YAML::Node & node, std::string & xml_element,
    std::vector<std::string> & xml_attributes) const;

  void ParseControlSignalOptionalGroup(
    const YAML::Node & node, bool & include_values, std::string & xml_element,
    std::vector<std::string> & xml_attributes) const;

  const rclcpp::Logger logger_;
};

}  // namespace kuka_rsi_driver

#endif  // KUKA_RSI_DRIVER__RSI_XML_CONFIGURATION_PARSER_HPP_
