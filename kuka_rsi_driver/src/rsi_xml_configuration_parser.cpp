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

#include "kuka_rsi_driver/rsi_xml_configuration_parser.hpp"

#include <utility>

#include <yaml-cpp/yaml.h>

namespace kuka_rsi_driver
{

RsiXmlConfigurationParser::RsiXmlConfigurationParser(const rclcpp::Logger & logger)
: logger_(logger)
{
}

bool RsiXmlConfigurationParser::Load(
  const std::string & path, std::size_t joint_count, std::size_t gpio_state_interface_count,
  kuka::external::control::kss::Configuration & config) const
{
  YAML::Node root;
  try
  {
    root = YAML::LoadFile(path);
  }
  catch (const YAML::Exception & e)
  {
    RCLCPP_ERROR(logger_, "Failed to load RSI XML config file '%s': %s", path.c_str(), e.what());
    return false;
  }

  YAML::Node rsi_node;
  try
  {
    rsi_node = root["rsi_xml_config"];
  }
  catch (const YAML::Exception & e)
  {
    RCLCPP_ERROR(
      logger_, "RSI XML config file '%s' has an invalid top-level structure: %s", path.c_str(),
      e.what());
    return false;
  }

  if (!rsi_node)
  {
    RCLCPP_ERROR(
      logger_, "RSI XML config file '%s' does not contain 'rsi_xml_config' key", path.c_str());
    return false;
  }

  try
  {
    if (!ParseMotionState(
          rsi_node["motion_state"], path, joint_count, gpio_state_interface_count, config))
    {
      return false;
    }

    if (!ParseControlSignal(rsi_node["control_signal"], path, config))
    {
      return false;
    }
  }
  catch (const YAML::Exception & e)
  {
    RCLCPP_ERROR(logger_, "Invalid RSI XML config file '%s': %s", path.c_str(), e.what());
    return false;
  }

  return true;
}

bool RsiXmlConfigurationParser::ParseMotionState(
  const YAML::Node & motion_state_node, const std::string & path, std::size_t joint_count,
  std::size_t gpio_state_interface_count,
  kuka::external::control::kss::Configuration & config) const
{
  using namespace kuka::external::control::kss;  // NOLINT

  if (!motion_state_node)
  {
    return true;
  }

  MotionStateXmlConfiguration motion_state_xml;

  if (!ParseCartesianMotionState(motion_state_node["cartesian"], motion_state_xml))
  {
    return false;
  }

  if (!ParseJointMotionState(motion_state_node["joints"], joint_count, motion_state_xml))
  {
    return false;
  }

  if (!ParseMotionStateGpio(
        motion_state_node["gpio"], gpio_state_interface_count, motion_state_xml))
  {
    return false;
  }

  RCLCPP_INFO(logger_, "Custom motion state XML configuration loaded from '%s'", path.c_str());
  config.motion_state_xml_config = std::move(motion_state_xml);
  return true;
}

bool RsiXmlConfigurationParser::ParseControlSignal(
  const YAML::Node & control_signal_node, const std::string & path,
  kuka::external::control::kss::Configuration & config) const
{
  using namespace kuka::external::control::kss;  // NOLINT

  if (!control_signal_node)
  {
    return true;
  }

  ControlSignalXmlConfiguration control_signal_xml;

  ParseControlSignalJointGroup(
    control_signal_node["joints"], control_signal_xml.joint_xml_element,
    control_signal_xml.joint_xml_attributes);
  ParseControlSignalJointGroup(
    control_signal_node["ext_joints"], control_signal_xml.ext_joint_xml_element,
    control_signal_xml.ext_joint_xml_attributes);

  ParseControlSignalOptionalGroup(
    control_signal_node["velocities"], control_signal_xml.include_velocity_values,
    control_signal_xml.velocity_xml_element, control_signal_xml.velocity_xml_attributes);
  ParseControlSignalOptionalGroup(
    control_signal_node["ext_velocities"], control_signal_xml.include_ext_velocity_values,
    control_signal_xml.ext_velocity_xml_element, control_signal_xml.ext_velocity_xml_attributes);
  ParseControlSignalOptionalGroup(
    control_signal_node["torques"], control_signal_xml.include_torque_values,
    control_signal_xml.torque_xml_element, control_signal_xml.torque_xml_attributes);
  ParseControlSignalOptionalGroup(
    control_signal_node["ext_torques"], control_signal_xml.include_ext_torque_values,
    control_signal_xml.ext_torque_xml_element, control_signal_xml.ext_torque_xml_attributes);

  if (const YAML::Node gpio = control_signal_node["gpio"])
  {
    ParseXmlElement(gpio, control_signal_xml.gpio_xml_element);
  }

  RCLCPP_INFO(logger_, "Custom control signal XML configuration loaded from '%s'", path.c_str());
  config.control_signal_xml_config = std::move(control_signal_xml);
  return true;
}

bool RsiXmlConfigurationParser::ParseCartesianMotionState(
  const YAML::Node & cartesian_node,
  kuka::external::control::kss::MotionStateXmlConfiguration & motion_state_xml) const
{
  if (!cartesian_node)
  {
    return true;
  }

  if (const YAML::Node enabled = cartesian_node["enabled"])
  {
    motion_state_xml.cartesian.enabled = enabled.as<bool>();
  }

  ParseXmlElement(cartesian_node, motion_state_xml.cartesian.xml_element);

  if (const YAML::Node attrs = cartesian_node["xml_attributes"])
  {
    if (attrs.size() != motion_state_xml.cartesian.xml_attributes.size())
    {
      RCLCPP_ERROR(
        logger_, "motion_state.cartesian.xml_attributes has %zu entries; expected %zu.",
        attrs.size(), motion_state_xml.cartesian.xml_attributes.size());
      return false;
    }

    for (std::size_t i = 0; i < attrs.size(); ++i)
    {
      motion_state_xml.cartesian.xml_attributes[i] = attrs[i].as<std::string>();
    }
  }

  return true;
}

bool RsiXmlConfigurationParser::ParseJointMotionState(
  const YAML::Node & joints_node, std::size_t joint_count,
  kuka::external::control::kss::MotionStateXmlConfiguration & motion_state_xml) const
{
  using namespace kuka::external::control::kss;  // NOLINT

  if (!joints_node)
  {
    RCLCPP_ERROR(
      logger_, "motion_state.joints.positions is required and must contain one entry per joint.");
    return false;
  }

  const YAML::Node positions = joints_node["positions"];
  if (!positions)
  {
    RCLCPP_ERROR(
      logger_, "motion_state.joints.positions is required and must contain one entry per joint.");
    return false;
  }

  if (positions.size() != joint_count)
  {
    RCLCPP_ERROR(
      logger_, "motion_state.joints.positions has %zu entries but URDF defines %zu joints.",
      positions.size(), joint_count);
    return false;
  }

  if (!AppendJointFields(positions, MotionStateSignalType::POSITION, motion_state_xml.joint_fields))
  {
    return false;
  }

  if (!AppendOptionalJointFields(
        joints_node["velocities"], MotionStateSignalType::VELOCITY, "joints.velocities",
        joint_count, motion_state_xml.joint_fields))
  {
    return false;
  }

  if (!AppendOptionalJointFields(
        joints_node["torques"], MotionStateSignalType::TORQUE, "joints.torques", joint_count,
        motion_state_xml.joint_fields))
  {
    return false;
  }

  return true;
}

bool RsiXmlConfigurationParser::ParseMotionStateGpio(
  const YAML::Node & gpio_node, std::size_t gpio_state_interface_count,
  kuka::external::control::kss::MotionStateXmlConfiguration & motion_state_xml) const
{
  if (!gpio_node)
  {
    return true;
  }

  ParseXmlElement(gpio_node, motion_state_xml.gpio_xml_element);

  if (const YAML::Node attrs = gpio_node["xml_attributes"])
  {
    for (const YAML::Node & attr : attrs)
    {
      motion_state_xml.gpio_xml_attributes.push_back(attr.as<std::string>());
    }
    if (motion_state_xml.gpio_xml_attributes.size() != gpio_state_interface_count)
    {
      RCLCPP_ERROR(
        logger_,
        "motion_state.gpio.xml_attributes has %zu entries but %zu GPIO state interfaces are "
        "defined. They must match.",
        motion_state_xml.gpio_xml_attributes.size(), gpio_state_interface_count);
      return false;
    }
  }

  return true;
}

bool RsiXmlConfigurationParser::AppendJointFields(
  const YAML::Node & node, kuka::external::control::kss::MotionStateSignalType signal_type,
  std::vector<kuka::external::control::kss::MotionStateJointFieldConfiguration> & joint_fields)
  const
{
  for (const YAML::Node & joint_node : node)
  {
    kuka::external::control::kss::MotionStateJointFieldConfiguration joint_field;
    joint_field.joint_identifier = joint_node["joint_identifier"].as<std::string>();
    joint_field.signal_type = signal_type;
    joint_field.xml_element = joint_node["xml_element"].as<std::string>();
    joint_field.xml_attribute = joint_node["xml_attribute"].as<std::string>();
    joint_fields.push_back(std::move(joint_field));
  }
  return true;
}

bool RsiXmlConfigurationParser::AppendOptionalJointFields(
  const YAML::Node & node, kuka::external::control::kss::MotionStateSignalType signal_type,
  const char * field_name, std::size_t joint_count,
  std::vector<kuka::external::control::kss::MotionStateJointFieldConfiguration> & joint_fields)
  const
{
  if (!node)
  {
    return true;
  }

  if (node.size() != joint_count)
  {
    RCLCPP_ERROR(
      logger_, "motion_state.%s has %zu entries but URDF defines %zu joints.", field_name,
      node.size(), joint_count);
    return false;
  }

  return AppendJointFields(node, signal_type, joint_fields);
}

void RsiXmlConfigurationParser::ParseXmlElement(
  const YAML::Node & node, std::string & xml_element) const
{
  if (const YAML::Node elem = node["xml_element"])
  {
    xml_element = elem.as<std::string>();
  }
}

void RsiXmlConfigurationParser::ParseXmlAttributes(
  const YAML::Node & node, std::vector<std::string> & xml_attributes) const
{
  if (const YAML::Node attrs = node["xml_attributes"])
  {
    for (const YAML::Node & attr : attrs)
    {
      xml_attributes.push_back(attr.as<std::string>());
    }
  }
}

void RsiXmlConfigurationParser::ParseControlSignalJointGroup(
  const YAML::Node & node, std::string & xml_element,
  std::vector<std::string> & xml_attributes) const
{
  if (!node)
  {
    return;
  }

  ParseXmlElement(node, xml_element);
  ParseXmlAttributes(node, xml_attributes);
}

void RsiXmlConfigurationParser::ParseControlSignalOptionalGroup(
  const YAML::Node & node, bool & include_values, std::string & xml_element,
  std::vector<std::string> & xml_attributes) const
{
  if (!node)
  {
    return;
  }

  include_values = true;

  if (const YAML::Node enabled = node["enabled"])
  {
    include_values = enabled.as<bool>();
  }

  ParseXmlElement(node, xml_element);
  ParseXmlAttributes(node, xml_attributes);
}

}  // namespace kuka_rsi_driver
