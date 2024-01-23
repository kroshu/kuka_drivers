// Copyright 2020 Zoltán Rési
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

#include <memory>
#include <string>
#include <vector>

#include "communication_helpers/service_tools.hpp"
#include "kuka_sunrise_fri_driver/configuration_manager.hpp"

namespace kuka_sunrise_fri_driver
{
ConfigurationManager::ConfigurationManager(
  std::shared_ptr<kuka_drivers_core::ROS2BaseLCNode> robot_manager_node)
: robot_manager_node_(robot_manager_node)
{
  auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
  qos.reliable();
  cbg_ = robot_manager_node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  fri_config_client_ =
    robot_manager_node->create_client<kuka_driver_interfaces::srv::SetFriConfiguration>(
      "fri_configuration_controller/set_fri_config", qos.get_rmw_qos_profile(), cbg_);

  control_mode_pub_ = robot_manager_node->create_publisher<std_msgs::msg::UInt32>(
    "control_mode_handler/control_mode", rclcpp::SystemDefaultsQoS());

  robot_manager_node_->registerStaticParameter<std::string>(
    "robot_model", "lbr_iiwa14_r820",
    kuka_drivers_core::ParameterSetAccessRights{true, false, false, false},
    [this](const std::string & robot_model)
    { return this->onRobotModelChangeRequest(robot_model); });

  robot_manager_node_->registerStaticParameter<std::string>(
    "controller_ip", "", kuka_drivers_core::ParameterSetAccessRights{true, false, false, false},
    [this](const std::string & controller_ip)
    { return this->onControllerIpChangeRequest(controller_ip); });

  robot_manager_node_->registerParameter<int>(
    "send_period_ms", 10,
    kuka_drivers_core::ParameterSetAccessRights{false, true, false, false, true},
    [this](const int & send_period) { return this->onSendPeriodChangeRequest(send_period); });

  robot_manager_node_->registerParameter<int>(
    "control_mode", static_cast<int>(kuka_drivers_core::ControlMode::JOINT_POSITION_CONTROL),
    kuka_drivers_core::ParameterSetAccessRights{true, true, true, false},
    [this](int control_mode) { return this->onControlModeChangeRequest(control_mode); });

  robot_manager_node_->registerParameter<std::string>(
    "position_controller_name", "",
    kuka_drivers_core::ParameterSetAccessRights{false, true, false, false, true},
    [this](const std::string & controller_name)
    {
      return this->onControllerNameChangeRequest(
        controller_name, kuka_drivers_core::ControllerType::JOINT_POSITION_CONTROLLER_TYPE);
    });

  robot_manager_node_->registerParameter<std::string>(
    "torque_controller_name", "",
    kuka_drivers_core::ParameterSetAccessRights{false, true, false, false, true},
    [this](const std::string & controller_name)
    {
      return this->onControllerNameChangeRequest(
        controller_name, kuka_drivers_core::ControllerType::TORQUE_CONTROLLER_TYPE);
    });

  robot_manager_node_->registerParameter<int>(
    "receive_multiplier", 1,
    kuka_drivers_core::ParameterSetAccessRights{false, true, false, false, true},
    [this](const int & receive_multiplier)
    { return this->onReceiveMultiplierChangeRequest(receive_multiplier); });

  // TODO(Svastits): consider readding stiffness and damping as params, that update the joint imp
  // controller
}

bool ConfigurationManager::onRobotModelChangeRequest(const std::string & robot_model)
{
  auto ns = std::string(robot_manager_node_->get_namespace());
  // Remove '/' from namespace (even empty namespace contains one '/')
  ns.erase(ns.begin());

  // Add '_' to prefix
  if (ns.size() > 0)
  {
    ns += "_";
  }
  robot_model_ = ns + robot_model;
  return true;
}

bool ConfigurationManager::onControlModeChangeRequest(int control_mode)
{
  switch (static_cast<kuka_drivers_core::ControlMode>(control_mode))
  {
    case kuka_drivers_core::ControlMode::JOINT_POSITION_CONTROL:
      break;
    case kuka_drivers_core::ControlMode::JOINT_IMPEDANCE_CONTROL:
      // TODO(Svastits): check whether this is necessary for impedance mode too
      [[fallthrough]];
    case kuka_drivers_core::ControlMode::JOINT_TORQUE_CONTROL:
      if (send_period_ms_ > 5)
      {
        RCLCPP_ERROR(
          robot_manager_node_->get_logger(),
          "Unable to set non-position control mode, if send period is bigger than 5 [ms]");
        return false;
      }
      break;
    default:
      RCLCPP_ERROR(
        robot_manager_node_->get_logger(), "Tried to change to a not implemented control mode");
      return false;
  }

  // Publish the control mode to controller handler
  control_mode_msg_.data = control_mode;
  control_mode_pub_->publish(control_mode_msg_);
  RCLCPP_INFO(robot_manager_node_->get_logger(), "Control mode change successful");

  return true;
}

bool ConfigurationManager::onSendPeriodChangeRequest(int send_period)
{
  if (send_period < 1 || send_period > 100)
  {
    RCLCPP_ERROR(
      robot_manager_node_->get_logger(), "Send period milliseconds must be >=1 && <=100");
    return false;
  }
  // Set send period of hardware interface through controller manager service
  if (!setFriConfiguration(send_period, receive_multiplier_))
  {
    return false;
  }

  send_period_ms_ = send_period;
  return true;
}

bool ConfigurationManager::onReceiveMultiplierChangeRequest(const int & receive_multiplier)
{
  if (receive_multiplier < 1)
  {
    RCLCPP_ERROR(robot_manager_node_->get_logger(), "Receive multiplier must be >=1");
    return false;
  }

  // Set receive multiplier of hardware interface through controller manager service
  if (!setFriConfiguration(send_period_ms_, receive_multiplier))
  {
    return false;
  }

  receive_multiplier_ = receive_multiplier;
  return true;
}

bool ConfigurationManager::onControllerIpChangeRequest(const std::string & controller_ip) const
{
  // Check IP validity
  size_t i = 0;
  std::vector<std::string> split_ip;
  auto pos = controller_ip.find('.');
  while (pos != std::string::npos)
  {
    split_ip.push_back(controller_ip.substr(i, pos - i));
    i = ++pos;
    pos = controller_ip.find('.', pos);
  }
  split_ip.push_back(controller_ip.substr(i, controller_ip.length()));

  if (split_ip.size() != 4)
  {
    RCLCPP_ERROR(robot_manager_node_->get_logger(), "Valid IP must have 3 '.' delimiters");
    return false;
  }

  for (const auto & ip : split_ip)
  {
    if (
      ip.empty() || (ip.find_first_not_of("[0123456789]") != std::string::npos) || stoi(ip) > 255 ||
      stoi(ip) < 0)
    {
      RCLCPP_ERROR(
        robot_manager_node_->get_logger(), "Valid IP must contain only numbers between 0 and 255");
      return false;
    }
  }
  return true;
}

bool ConfigurationManager::onControllerNameChangeRequest(
  const std::string & controller_name, kuka_drivers_core::ControllerType controller_type)
{
  switch (controller_type)
  {
    case kuka_drivers_core::ControllerType::JOINT_POSITION_CONTROLLER_TYPE:
      joint_pos_controller_name_ = controller_name;
      break;
    case kuka_drivers_core::ControllerType::TORQUE_CONTROLLER_TYPE:
      joint_torque_controller_name_ = controller_name;
      break;
    default:
      // This should actually never happen
      RCLCPP_ERROR(robot_manager_node_->get_logger(), "Invalid controller type");
      return false;
  }
  return true;
}

// the joint impedannce attributes cannot be modified in FRI after activation, therefore only one
// controller controls in each control mode
std::string ConfigurationManager::GetControllerName()
{
  switch (static_cast<kuka_drivers_core::ControlMode>(control_mode_msg_.data))
  {
    case kuka_drivers_core::ControlMode::JOINT_POSITION_CONTROL:
      return joint_pos_controller_name_;
    case kuka_drivers_core::ControlMode::JOINT_IMPEDANCE_CONTROL:
      return joint_pos_controller_name_;
    case kuka_drivers_core::ControlMode::JOINT_TORQUE_CONTROL:
      return joint_torque_controller_name_;
    default:
      throw std::runtime_error("Stored control mode is not allowed");
  }
}

bool ConfigurationManager::setFriConfiguration(int send_period_ms, int receive_multiplier)
{
  auto request = std::make_shared<kuka_driver_interfaces::srv::SetFriConfiguration::Request>();
  request->receive_multiplier = receive_multiplier;
  request->send_period_ms = send_period_ms;
  auto response =
    kuka_drivers_core::sendRequest<kuka_driver_interfaces::srv::SetFriConfiguration::Response>(
      fri_config_client_, request, 0, 1000);

  if (!response || !response->success)
  {
    RCLCPP_ERROR(
      robot_manager_node_->get_logger(), "Could not set receive_multiplierFRI configuration");
    return false;
  }
  return true;
}
}  // namespace kuka_sunrise_fri_driver
