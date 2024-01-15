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
  std::shared_ptr<kuka_drivers_core::ROS2BaseLCNode> robot_manager_node,
  std::shared_ptr<FRIConnection> fri_connection)
: robot_manager_node_(robot_manager_node), fri_connection_(fri_connection)
{
  auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
  qos.reliable();
  cbg_ = robot_manager_node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  param_cbg_ =
    robot_manager_node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  receive_multiplier_client_ =
    robot_manager_node->create_client<kuka_driver_interfaces::srv::SetInt>(
      "fri_configuration_controller/set_receive_multiplier", qos.get_rmw_qos_profile(), cbg_);

  robot_manager_node_->registerParameter<std::string>(
    "controller_ip", "",
    kuka_drivers_core::ParameterSetAccessRights{false, false, false, false, true},
    [this](const std::string & controller_ip)
    { return this->onControllerIpChangeRequest(controller_ip); });

  set_parameter_service_ = robot_manager_node->create_service<std_srvs::srv::Trigger>(
    "configuration_manager/set_params",
    [this](
      std_srvs::srv::Trigger::Request::SharedPtr,
      std_srvs::srv::Trigger::Response::SharedPtr response) { this->setParameters(response); },
    ::rmw_qos_profile_default, param_cbg_);

  get_controllers_client_ =
    robot_manager_node->create_client<controller_manager_msgs::srv::ListControllers>(
      "controller_manager/list_controllers", qos.get_rmw_qos_profile(), cbg_);
}

bool ConfigurationManager::onCommandModeChangeRequest(const std::string & command_mode) const
{
  if (command_mode == POSITION_COMMAND)
  {
    if (!position_controller_available_ || !setCommandMode(POSITION_COMMAND))
    {
      return false;
    }
  }
  else if (command_mode == TORQUE_COMMAND)
  {
    if (robot_manager_node_->get_parameter("control_mode").as_string() != IMPEDANCE_CONTROL)
    {
      RCLCPP_ERROR(
        robot_manager_node_->get_logger(),
        "Unable to set torque command mode, if control mode is not 'joint impedance'");
      return false;
    }
    if (robot_manager_node_->get_parameter("send_period_ms").as_int() > 5)
    {
      RCLCPP_ERROR(
        robot_manager_node_->get_logger(),
        "Unable to set torque command mode, if send period is bigger than 5 [ms]");
      return false;
    }
    if (!torque_controller_available_ || !setCommandMode(TORQUE_COMMAND))
    {
      return false;
    }
  }
  else
  {
    RCLCPP_ERROR(
      robot_manager_node_->get_logger(), "Command mode should be '%s' or '%s'",
      POSITION_COMMAND.c_str(), TORQUE_COMMAND.c_str());
    return false;
  }
  RCLCPP_INFO(robot_manager_node_->get_logger(), "Successfully set command mode");
  return true;
}

bool ConfigurationManager::onControlModeChangeRequest(const std::string & control_mode) const
{
  if (control_mode == POSITION_CONTROL)
  {
    return fri_connection_->setPositionControlMode();
  }
  else if (control_mode == IMPEDANCE_CONTROL)
  {
    try
    {
      return fri_connection_->setJointImpedanceControlMode(joint_stiffness_, joint_damping_);
    }
    catch (const std::exception & e)
    {
      RCLCPP_ERROR(robot_manager_node_->get_logger(), e.what());
    }
    return false;
  }
  else
  {
    RCLCPP_ERROR(
      robot_manager_node_->get_logger(), "Control mode should be '%s' or '%s'",
      POSITION_CONTROL.c_str(), IMPEDANCE_CONTROL.c_str());
    return false;
  }
}

bool ConfigurationManager::onJointStiffnessChangeRequest(
  const std::vector<double> & joint_stiffness)
{
  if (joint_stiffness.size() != 7)
  {
    RCLCPP_ERROR(
      robot_manager_node_->get_logger(),
      "Invalid parameter array length for parameter joint stiffness");
    return false;
  }
  for (double js : joint_stiffness)
  {
    if (js < 0)
    {
      RCLCPP_ERROR(robot_manager_node_->get_logger(), "Joint stiffness values must be >=0");
      return false;
    }
  }
  joint_stiffness_ = joint_stiffness;
  return true;
}

bool ConfigurationManager::onJointDampingChangeRequest(const std::vector<double> & joint_damping)
{
  if (joint_damping.size() != 7)
  {
    RCLCPP_ERROR(
      robot_manager_node_->get_logger(),
      "Invalid parameter array length for parameter joint damping");
    return false;
  }
  for (double jd : joint_damping)
  {
    if (jd < 0 || jd > 1)
    {
      RCLCPP_ERROR(robot_manager_node_->get_logger(), "Joint damping values must be >=0 && <=1");
      return false;
    }
  }
  joint_damping_ = joint_damping;
  return true;
}

bool ConfigurationManager::onSendPeriodChangeRequest(const int & send_period) const
{
  if (send_period < 1 || send_period > 100)
  {
    RCLCPP_ERROR(
      robot_manager_node_->get_logger(), "Send period milliseconds must be >=1 && <=100");
    return false;
  }
  return true;
}

bool ConfigurationManager::onReceiveMultiplierChangeRequest(const int & receive_multiplier) const
{
  if (receive_multiplier < 1)
  {
    RCLCPP_ERROR(robot_manager_node_->get_logger(), "Receive multiplier must be >=1");
    return false;
  }
  if (!setReceiveMultiplier(receive_multiplier))
  {
    return false;
  }
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
  const std::string & controller_name, bool position)
{
  auto request = std::make_shared<controller_manager_msgs::srv::ListControllers::Request>();
  auto response =
    kuka_drivers_core::sendRequest<controller_manager_msgs::srv::ListControllers::Response>(
      get_controllers_client_, request, 0, 1000);

  if (!response)
  {
    RCLCPP_ERROR(robot_manager_node_->get_logger(), "Could not get controller names");
    return false;
  }

  if (controller_name == "")
  {
    RCLCPP_WARN(
      robot_manager_node_->get_logger(), "Controller for %s command mode not available",
      position ? POSITION_COMMAND.c_str() : TORQUE_COMMAND.c_str());
    if (position)
    {
      position_controller_available_ = false;
    }
    else
    {
      torque_controller_available_ = false;
    }
    return true;
  }

  for (const auto & controller : response->controller)
  {
    if (controller_name == controller.name)
    {
      if (position)
      {
        position_controller_available_ = true;
      }
      else
      {
        torque_controller_available_ = true;
      }
      return true;
    }
  }
  RCLCPP_ERROR(
    robot_manager_node_->get_logger(), "Controller name '%s' not available",
    controller_name.c_str());
  return false;
}

bool ConfigurationManager::setCommandMode(const std::string & command_mode) const
{
  ClientCommandModeID client_command_mode;
  if (command_mode == POSITION_COMMAND)
  {
    client_command_mode = POSITION_COMMAND_MODE;
  }
  else if (command_mode == TORQUE_COMMAND)
  {
    client_command_mode = TORQUE_COMMAND_MODE;
  }
  else
  {
    RCLCPP_ERROR(robot_manager_node_->get_logger(), "Invalid control mode");
    return false;
  }
  if (fri_connection_)
  {
    fri_connection_->setClientCommandMode(client_command_mode);
  }
  else
  {
    RCLCPP_ERROR(robot_manager_node_->get_logger(), "Robot Manager not available");
    return false;
  }
  return true;
}

bool ConfigurationManager::setReceiveMultiplier(int receive_multiplier) const
{
  // Set receive multiplier of hardware interface through controller manager service
  auto request = std::make_shared<kuka_driver_interfaces::srv::SetInt::Request>();
  request->data = receive_multiplier;
  auto response = kuka_drivers_core::sendRequest<kuka_driver_interfaces::srv::SetInt::Response>(
    receive_multiplier_client_, request, 0, 1000);

  if (!response || !response->success)
  {
    RCLCPP_ERROR(robot_manager_node_->get_logger(), "Could not set receive_multiplier");
    return false;
  }
  return true;
}

void ConfigurationManager::setParameters(std_srvs::srv::Trigger::Response::SharedPtr response)
{
  if (configured_)
  {
    RCLCPP_WARN(robot_manager_node_->get_logger(), "Parameters already registered");
    response->success = true;
    return;
  }
  // The parameter callbacks are called on this thread
  // Response is sent only after all parameters are declared (or error occurs)
  // Parameter exceptions are intentionally not caught, because in case of an invalid
  //   parameter type (or value), the nodes must be launched again with changed parameters
  //   because they could not be declared, therefore change is not possible in runtime
  robot_manager_node_->registerParameter<int>(
    "send_period_ms", 10,
    kuka_drivers_core::ParameterSetAccessRights{false, true, false, false, true},
    [this](const int & send_period) { return this->onSendPeriodChangeRequest(send_period); });

  robot_manager_node_->registerParameter<std::vector<double>>(
    "joint_stiffness", joint_stiffness_,
    kuka_drivers_core::ParameterSetAccessRights{false, true, true, false, true},
    [this](const std::vector<double> & joint_stiffness)
    { return this->onJointStiffnessChangeRequest(joint_stiffness); });

  robot_manager_node_->registerParameter<std::vector<double>>(
    "joint_damping", joint_damping_,
    kuka_drivers_core::ParameterSetAccessRights{false, true, true, false, true},
    [this](const std::vector<double> & joint_damping)
    { return this->onJointDampingChangeRequest(joint_damping); });

  robot_manager_node_->registerParameter<std::string>(
    "control_mode", POSITION_CONTROL,
    kuka_drivers_core::ParameterSetAccessRights{false, true, true, false, true},
    [this](const std::string & control_mode)
    { return this->onControlModeChangeRequest(control_mode); });

  robot_manager_node_->registerParameter<std::string>(
    "position_controller_name", "",
    kuka_drivers_core::ParameterSetAccessRights{false, true, false, false, true},
    [this](const std::string & controller_name)
    { return this->onControllerNameChangeRequest(controller_name, true); });

  robot_manager_node_->registerParameter<std::string>(
    "torque_controller_name", "",
    kuka_drivers_core::ParameterSetAccessRights{false, true, false, false, true},
    [this](const std::string & controller_name)
    { return this->onControllerNameChangeRequest(controller_name, false); });

  robot_manager_node_->registerParameter<std::string>(
    "command_mode", POSITION_COMMAND,
    kuka_drivers_core::ParameterSetAccessRights{false, true, true, false, true},
    [this](const std::string & command_mode)
    { return this->onCommandModeChangeRequest(command_mode); });

  robot_manager_node_->registerParameter<int>(
    "receive_multiplier", 1,
    kuka_drivers_core::ParameterSetAccessRights{false, true, false, false, true},
    [this](const int & receive_multiplier)
    { return this->onReceiveMultiplierChangeRequest(receive_multiplier); });

  configured_ = true;
  response->success = true;
}
}  // namespace kuka_sunrise_fri_driver
