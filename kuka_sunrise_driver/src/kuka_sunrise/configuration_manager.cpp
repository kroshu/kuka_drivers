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

#include "kuka_sunrise/configuration_manager.hpp"
#include "kuka_sunrise/internal/service_tools.hpp"
#include "kuka_sunrise/robot_manager.hpp"

namespace kuka_sunrise
{

ConfigurationManager::ConfigurationManager(
  rclcpp_lifecycle::LifecycleNode::SharedPtr robot_manager_node,
  std::shared_ptr<RobotManager> robot_manager)
: robot_manager_node_(robot_manager_node), robot_manager_(robot_manager),
  joint_stiffness_temp_(std::vector<double>(7, 1000.0)),
  joint_damping_temp_(std::vector<double>(7, 0.7))
{
  parameter_set_access_rights_.emplace(
    "command_mode", ParameterSetAccessRights {false, true, true,
      false, true});
  parameter_set_access_rights_.emplace(
    "control_mode", ParameterSetAccessRights {false, true, true,
      false, true});
  parameter_set_access_rights_.emplace(
    "joint_stiffness", ParameterSetAccessRights {false, true,
      true, false, true});
  parameter_set_access_rights_.emplace(
    "joint_damping", ParameterSetAccessRights {false, true, true,
      false, true});
  parameter_set_access_rights_.emplace(
    "send_period_ms", ParameterSetAccessRights {false, true,
      false, false, true});
  parameter_set_access_rights_.emplace(
    "receive_multiplier", ParameterSetAccessRights {false, true,
      false, false, true});
  parameter_set_access_rights_.emplace(
    "controller_ip", ParameterSetAccessRights {false, false,
      false, false, true});

  auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
  qos.reliable();
  cbg_ = robot_manager_node->create_callback_group(
    rclcpp::callback_group::CallbackGroupType::MutuallyExclusive);
  command_mode_client_ = robot_manager_node->create_client<std_srvs::srv::SetBool>(
    "set_command_mode", qos.get_rmw_qos_profile(), cbg_);
  receive_multiplier_client_ = robot_manager_node->create_client<
    kuka_sunrise_interfaces::srv::SetInt>(
    "set_receive_multiplier", qos.get_rmw_qos_profile(),
    cbg_);
  sync_receive_multiplier_client_ = robot_manager_node->create_client<
    kuka_sunrise_interfaces::srv::SetInt>(
    "sync_receive_multiplier", qos.get_rmw_qos_profile(),
    cbg_);
  sync_send_period_client_ = robot_manager_node->create_client<
    kuka_sunrise_interfaces::srv::SetInt>(
    "sync_send_period", qos.get_rmw_qos_profile(),
    cbg_);

  if (!robot_manager_node_->has_parameter("control_mode")) {
    robot_manager_node_->declare_parameter("control_mode", rclcpp::ParameterValue("position"));
  }

  if (!robot_manager_node_->has_parameter("command_mode")) {
    robot_manager_node_->declare_parameter("command_mode", rclcpp::ParameterValue("position"));
  }

  param_callback_ = robot_manager_node_->add_on_set_parameters_callback(
    [this](
      const std::vector<rclcpp::Parameter> & parameters)
    {return this->onParamChange(parameters);});

  if (!robot_manager_node_->has_parameter("receive_multiplier")) {
    robot_manager_node_->declare_parameter("receive_multiplier", rclcpp::ParameterValue(1));
  }

  if (!robot_manager_node_->has_parameter("send_period_ms")) {
    robot_manager_node_->declare_parameter("send_period_ms", rclcpp::ParameterValue(10));
  }

  if (!robot_manager_node_->has_parameter("joint_stiffness")) {
    robot_manager_node_->declare_parameter(
      "joint_stiffness", rclcpp::ParameterValue(joint_stiffness_temp_));
  }

  if (!robot_manager_node_->has_parameter("joint_damping")) {
    robot_manager_node_->declare_parameter(
      "joint_damping",
      rclcpp::ParameterValue(joint_damping_temp_));
  }

  if (!robot_manager_node_->has_parameter("controller_ip")) {
    robot_manager_node_->declare_parameter("controller_ip");
  }

  auto set_param_callback = [this](
    std_srvs::srv::Trigger::Request::SharedPtr,
    std_srvs::srv::Trigger::Response::SharedPtr response) {
      response->success = true;
      if (!onControlModeChangeRequest(
          robot_manager_node_->get_parameter("control_mode")))
      {
        RCLCPP_ERROR(
          robot_manager_node_->get_logger(),
          "Could not set parameter control_mode");
        response->success = false;
      }

      if (!onCommandModeChangeRequest(
          robot_manager_node_->get_parameter("command_mode")))
      {
        RCLCPP_ERROR(
          robot_manager_node_->get_logger(),
          "Could not set parameter command_mode");
        response->success = false;
      }
    };

  set_parameter_service_ = robot_manager_node->create_service<std_srvs::srv::Trigger>(
    "configuration_manager/set_params", set_param_callback, ::rmw_qos_profile_default, cbg_);
}

rcl_interfaces::msg::SetParametersResult ConfigurationManager::onParamChange(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  for (const rclcpp::Parameter & param : parameters) {
    if (param.get_name() == "command_mode") {
      result.successful = onCommandModeChangeRequest(param);
    } else if (param.get_name() == "control_mode") {
      result.successful = onControlModeChangeRequest(param);
    } else if (param.get_name() == "joint_stiffness") {
      result.successful = onJointStiffnessChangeRequest(param);
    } else if (param.get_name() == "joint_damping") {
      result.successful = onJointDampingChangeRequest(param);
    } else if (param.get_name() == "send_period_ms") {
      result.successful = onSendPeriodChangeRequest(param);
    } else if (param.get_name() == "receive_multiplier") {
      result.successful = onReceiveMultiplierChangeRequest(param);
    } else if (param.get_name() == "controller_ip") {
      result.successful = onControllerIpChangeRequest(param);
    } else {
      RCLCPP_ERROR(
        robot_manager_node_->get_logger(), "Invalid parameter name %s",
        param.get_name().c_str());
      result.successful = false;
    }
  }
  return result;
}

bool ConfigurationManager::canSetParameter(const rclcpp::Parameter & param)
{
  try {
    if (!parameter_set_access_rights_.at(param.get_name()).isSetAllowed(
        robot_manager_node_->get_current_state().id()))
    {
      RCLCPP_ERROR(
        robot_manager_node_->get_logger(),
        "Parameter %s cannot be changed while in state %s", param.get_name().c_str(),
        robot_manager_node_->get_current_state().label().c_str());
      return false;
    }
  } catch (const std::out_of_range & e) {
    RCLCPP_ERROR(
      robot_manager_node_->get_logger(),
      "Parameter set access rights for parameter %s couldn't be determined",
      param.get_name().c_str());
    return false;
  }
  return true;
}

bool ConfigurationManager::onCommandModeChangeRequest(const rclcpp::Parameter & param)
{
  if (param.get_type() != rcl_interfaces::msg::ParameterType::PARAMETER_STRING) {
    RCLCPP_ERROR(
      robot_manager_node_->get_logger(), "Invalid parameter type for parameter %s",
      param.get_name().c_str());
    return false;
  }

  if (!canSetParameter(param)) {
    return false;
  }
  if (param.as_string() == "position") {
    if (!setCommandMode("position")) {
      return false;
    }
  } else if (param.as_string() == "torque") {
    if (robot_manager_node_->get_parameter("control_mode").as_string() != "joint_impedance") {
      RCLCPP_ERROR(
        robot_manager_node_->get_logger(),
        "Unable to set torque command mode, if control mode is not 'joint impedance'");
      return false;
    }
    if (robot_manager_node_->get_parameter("send_period_ms").as_int() > 5) {
      RCLCPP_ERROR(
        robot_manager_node_->get_logger(),
        "Unable to set torque command mode, if send periods is bigger than 5");
      return false;
    }
    if (!setCommandMode("torque")) {
      return false;
    }
  } else {
    RCLCPP_ERROR(
      robot_manager_node_->get_logger(), "Invalid parameter value for parameter %s",
      param.get_name().c_str());
    return false;
  }
  return true;
}

bool ConfigurationManager::onControlModeChangeRequest(const rclcpp::Parameter & param)
{
  if (param.get_type() != rcl_interfaces::msg::ParameterType::PARAMETER_STRING) {
    RCLCPP_ERROR(
      robot_manager_node_->get_logger(), "Invalid parameter type for parameter %s",
      param.get_name().c_str());
    return false;
  }
  if (!canSetParameter(param)) {
    return false;
  }
  RCLCPP_INFO(robot_manager_node_->get_logger(), "Can set parameter %s", param.get_name().c_str());
  if (param.as_string() == "position") {
    return robot_manager_->setPositionControlMode();
  } else if (param.as_string() == "joint_impedance") {
    try {
      return robot_manager_->setJointImpedanceControlMode(
        joint_stiffness_temp_,
        joint_damping_temp_);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(robot_manager_node_->get_logger(), e.what());
    }
    return false;
  } else {
    RCLCPP_ERROR(
      robot_manager_node_->get_logger(), "Invalid parameter value for parameter %s",
      param.get_name().c_str());
    return false;
  }
}

bool ConfigurationManager::onJointStiffnessChangeRequest(const rclcpp::Parameter & param)
{
  if (param.get_type() != rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE_ARRAY) {
    RCLCPP_ERROR(
      robot_manager_node_->get_logger(), "Invalid parameter type for parameter %s",
      param.get_name().c_str());
    return false;
  }
  if (!canSetParameter(param)) {
    return false;
  }
  if (param.as_double_array().size() != 7) {
    RCLCPP_ERROR(
      robot_manager_node_->get_logger(),
      "Invalid parameter array length for parameter %s", param.get_name().c_str());
    return false;
  }
  for (double js : param.as_double_array()) {
    if (js < 0) {
      RCLCPP_ERROR(
        robot_manager_node_->get_logger(), "Invalid parameter value for parameter %s",
        param.get_name().c_str());
      RCLCPP_ERROR(
        robot_manager_node_->get_logger(), "Joint stiffness values must be >=0",
        param.get_name().c_str());
      return false;
    }
  }
  joint_stiffness_temp_ = param.as_double_array();
  return true;
}

bool ConfigurationManager::onJointDampingChangeRequest(const rclcpp::Parameter & param)
{
  if (param.get_type() != rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE_ARRAY) {
    RCLCPP_ERROR(
      robot_manager_node_->get_logger(), "Invalid parameter type for parameter %s",
      param.get_name().c_str());
    return false;
  }
  if (!canSetParameter(param)) {
    return false;
  }
  if (param.as_double_array().size() != 7) {
    RCLCPP_ERROR(
      robot_manager_node_->get_logger(),
      "Invalid parameter array length for parameter %s", param.get_name().c_str());
    return false;
  }
  for (double jd : param.as_double_array()) {
    if (jd < 0 || jd > 1) {
      RCLCPP_ERROR(
        robot_manager_node_->get_logger(), "Invalid parameter value for parameter %s",
        param.get_name().c_str());
      RCLCPP_ERROR(robot_manager_node_->get_logger(), "Joint damping values must be >=0 && <=1");
      return false;
    }
  }
  joint_damping_temp_ = param.as_double_array();
  return true;
}

bool ConfigurationManager::onSendPeriodChangeRequest(const rclcpp::Parameter & param)
{
  if (param.get_type() != rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER) {
    RCLCPP_ERROR(
      robot_manager_node_->get_logger(), "Invalid parameter type for parameter %s",
      param.get_name().c_str());
    return false;
  }
  if (!canSetParameter(param)) {
    return false;
  }
  if (param.as_int() < 1 || param.as_int() > 100) {
    RCLCPP_ERROR(
      robot_manager_node_->get_logger(), "Invalid parameter value for parameter %s",
      param.get_name().c_str());
    RCLCPP_ERROR(
      robot_manager_node_->get_logger(),
      "Send period milliseconds must be >=1 && <=100");
    return false;
  }
  if (!setSendPeriod(param.as_int())) {
    return false;
  }
  return true;
}

bool ConfigurationManager::onReceiveMultiplierChangeRequest(const rclcpp::Parameter & param)
{
  if (param.get_type() != rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER) {
    RCLCPP_ERROR(
      robot_manager_node_->get_logger(), "Invalid parameter type for parameter %s",
      param.get_name().c_str());
    return false;
  }
  if (!canSetParameter(param)) {
    return false;
  }
  if (param.as_int() < 1) {
    RCLCPP_ERROR(
      robot_manager_node_->get_logger(), "Invalid parameter value for parameter %s",
      param.get_name().c_str());
    RCLCPP_ERROR(robot_manager_node_->get_logger(), "Receive multiplier must be >=1");
    return false;
  }
  if (!setReceiveMultiplier(param.as_int())) {
    return false;
  }
  return true;
}

bool ConfigurationManager::onControllerIpChangeRequest(const rclcpp::Parameter & param)
{
  if (param.get_type() != rcl_interfaces::msg::ParameterType::PARAMETER_STRING) {
    RCLCPP_ERROR(
      robot_manager_node_->get_logger(), "Invalid parameter type for parameter %s",
      param.get_name().c_str());
    return false;
  }
  if (!canSetParameter(param)) {
    return false;
  }
  // TODO(Svastits): check ip validity
  return true;
}

bool ConfigurationManager::setCommandMode(const std::string & control_mode)
{
  auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
  ClientCommandModeID client_command_mode;
  if (control_mode == "position") {
    request->data = false;
    client_command_mode = POSITION_COMMAND_MODE;
  } else if (control_mode == "torque") {
    request->data = true;
    client_command_mode = TORQUE_COMMAND_MODE;
  } else {
    RCLCPP_ERROR(robot_manager_node_->get_logger(), "Invalid control mode");
    return false;
  }
  auto future_result = command_mode_client_->async_send_request(request);
  auto future_status = wait_for_result(future_result, std::chrono::milliseconds(3000));
  if (future_status != std::future_status::ready) {
    RCLCPP_ERROR(
      robot_manager_node_->get_logger(), "Future status not ready, could not set command mode");
    return false;
  }
  if (future_result.get()->success) {
    if (robot_manager_) {
      robot_manager_->setClientCommandMode(client_command_mode);
    } else {
      RCLCPP_ERROR(robot_manager_node_->get_logger(), "Robot Manager not available");
      return false;
    }
  } else {
    RCLCPP_ERROR(
      robot_manager_node_->get_logger(), "Future result not success, could not set command mode");
    return false;
  }

  return true;
}

bool ConfigurationManager::setReceiveMultiplier(int receive_multiplier)
{
  // Set parameter of control client
  auto request = std::make_shared<kuka_sunrise_interfaces::srv::SetInt::Request>();
  request->data = receive_multiplier;
  auto future_result = receive_multiplier_client_->async_send_request(request);
  auto future_status = wait_for_result(future_result, std::chrono::milliseconds(3000));
  if (future_status != std::future_status::ready) {
    RCLCPP_ERROR(
      robot_manager_node_->get_logger(),
      "Future status not ready, could not set receive_multiplier");
    return false;
  }

  if (!future_result.get()->success) {
    RCLCPP_ERROR(
      robot_manager_node_->get_logger(),
      "Future result not success, could not set receive_multiplier");
    return false;
  }

  // Sync with joint controller
  future_result = sync_receive_multiplier_client_->async_send_request(request);
  future_status = wait_for_result(
    future_result,
    std::chrono::milliseconds(3000));
  if (future_status != std::future_status::ready) {
    RCLCPP_ERROR(
      robot_manager_node_->get_logger(),
      "Future status not ready, could not sync receive_multiplier");
    return false;
  }

  if (!future_result.get()->success) {
    RCLCPP_ERROR(
      robot_manager_node_->get_logger(),
      "Future result not success, could not sync receive_multiplier");
    return false;
  }
  return true;
}

bool ConfigurationManager::setSendPeriod(int send_period)
{
  auto request = std::make_shared<kuka_sunrise_interfaces::srv::SetInt::Request>();
  request->data = send_period;
  auto future_result = sync_send_period_client_->async_send_request(request);
  auto future_status = wait_for_result(future_result, std::chrono::milliseconds(3000));
  if (future_status != std::future_status::ready) {
    RCLCPP_ERROR(
      robot_manager_node_->get_logger(),
      "Future status not ready, could not sync send_period");
    return false;
  }

  if (!future_result.get()->success) {
    RCLCPP_ERROR(
      robot_manager_node_->get_logger(),
      "Future result not success, could not sync send_period");
    return false;
  }
  return true;
}
}  // namespace kuka_sunrise
