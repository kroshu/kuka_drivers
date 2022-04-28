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

#include "robot_control/interpolating_controller.hpp"

#include <sys/mman.h>
#include <string>
#include <memory>
#include <vector>

namespace robot_control
{

double d2r(double degrees)
{
  return degrees / 180 * M_PI;
}

InterpolatingController::InterpolatingController(
  const std::string & node_name,
  const rclcpp::NodeOptions & options)
: JointControllerBase(node_name, options)
{
  parameter_set_access_rights_.emplace(
    "max_velocities_degPs",
    ParameterSetAccessRights {true, true, false, false});
  parameter_set_access_rights_.emplace(
    "lower_limits_deg",
    ParameterSetAccessRights {true, true, false, false});
  parameter_set_access_rights_.emplace(
    "upper_limits_deg",
    ParameterSetAccessRights {true, true, false, false});


  auto qos = rclcpp::QoS(rclcpp::KeepLast(1));
  qos.best_effort();

  reference_joint_state_listener_ = this->create_subscription<
    sensor_msgs::msg::JointState>(
    "reference_joint_state", qos,
    [this](sensor_msgs::msg::JointState::SharedPtr state) {
      this->referenceUpdateCallback(state);
    });

  param_callback_ = this->add_on_set_parameters_callback(
    [this](const std::vector<rclcpp::Parameter> & parameters) {
      return this->onParamChange(parameters);
    });
}

rcl_interfaces::msg::SetParametersResult InterpolatingController::onParamChange(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  for (const rclcpp::Parameter & param : parameters) {
    if (param.get_name() == "max_velocities_degPs" && canSetParameter(param)) {
      result.successful = onMaxVelocitiesChangeRequest(param);
    } else if (param.get_name() == "lower_limits_deg" &&  // NOLINT
      canSetParameter(param))
    {
      result.successful = onLowerLimitsChangeRequest(param);
    } else if (param.get_name() == "upper_limits_deg" &&  // NOLINT
      canSetParameter(param))
    {
      result.successful = onUpperLimitsChangeRequest(param);
    } else {
      RCLCPP_ERROR(
        this->get_logger(), "Invalid parameter name %s",
        param.get_name().c_str());
      result.successful = false;
    }
  }
  return result;
}

bool InterpolatingController::canSetParameter(const rclcpp::Parameter & param)
{
  try {
    if (!parameter_set_access_rights_.at(param.get_name()).isSetAllowed(
        this->get_current_state().id()))
    {
      RCLCPP_ERROR(
        this->get_logger(),
        "Parameter %s cannot be changed while in state %s",
        param.get_name().c_str(),
        this->get_current_state().label().c_str());
      return false;
    }
  } catch (const std::out_of_range & e) {
    RCLCPP_ERROR(
      this->get_logger(),
      "Parameter set access rights for parameter %s couldn't be determined",
      param.get_name().c_str());
    return false;
  }
  return true;
}

bool InterpolatingController::onMaxVelocitiesChangeRequest(
  const rclcpp::Parameter & param)
{
  if (param.get_type() !=
    rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE_ARRAY)
  {
    RCLCPP_ERROR(
      this->get_logger(),
      "Invalid parameter type for parameter %s",
      param.get_name().c_str());
    return false;
  }

  if (param.as_double_array().size() != 7) {
    RCLCPP_ERROR(
      this->get_logger(),
      "Invalid parameter array length for parameter %s",
      param.get_name().c_str());
    return false;
  }
  std::transform(
    param.as_double_array().begin(), param.as_double_array().end(),
    max_velocities_radPs_.begin(), [](double v) {
      return d2r(v * 0.9);
    });
  updateMaxPositionDifference();
  return true;
}

bool InterpolatingController::onLowerLimitsChangeRequest(
  const rclcpp::Parameter & param)
{
  if (param.get_type() !=
    rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE_ARRAY)
  {
    RCLCPP_ERROR(
      this->get_logger(),
      "Invalid parameter type for parameter %s",
      param.get_name().c_str());
    return false;
  }

  if (param.as_double_array().size() != 7) {
    RCLCPP_ERROR(
      this->get_logger(),
      "Invalid parameter array length for parameter %s",
      param.get_name().c_str());
    return false;
  }
  std::transform(
    param.as_double_array().begin(),
    param.as_double_array().end(), lower_limits_rad_.begin(), [](double v) {
      return d2r(v * 0.9);
    });
  return true;
}

bool InterpolatingController::onUpperLimitsChangeRequest(
  const rclcpp::Parameter & param)
{
  if (param.get_type() !=
    rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE_ARRAY)
  {
    RCLCPP_ERROR(
      this->get_logger(),
      "Invalid parameter type for parameter %s",
      param.get_name().c_str());
    return false;
  }

  if (param.as_double_array().size() != 7) {
    RCLCPP_ERROR(
      this->get_logger(),
      "Invalid parameter array length for parameter %s",
      param.get_name().c_str());
    return false;
  }
  std::transform(
    param.as_double_array().begin(),
    param.as_double_array().end(), upper_limits_rad_.begin(), [](double v) {
      return d2r(v * 0.9);
    });
  return true;
}


void InterpolatingController::controlLoopCallback(
  sensor_msgs::msg::JointState::SharedPtr measured_joint_state)
{
  if (!reference_joint_state_) {
    reference_joint_state_ = measured_joint_state;
  }
  if (reference_joint_state_->position.size() == 7) {
    setJointCommandPosition(measured_joint_state->position);
    enforceSpeedLimits(measured_joint_state->position);
  }
  if (reference_joint_state_->velocity.size() == 7) {
    joint_command_->velocity = reference_joint_state_->velocity;
  }
  if (reference_joint_state_->effort.size() == 7) {
    joint_command_->effort = reference_joint_state_->effort;
  }
  joint_command_->header = measured_joint_state->header;
  joint_command_publisher_->publish(*joint_command_);
}


void InterpolatingController::setJointCommandPosition(
  const std::vector<double> & measured_joint_position)
{
  (void) measured_joint_position;
  joint_command_->position = reference_joint_state_->position;
}

void InterpolatingController::enforceSpeedLimits(
  const std::vector<double> & measured_joint_position)
{
  std::vector<double> & joint_command_position = joint_command_->position;
  for (int i = 0; i < 7; i++) {
    if (abs(measured_joint_position[i] - joint_command_position[i]) <=
      max_position_difference_[i])
    {
      RCLCPP_DEBUG(
        get_logger(),
        "Successfully set step size to the speed of movement");
    } else if (joint_command_position[i] > measured_joint_position[i]) {
      joint_command_position[i] = measured_joint_position[i] +
        max_position_difference_[i];
      RCLCPP_DEBUG(get_logger(), "Movement was too fast around joint %i", i + 1);

    } else if (joint_command_position[i] < measured_joint_position[i]) {
      joint_command_position[i] = measured_joint_position[i] -
        max_position_difference_[i];
      RCLCPP_DEBUG(get_logger(), "Movement was too fast around joint %i", i + 1);

    } else {
      RCLCPP_ERROR(get_logger(), "Reference or measured joint state is NaN");
    }
  }
}

void InterpolatingController::referenceUpdateCallback(
  sensor_msgs::msg::JointState::SharedPtr reference_joint_state)
{
  if (this->get_current_state().label() != "active") {
    return;
  }
  auto & reference_joint_positions = reference_joint_state->position;
  if (!reference_joint_state_) {
    reference_joint_state_ = reference_joint_state;
  }
  for (int i = 0; i < 7; i++) {
    if (reference_joint_positions[i] < lower_limits_rad_[i]) {
      reference_joint_positions[i] = lower_limits_rad_[i];
      RCLCPP_WARN(
        get_logger(),
        "Reference for joint %i exceeded lower limit", i + 1);
    } else if (reference_joint_positions[i] > upper_limits_rad_[i]) {
      reference_joint_positions[i] = upper_limits_rad_[i];
      RCLCPP_WARN(
        get_logger(),
        "Reference for joint %i exceeded upper limit", i + 1);
    }
  }
  reference_joint_state_ = reference_joint_state;
}

}  // namespace robot_control
