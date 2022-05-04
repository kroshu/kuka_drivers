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

#include "robot_control/joint_controller_base.hpp"

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

JointControllerBase::JointControllerBase(
  const std::string & node_name,
  const rclcpp::NodeOptions & options)
: kroshu_ros2_core::ROS2BaseNode(node_name, options)
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

  measured_joint_state_listener_ = this->create_subscription<
    sensor_msgs::msg::JointState>(
    "measured_joint_state", qos,
    [this](sensor_msgs::msg::JointState::SharedPtr state) {
      this->jointStateMeasurementsCallback(state);
    });

  joint_command_publisher_ = this->create_publisher<
    sensor_msgs::msg::JointState>("joint_command", qos);
  joint_controller_is_active_publisher_ = this->create_publisher<
    std_msgs::msg::Bool>("joint_controller_is_active", qos);

  param_callback_ = this->add_on_set_parameters_callback(
    [this](const std::vector<rclcpp::Parameter> & parameters) {
      return this->onParamChange(parameters);
    });

  this->declare_parameter(
    "max_velocities_degPs",
    rclcpp::ParameterValue(
      std::vector<double>(
        {300, 300, 400, 300, 160,
          160, 400})));
  this->declare_parameter(
    "lower_limits_deg",
    rclcpp::ParameterValue(
      std::vector<double>(
        {-170, -120, -170,
          -120, -170, -120, -175})));
  this->declare_parameter(
    "upper_limits_deg",
    rclcpp::ParameterValue(
      std::vector<double>(
        {170, 120, 170, 120,
          170, 120, 175})));

  auto send_period_callback = [this](
    kuka_sunrise_interfaces::srv::SetInt::Request::SharedPtr request,
    kuka_sunrise_interfaces::srv::SetInt::Response::SharedPtr response) {
      if (this->get_current_state().label() != "active") {
        send_period_ms_ = request->data;
        loop_period_ms_ = send_period_ms_ * receive_multiplier_;
        updateMaxPositionDifference();
        RCLCPP_INFO(
          get_logger(),
          "Succesfully synced send period");
        response->success = true;
      } else {
        RCLCPP_ERROR(
          get_logger(),
          "Joint controller is active, could not change send_period");
        response->success = false;
      }
    };

  sync_send_period_service_ = this->create_service<
    kuka_sunrise_interfaces::srv::SetInt>("sync_send_period", send_period_callback);

  auto receive_multiplier_callback = [this](
    kuka_sunrise_interfaces::srv::SetInt::Request::SharedPtr request,
    kuka_sunrise_interfaces::srv::SetInt::Response::SharedPtr response) {
      if (this->get_current_state().label() != "active") {
        receive_multiplier_ = request->data;
        loop_period_ms_ = send_period_ms_ * receive_multiplier_;
        updateMaxPositionDifference();
        RCLCPP_INFO(
          get_logger(),
          "Succesfully synced receive multiplier");
        response->success = true;
      } else {
        RCLCPP_ERROR(
          get_logger(),
          "Joint controller is active, could not change receive_multiplier");
        response->success = false;
      }
    };

  sync_receive_multiplier_service_ = this->create_service<
    kuka_sunrise_interfaces::srv::SetInt>("sync_receive_multiplier", receive_multiplier_callback);
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn JointControllerBase::
on_configure(
  const rclcpp_lifecycle::State &)
{
  joint_controller_is_active_ = std::make_shared<std_msgs::msg::Bool>();

  joint_command_ = std::make_shared<sensor_msgs::msg::JointState>();
  joint_command_->position.resize(7);
  joint_command_->velocity.resize(7);
  joint_command_->effort.resize(7);
  if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1) {
    RCLCPP_ERROR(get_logger(), "mlockall error");
    RCLCPP_ERROR(get_logger(), strerror(errno));
    return ERROR;
  }

  struct sched_param param;
  param.sched_priority = 90;
  if (sched_setscheduler(0, SCHED_FIFO, &param) == -1) {
    RCLCPP_ERROR(get_logger(), "setscheduler error");
    RCLCPP_ERROR(get_logger(), strerror(errno));
    return ERROR;
  }
  return SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn JointControllerBase::
on_cleanup(
  const rclcpp_lifecycle::State &)
{
  if (munlockall() == -1) {
    RCLCPP_ERROR(get_logger(), "munlockall error");
    return ERROR;
  }
  return SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn JointControllerBase::
on_activate(
  const rclcpp_lifecycle::State &)
{
  joint_command_publisher_->on_activate();
  joint_controller_is_active_publisher_->on_activate();
  joint_controller_is_active_->data = true;
  joint_controller_is_active_publisher_->publish(
    *joint_controller_is_active_);
  return SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn JointControllerBase::
on_deactivate(
  const rclcpp_lifecycle::State &)
{
  joint_command_publisher_->on_deactivate();
  joint_controller_is_active_->data = false;
  joint_controller_is_active_publisher_->publish(
    *joint_controller_is_active_);
  joint_controller_is_active_publisher_->on_deactivate();
  return SUCCESS;
}


rcl_interfaces::msg::SetParametersResult JointControllerBase::onParamChange(
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

bool JointControllerBase::canSetParameter(const rclcpp::Parameter & param)
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

bool JointControllerBase::onMaxVelocitiesChangeRequest(
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

bool JointControllerBase::onLowerLimitsChangeRequest(
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

bool JointControllerBase::onUpperLimitsChangeRequest(
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

void JointControllerBase::updateMaxPositionDifference()
{
  auto calc_pos_diff = [ & loop_period_ms_ = loop_period_ms_](double v) {
      return v * loop_period_ms_ / JointControllerBase::ms_in_sec_;
    };
  std::transform(
    max_velocities_radPs_.begin(), max_velocities_radPs_.end(),
    max_position_difference_.begin(), calc_pos_diff);
}

void JointControllerBase::jointStateMeasurementsCallback(
  sensor_msgs::msg::JointState::SharedPtr measured_joint_state)
{
  if (++loop_count_ == receive_multiplier_) {
    loop_count_ = 0;
    controlLoopCallback(measured_joint_state);
  }
}
}  // namespace robot_control
