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

#include <vector>
#include <memory>
#include <string>
#include <cmath>

#include "control_logic/keyboard_control.hpp"

namespace teleop_guided_robot
{

double d2r(double degrees)
{
  return degrees / 180 * M_PI;
}

KeyboardControl::KeyboardControl(const std::string & node_name, const rclcpp::NodeOptions & options)
: LifecycleNode(node_name, options),
  lower_limits_rad_(7),
  upper_limits_rad_(7),
  changing_joint_(false),
  turning_velocity_increment_(3.0 * M_PI / 180),
  elapsed_time_treshold_(100000000),
  last_time_(RCL_ROS_TIME)
{
  auto qos = rclcpp::QoS(rclcpp::KeepAll());
  auto callback = [this](geometry_msgs::msg::Twist::SharedPtr msg) {
      this->messageReceivedCallback(msg);
    };
  key_teleop_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "key_vel", qos,
    callback);
  reference_joint_state_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>(
    "reference_joint_state", qos);
  reference_joint_state_ = std::make_shared<sensor_msgs::msg::JointState>();
  reference_joint_state_->position.resize(7);
  this->declare_parameter(
    "lower_limits_deg", rclcpp::ParameterValue(
      std::vector<double>(
        {-170, -120, -170, -120, -170,
          -120, -175})));
  this->declare_parameter(
    "upper_limits_deg", rclcpp::ParameterValue(
      std::vector<double>(
        {170, 120, 170, 120, 170, 120,
          175})));

  param_callback_ = this->add_on_set_parameters_callback(
    [this](const std::vector<rclcpp::Parameter> & parameters)
    {return this->onParamChange(parameters);});
  parameter_set_access_rights_.emplace(
    "lower_limits_deg", ParameterSetAccessRights {true, true,
      true, false});
  parameter_set_access_rights_.emplace(
    "upper_limits_deg", ParameterSetAccessRights {true, true,
      true, false});
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn KeyboardControl::
on_configure(
  const rclcpp_lifecycle::State & state)
{
  (void)state;
  onLowerLimitsChangeRequest(this->get_parameter("lower_limits_deg"));
  onUpperLimitsChangeRequest(this->get_parameter("upper_limits_deg"));
  return SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn KeyboardControl::
on_cleanup(
  const rclcpp_lifecycle::State & state)
{
  (void)state;
  reference_joint_state_->position.assign(7, 0);
  active_joint_ = 0;
  return SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn KeyboardControl::
on_activate(
  const rclcpp_lifecycle::State & state)
{
  (void)state;
  reference_joint_state_publisher_->on_activate();
  last_time_ = this->now();
  reference_joint_state_publisher_->publish(*reference_joint_state_);
  return SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn KeyboardControl::
on_deactivate(
  const rclcpp_lifecycle::State & state)
{
  (void)state;
  reference_joint_state_publisher_->on_deactivate();
  return SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn KeyboardControl::
on_shutdown(
  const rclcpp_lifecycle::State & state)
{
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn result = SUCCESS;
  switch (state.id()) {
    case lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE:
      result = this->on_deactivate(get_current_state());
      if (result != SUCCESS) {
        break;
      }
      result = this->on_cleanup(get_current_state());
      break;
    case lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE:
      result = this->on_cleanup(get_current_state());
      break;
    case lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED:
      break;
    default:
      break;
  }
  return result;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn KeyboardControl::on_error(
  const rclcpp_lifecycle::State & state)
{
  (void)state;
  RCLCPP_INFO(get_logger(), "An error occured");
  return SUCCESS;
}

void KeyboardControl::messageReceivedCallback(geometry_msgs::msg::Twist::SharedPtr msg)
{
  if (this->get_current_state().label() != "active") {
    return;
  }

  rclcpp::Time current_time = this->now();
  rclcpp::Duration elapsed_time = current_time - last_time_;
  if (elapsed_time > elapsed_time_treshold_) {
    double z = -msg->angular.z;
    // RCLCPP_INFO(get_logger(), "changing_joint: %i", changing_joint_);
    if (z > 0) {
      if (!changing_joint_) {
        changing_joint_ = true;
        if (++active_joint_ > 6) {
          active_joint_ = 0;
        }
        RCLCPP_INFO(get_logger(), "Active joint: %i", active_joint_ + 1);
      }
    } else if (z < 0) {
      if (!changing_joint_) {
        changing_joint_ = true;
        if (--active_joint_ < 0) {
          active_joint_ = 6;
        }
        RCLCPP_INFO(get_logger(), "Active joint: %i", active_joint_ + 1);
      }
    } else if (changing_joint_) {
      changing_joint_ = false;
    }

    double x = msg->linear.x;
    double x_dir = x > 0 ? x * WEIGHTS.X_POS : x * WEIGHTS.X_NEG;
    double new_reference_joint_state = reference_joint_state_->position[active_joint_] + x_dir *
      turning_velocity_increment_;
    if (lower_limits_rad_[active_joint_] * 0.9 < new_reference_joint_state &&
      new_reference_joint_state < upper_limits_rad_[active_joint_] * 0.9)
    {
      reference_joint_state_->position[active_joint_] = new_reference_joint_state;
      last_time_ = current_time;
      reference_joint_state_publisher_->publish(*reference_joint_state_);
    } else {
      RCLCPP_WARN(get_logger(), "Joint limit reached!");
    }
  }
}

rcl_interfaces::msg::SetParametersResult KeyboardControl::onParamChange(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = false;
  for (const rclcpp::Parameter & param : parameters) {
    if (param.get_name() == "lower_limits_deg" && canSetParameter(param)) {
      result.successful = onLowerLimitsChangeRequest(param);
    } else if (param.get_name() == "upper_limits_deg" && canSetParameter(param)) {
      result.successful = onUpperLimitsChangeRequest(param);
    } else {
      RCLCPP_ERROR(this->get_logger(), "Invalid parameter name %s", param.get_name().c_str());
    }
  }
  return result;
}

bool KeyboardControl::canSetParameter(const rclcpp::Parameter & param)
{
  try {
    if (!parameter_set_access_rights_.at(param.get_name()).isSetAllowed(
        this->get_current_state().id()))
    {
      RCLCPP_ERROR(
        this->get_logger(), "Parameter %s cannot be changed while in state %s",
        param.get_name().c_str(), this->get_current_state().label().c_str());
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

bool KeyboardControl::onLowerLimitsChangeRequest(const rclcpp::Parameter & param)
{
  if (param.get_type() != rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE_ARRAY) {
    RCLCPP_ERROR(
      this->get_logger(), "Invalid parameter type for parameter %s",
      param.get_name().c_str());
    return false;
  }


  if (param.as_double_array().size() != 7) {
    RCLCPP_ERROR(
      this->get_logger(), "Invalid parameter array length for parameter %s",
      param.get_name().c_str());
    return false;
  }
  std::transform(
    param.as_double_array().begin(),
    param.as_double_array().end(), lower_limits_rad_.begin(), d2r);
  return true;
}

bool KeyboardControl::onUpperLimitsChangeRequest(const rclcpp::Parameter & param)
{
  if (param.get_type() != rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE_ARRAY) {
    RCLCPP_ERROR(
      this->get_logger(), "Invalid parameter type for parameter %s",
      param.get_name().c_str());
    return false;
  }
  if (param.as_double_array().size() != 7) {
    RCLCPP_ERROR(
      this->get_logger(), "Invalid parameter array length for parameter %s",
      param.get_name().c_str());
    return false;
  }
  std::transform(
    param.as_double_array().begin(),
    param.as_double_array().end(), upper_limits_rad_.begin(), d2r);
  return true;
}

}  // namespace teleop_guided_robot

int main(int argc, char * argv[])
{
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor executor;
  auto node = std::make_shared<teleop_guided_robot::KeyboardControl>(
    "keyboard_control",
    rclcpp::NodeOptions());
  executor.add_node(node->get_node_base_interface());
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
