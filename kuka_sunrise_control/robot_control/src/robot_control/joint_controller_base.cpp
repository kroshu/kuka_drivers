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

JointControllerBase::JointControllerBase(
  const std::string & node_name,
  const rclcpp::NodeOptions & options)
: kroshu_ros2_core::ROS2BaseNode(node_name, options)
{
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
