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
InterpolatingController::InterpolatingController(
  const std::string & node_name,
  const rclcpp::NodeOptions & options)
: JointControllerBase(node_name, options)
{
  auto qos = rclcpp::QoS(rclcpp::KeepLast(1));
  qos.best_effort();

  reference_joint_state_listener_ = this->create_subscription<
    sensor_msgs::msg::JointState>(
    "reference_joint_state", qos,
    [this](sensor_msgs::msg::JointState::SharedPtr state) {
      this->referenceUpdateCallback(state);
    });
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
    joint_command_.velocity = reference_joint_state_->velocity;
  }
  if (reference_joint_state_->effort.size() == 7) {
    joint_command_.effort = reference_joint_state_->effort;
  }
  joint_command_.header = measured_joint_state->header;
  jointCommandPublisher()->publish(joint_command_);
}


void InterpolatingController::setJointCommandPosition(
  const std::vector<double> &)
{
  joint_command_.position = reference_joint_state_->position;
}

void InterpolatingController::enforceSpeedLimits(
  const std::vector<double> & measured_joint_position)
{
  std::vector<double> & joint_command_position = joint_command_.position;
  for (int i = 0; i < 7; i++) {
    if (abs(measured_joint_position[i] - joint_command_position[i]) <=
      maxPosDiff()[i])
    {
      RCLCPP_DEBUG(
        get_logger(),
        "Successfully set step size to the speed of movement");
    } else if (joint_command_position[i] > measured_joint_position[i]) {
      joint_command_position[i] = measured_joint_position[i] +
        maxPosDiff()[i];
      RCLCPP_DEBUG(get_logger(), "Movement was too fast around joint %i", i + 1);

    } else if (joint_command_position[i] < measured_joint_position[i]) {
      joint_command_position[i] = measured_joint_position[i] -
        maxPosDiff()[i];
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
  for (int i = 0; i < 7; i++) {
    if (reference_joint_positions[i] < lowerLimitsRad()[i]) {
      reference_joint_positions[i] = lowerLimitsRad()[i];
      RCLCPP_WARN(
        get_logger(),
        "Reference for joint %i exceeded lower limit", i + 1);
    } else if (reference_joint_positions[i] > upperLimitsRad()[i]) {
      reference_joint_positions[i] = upperLimitsRad()[i];
      RCLCPP_WARN(
        get_logger(),
        "Reference for joint %i exceeded upper limit", i + 1);
    }
  }
  reference_joint_state_ = reference_joint_state;
}

sensor_msgs::msg::JointState::ConstSharedPtr InterpolatingController::refJointState() const
{
  return reference_joint_state_;
}

}  // namespace robot_control
