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

#include "robot_control/rate_scaled_controller.hpp"

#include <sys/mman.h>
#include <string>
#include <memory>
#include <vector>

namespace robot_control
{

ScaledJointController::ScaledJointController(
  const std::string & node_name,
  const rclcpp::NodeOptions & options)
: InterpolatingController(node_name, options)
{
  auto set_rate_callback = [this](
    kuka_sunrise_interfaces::srv::SetDouble::Request::SharedPtr request,
    kuka_sunrise_interfaces::srv::SetDouble::Response::SharedPtr response) {
      int cmd_per_frame = static_cast<int>(JointControllerBase::ms_in_sec_ /
        loop_period_ms_ / (8 * request->data)) + 1;
      if (cmd_per_frame > 2) {
        cmd_per_frame_temp_ = cmd_per_frame;
        RCLCPP_INFO(
          get_logger(),
          "Successfully changed rate, receiving commands in every %i. frame",
          cmd_per_frame);
        response->success = true;
      } else {
        RCLCPP_ERROR(
          get_logger(),
          "Control loop frequency should be bigger than command receive frequency");
        response->success = false;
      }
    };

  set_rate_service_ = this->create_service<kuka_sunrise_interfaces::srv::SetDouble>(
    "joint_controller/set_rate", set_rate_callback);
}

void ScaledJointController::setJointCommandPosition(
  const std::vector<double> & measured_joint_position)
{
  const std::vector<double> & reference_joint_position =
    reference_joint_state_->position;
  std::vector<double> & joint_command_position = joint_command_->position;
  for (int i = 0; i < 7; i++) {
    if (max_position_difference_[i] < 0) {
      RCLCPP_WARN(get_logger(), "max position difference is not positive");
    }
    double position_error = reference_joint_position[i] -
      measured_joint_position[i];
    double reference_error = reference_joint_position[i] -
      joint_command_position[i];

    // Set speed so, that the motion finishes when the next reference is received
    if (cmd_count_ >= ScaledJointController::cmd_per_frame_) {
      joint_command_position[i] = reference_joint_position[i];
      if (i == 0) {
        RCLCPP_DEBUG(
          get_logger(),
          "Frame not received in expected time, command count is %i",
          cmd_count_);
      }
    } else if (start_flag_) {  // First command: based on measured
      joint_command_position[i] = measured_joint_position[i] +
        position_error / (ScaledJointController::cmd_per_frame_ - cmd_count_);
      if (i == 6) {
        start_flag_ = false;
        prev_ref_joint_pos_ = reference_joint_position;
      }
      RCLCPP_DEBUG(get_logger(), "First command");
    } else {  // Not first command: based on previous command
      joint_command_position[i] += reference_error /
        (ScaledJointController::cmd_per_frame_ - cmd_count_);
      RCLCPP_DEBUG(get_logger(), "Command calculated relative to previous");
    }
  }
}

void ScaledJointController::enforceSpeedLimits(
  const std::vector<double> & measured_joint_position)
{
  std::vector<double> & joint_command_position = joint_command_->position;
  for (int i = 0; i < 7; i++) {
    // If axis is marked, slow down motion at new reference
    double vel_factor;
    if (slow_start_[i]) {
      if (cmd_count_ == 0) {
        vel_factor = 0.55;
      } else if (cmd_count_ == 1) {
        vel_factor = 0.8;
      } else {
        vel_factor = 1;
      }
    } else {
      vel_factor = 1;
    }
    if (abs(measured_joint_position[i] - joint_command_position[i]) <=
      max_position_difference_[i] * vel_factor)
    {
      RCLCPP_DEBUG(
        get_logger(),
        "Successfully set step size to the speed of movement");
    } else if (joint_command_position[i] > measured_joint_position[i]) {
      joint_command_position[i] = measured_joint_position[i] +
        max_position_difference_[i] * vel_factor;
      RCLCPP_DEBUG(get_logger(), "Movement was too fast around joint %i", i + 1);
    } else if (joint_command_position[i] < measured_joint_position[i]) {
      joint_command_position[i] = measured_joint_position[i] -
        max_position_difference_[i] * vel_factor;
      RCLCPP_DEBUG(get_logger(), "Movement was too fast around joint %i", i + 1);
    } else {
      RCLCPP_ERROR(get_logger(), "Reference or measured joint state is NaN");
    }
  }
  cmd_count_++;
}

void ScaledJointController::referenceUpdateCallback(
  sensor_msgs::msg::JointState::SharedPtr reference_joint_state)
{
  if (this->get_current_state().label() != "active") {
    return;
  }
  auto & reference_joint_positions = reference_joint_state->position;
  if (!reference_joint_state_) {
    reference_joint_state_ = reference_joint_state;
  }
  auto & p_reference_joint_positions = reference_joint_state_->position;
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
    // if the change of reference changes sign, mark that axis to be slowed down
    if ((p_reference_joint_positions[i] - prev_ref_joint_pos_[i]) *
      (reference_joint_positions[i] - p_reference_joint_positions[i]) > 0)
    {
      slow_start_[i] = false;
    } else if (reference_joint_positions[i] == p_reference_joint_positions[i]) {
      slow_start_[i] = false;
    } else if (reference_joint_positions[i] > p_reference_joint_positions[i] &&  // NOLINT
      joint_command_->position[i] > reference_joint_positions[i])
    {
      slow_start_[i] = false;
    } else if (reference_joint_positions[i] < p_reference_joint_positions[i] &&  // NOLINT
      joint_command_->position[i] < reference_joint_positions[i])
    {
      slow_start_[i] = false;
    } else {
      slow_start_[i] = true;
    }
  }
  RCLCPP_DEBUG(get_logger(), "commands per frame: %i", cmd_count_);
  cmd_count_ = 0;
  if (cmd_per_frame_temp_) {
    cmd_per_frame_ = cmd_per_frame_temp_;
    cmd_per_frame_temp_ = 0;
  }
  prev_ref_joint_pos_ = reference_joint_state_->position;
  reference_joint_state_ = reference_joint_state;
}

}  // namespace robot_control

int main(int argc, char * argv[])
{
  setvbuf(stdout, nullptr, _IONBF, BUFSIZ);

  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor executor;
  auto node = std::make_shared<robot_control::ScaledJointController>(
    "joint_controller", rclcpp::NodeOptions());
  executor.add_node(node->get_node_base_interface());
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
