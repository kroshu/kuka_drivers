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

RateScaledJointController::RateScaledJointController(
  const std::string & node_name,
  const rclcpp::NodeOptions & options)
: InterpolatingController(node_name, options)
{
  // Enforce setting parameter by giving invalid value as default
  registerParameter<double>(
    "reference_rate", 0, kroshu_ros2_core::ParameterSetAccessRights {true, true,
      false, false}, [this](const double & ref_rate) {
      return this->OnReferenceRateChangeRequest(ref_rate);
    });
/*
  auto set_rate_callback = [this](
    kuka_sunrise_interfaces::srv::SetDouble::Request::SharedPtr request,
    kuka_sunrise_interfaces::srv::SetDouble::Response::SharedPtr response) {
      int cmd_per_frame = static_cast<int>(JointControllerBase::ms_in_sec_ /
        loopPeriod() / (8 * request->data)) + 1;
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
    "joint_controller/set_rate", set_rate_callback);*/
}

void RateScaledJointController::setJointCommandPosition(
  const std::vector<double> & measured_joint_position)
{
  // referenceUpdateCallback is called is base class constructor, so it cannot be ovverriden
  // as a workaround, set velocity scaling here, if reference has changed
  if (refJointState()->position != prev_ref_joint_pos_) {
    setSlowStart();
  }
  const std::vector<double> & reference_joint_position =
    refJointState()->position;
  std::vector<double> & joint_command_position = joint_command_->position;
  for (int i = 0; i < 7; i++) {
    if (maxPosDiff()[i] < 0) {
      RCLCPP_WARN(get_logger(), "max position difference is not positive");
    }
    double position_error = reference_joint_position[i] -
      measured_joint_position[i];
    double reference_error = reference_joint_position[i] -
      joint_command_position[i];

    // Set speed so, that the motion finishes when the next reference is received
    if (cmd_count_ >= cmd_per_frame_) {
      joint_command_position[i] = reference_joint_position[i];
      if (i == 0) {
        RCLCPP_DEBUG(
          get_logger(),
          "Frame not received in expected time, command count is %i",
          cmd_count_);
      }
    } else if (start_flag_) {  // First command: based on measured
      joint_command_position[i] = measured_joint_position[i] +
        position_error / (cmd_per_frame_ - cmd_count_);
      if (i == 6) {
        start_flag_ = false;
        prev_ref_joint_pos_ = reference_joint_position;
      }
      RCLCPP_DEBUG(get_logger(), "First command");
    } else {  // Not first command: based on previous command
      joint_command_position[i] += reference_error /
        (cmd_per_frame_ - cmd_count_);
      RCLCPP_DEBUG(get_logger(), "Command calculated relative to previous");
    }
  }
}

void RateScaledJointController::enforceSpeedLimits(
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
      maxPosDiff()[i] * vel_factor)
    {
      RCLCPP_DEBUG(
        get_logger(),
        "Successfully set step size to the speed of movement");
    } else if (joint_command_position[i] > measured_joint_position[i]) {
      joint_command_position[i] = measured_joint_position[i] +
        maxPosDiff()[i] * vel_factor;
      RCLCPP_DEBUG(get_logger(), "Movement was too fast around joint %i", i + 1);
    } else if (joint_command_position[i] < measured_joint_position[i]) {
      joint_command_position[i] = measured_joint_position[i] -
        maxPosDiff()[i] * vel_factor;
      RCLCPP_DEBUG(get_logger(), "Movement was too fast around joint %i", i + 1);
    } else {
      RCLCPP_ERROR(get_logger(), "Reference or measured joint state is NaN");
    }
  }
  cmd_count_++;
}

void RateScaledJointController::setSlowStart()
{
  auto & reference_joint_positions = refJointState()->position;
  // if the change of reference changes sign, mark that axis to be slowed down
  for (int i = 0; i < 7; i++) {
    if ((prev_ref_joint_pos_[i] - pprev_ref_joint_pos_[i]) *
      (reference_joint_positions[i] - prev_ref_joint_pos_[i]) > 0)
    {
      slow_start_[i] = false;
    } else if (reference_joint_positions[i] == prev_ref_joint_pos_[i]) {
      slow_start_[i] = false;
    } else if (reference_joint_positions[i] > prev_ref_joint_pos_[i] &&     // NOLINT
      joint_command_->position[i] > reference_joint_positions[i])
    {
      slow_start_[i] = false;
    } else if (reference_joint_positions[i] < prev_ref_joint_pos_[i] &&     // NOLINT
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
  prev_ref_joint_pos_ = refJointState()->position;
}

bool RateScaledJointController::OnReferenceRateChangeRequest(const double & reference_rate)
{
  if (reference_rate < 1) {
    RCLCPP_ERROR(
      get_logger(),
      "Reference rate should be at least 1 [Hz]");
    return false;
  }
  int cmd_per_frame = static_cast<int>(JointControllerBase::ms_in_sec_ /
    loopPeriod() / reference_rate) + 1;

  if (cmd_per_frame < 2) {
    RCLCPP_ERROR(
      get_logger(),
      "Control loop frequency should be bigger than command receive frequency");
    return false;
  }
  cmd_per_frame_temp_ = cmd_per_frame;
  RCLCPP_INFO(
    get_logger(),
    "Successfully changed rate, receiving commands in every %i. frame",
    cmd_per_frame);
  return true;
}


}  // namespace robot_control

int main(int argc, char * argv[])
{
  setvbuf(stdout, nullptr, _IONBF, BUFSIZ);

  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor executor;
  auto node = std::make_shared<robot_control::RateScaledJointController>(
    "joint_controller", rclcpp::NodeOptions());
  executor.add_node(node->get_node_base_interface());
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
