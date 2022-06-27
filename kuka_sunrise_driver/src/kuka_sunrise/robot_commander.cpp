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

#include "kuka_sunrise/robot_commander.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/u_int64.hpp"
#include "std_msgs/msg/float64.hpp"

namespace kuka_sunrise
{

RobotCommander::RobotCommander(
  KUKA::FRI::LBRCommand & robot_command,
  const KUKA::FRI::LBRState & robot_state)
: robot_command_(robot_command), robot_state_(robot_state), torque_command_mode_(false),
  ros_clock_(RCL_ROS_TIME)
{
  auto qos = rclcpp::QoS(rclcpp::KeepLast(1));
  qos.best_effort();
  auto callback =
    [this](sensor_msgs::msg::JointState::ConstSharedPtr msg) -> void
    {this->commandReceivedCallback(msg);};
  // TODO(resizoltan) use TLSFAllocator? implement static strategy for jointstatemsg?
  auto msg_strategy = std::make_shared<MessageMemoryStrategy<sensor_msgs::msg::JointState>>();

  auto command_srv_callback = [this](
    std_srvs::srv::SetBool::Request::SharedPtr request,
    std_srvs::srv::SetBool::Response::SharedPtr response) {
      if (this->setTorqueCommanding(request->data)) {
        response->success = true;
      } else {
        response->success = false;
      }
    };

  // TODO(Svastits): create service for command mode changes
  /*set_command_mode_service_ = robot_control_node_->create_service<std_srvs::srv::SetBool>(
    "set_command_mode", command_srv_callback);*/
}

void RobotCommander::addBooleanOutputCommander(const std::string & name)
{
  // TODO(Svastits): add to command interfaces

  /*if (robot_control_node_->get_current_state().label() != "unconfigured") {
    return;  // TODO(resizoltan) handle other states
  }
  auto output_setter_func = [this](std::string name, bool value) -> void {
      return this->robot_command_.setBooleanIOValue(name.c_str(), value);
    };
  output_subscriptions_.emplace_back(
    std::make_unique<OutputSubscription<bool, std_msgs::msg::Bool>>(
      name, output_setter_func,
      is_active_,
      robot_control_node_));*/
}

void RobotCommander::addDigitalOutputCommander(const std::string & name)
{
  // TODO(Svastits): add to command interfaces

  /*if (robot_control_node_->get_current_state().label() != "unconfigured") {
    return;  // TODO(resizoltan) handle other states
  }
  auto output_setter_func = [this](std::string name, uint64_t value) -> void {
      return this->robot_command_.setDigitalIOValue(name.c_str(), value);
    };
  output_subscriptions_.emplace_back(
    std::make_unique<OutputSubscription<uint64_t, std_msgs::msg::UInt64>>(
      name, output_setter_func, is_active_, robot_control_node_));*/
}

void RobotCommander::addAnalogOutputCommander(const std::string & name)
{
  // TODO(Svastits): add to command interfaces

  /*if (robot_control_node_->get_current_state().label() != "unconfigured") {
    return;  // TODO(resizoltan) handle other states
  }
  auto output_setter_func = [this](std::string name, double value) -> void {
      return this->robot_command_.setAnalogIOValue(name.c_str(), value);
    };
  output_subscriptions_.emplace_back(
    std::make_unique<OutputSubscription<double, std_msgs::msg::Float64>>(
      name, output_setter_func,
      is_active_,
      robot_control_node_));*/
}

bool RobotCommander::setTorqueCommanding(bool is_torque_mode_active)
{
  if (!is_active_) {
    torque_command_mode_ = is_torque_mode_active;
    return true;
  } else {
    return false;
  }
}

void RobotCommander::updateCommand(const rclcpp::Time & stamp)
{
  if (!is_active_) {
    printf("robot commander deactivated, exiting updatecommand\n");
    return;
  }

  if (torque_command_mode_) {
    if (joint_command_msg_->effort.empty()) {
      // raise some error/warning
      printf("Effort of joint command msg is empty in torque command mode\n");
      return;
    }
    const double * joint_torques_ = joint_command_msg_->effort.data();
    robot_command_.setJointPosition(robot_state_.getIpoJointPosition());
    robot_command_.setTorque(joint_torques_);
  } else {
    if (joint_command_msg_->position.empty()) {
      // raise some error/warning
      printf("Position of joint command msg is empty in position command mode\n");
      return;
    }
    const double * joint_positions_ = joint_command_msg_->position.data();
    robot_command_.setJointPosition(joint_positions_);
  }

  for (auto & output_subscription : output_subscriptions_) {
    output_subscription->updateOutput();
  }
}


bool RobotCommander::deactivate()
{
  is_active_ = false;
  return true;
}

}  // namespace kuka_sunrise
