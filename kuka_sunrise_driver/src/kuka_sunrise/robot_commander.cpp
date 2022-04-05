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
  const KUKA::FRI::LBRState & robot_state,
  rclcpp_lifecycle::LifecycleNode::SharedPtr robot_control_node)
: robot_command_(robot_command), robot_state_(robot_state), torque_command_mode_(false),
  robot_control_node_(robot_control_node), ros_clock_(RCL_ROS_TIME)
{
  auto qos = rclcpp::QoS(rclcpp::KeepLast(1));
  qos.best_effort();
  auto callback =
    [this](sensor_msgs::msg::JointState::ConstSharedPtr msg) -> void
    {this->commandReceivedCallback(msg);};
  // TODO(resizoltan) use TLSFAllocator? implement static strategy for jointstatemsg?
  auto msg_strategy = std::make_shared<MessageMemoryStrategy<sensor_msgs::msg::JointState>>();
  joint_command_subscription_ = robot_control_node_->create_subscription<
    sensor_msgs::msg::JointState>(
    "lbr_joint_command", qos, callback,
    rclcpp::SubscriptionOptions(), msg_strategy);

  auto command_srv_callback = [this](
    std_srvs::srv::SetBool::Request::SharedPtr request,
    std_srvs::srv::SetBool::Response::SharedPtr response) {
      if (this->setTorqueCommanding(request->data)) {
        response->success = true;
      } else {
        response->success = false;
      }
    };
  set_command_mode_service_ = robot_control_node_->create_service<std_srvs::srv::SetBool>(
    "set_command_mode", command_srv_callback);
}

void RobotCommander::addBooleanOutputCommander(const std::string & name)
{
  if (robot_control_node_->get_current_state().label() != "unconfigured") {
    return;  // TODO(resizoltan) handle other states
  }
  auto output_setter_func = [this](std::string name, bool value) -> void {
      return this->robot_command_.setBooleanIOValue(name.c_str(), value);
    };
  output_subscriptions_.emplace_back(
    std::make_unique<OutputSubscription<bool, std_msgs::msg::Bool>>(
      name, output_setter_func,
      is_active_,
      robot_control_node_));
}

void RobotCommander::addDigitalOutputCommander(const std::string & name)
{
  if (robot_control_node_->get_current_state().label() != "unconfigured") {
    return;  // TODO(resizoltan) handle other states
  }
  auto output_setter_func = [this](std::string name, uint64_t value) -> void {
      return this->robot_command_.setDigitalIOValue(name.c_str(), value);
    };
  output_subscriptions_.emplace_back(
    std::make_unique<OutputSubscription<uint64_t, std_msgs::msg::UInt64>>(
      name, output_setter_func, is_active_, robot_control_node_));
}

void RobotCommander::addAnalogOutputCommander(const std::string & name)
{
  if (robot_control_node_->get_current_state().label() != "unconfigured") {
    return;  // TODO(resizoltan) handle other states
  }
  auto output_setter_func = [this](std::string name, double value) -> void {
      return this->robot_command_.setAnalogIOValue(name.c_str(), value);
    };
  output_subscriptions_.emplace_back(
    std::make_unique<OutputSubscription<double, std_msgs::msg::Float64>>(
      name, output_setter_func,
      is_active_,
      robot_control_node_));
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
  std::unique_lock<std::mutex> lk(m_);
  while (!joint_command_msg_ || joint_command_msg_->header.stamp != stamp) {
    if (!is_active_) {
      RCLCPP_INFO(
        robot_control_node_->get_logger(),
        "robot commander deactivated, exiting updatecommand");
      return;
    }
    cv_.wait(lk);
    // check if wait has been interrupted by the robot manager
    if (!is_active_) {
      RCLCPP_INFO(
        robot_control_node_->get_logger(),
        "robot commander deactivated, exiting updatecommand");
      return;
    }
  }

  if (torque_command_mode_) {
    if (joint_command_msg_->effort.empty()) {
      // raise some error/warning
      RCLCPP_ERROR(
        robot_control_node_->get_logger(),
        "Effort of joint command msg is empty in torque command mode");
      return;
    }
    const double * joint_torques_ = joint_command_msg_->effort.data();
    robot_command_.setJointPosition(robot_state_.getIpoJointPosition());
    robot_command_.setTorque(joint_torques_);
  } else {
    if (joint_command_msg_->position.empty()) {
      // raise some error/warning
      RCLCPP_ERROR(
        robot_control_node_->get_logger(),
        "Position of joint command msg is empty in position command mode");
      return;
    }
    const double * joint_positions_ = joint_command_msg_->position.data();
    robot_command_.setJointPosition(joint_positions_);
  }

  for (auto & output_subscription : output_subscriptions_) {
    output_subscription->updateOutput();
  }
}

void RobotCommander::commandReceivedCallback(sensor_msgs::msg::JointState::ConstSharedPtr msg)
{
  std::lock_guard<std::mutex> lk(m_);
  if (!is_active_) {
    RCLCPP_INFO(robot_control_node_->get_logger(), "commander not activated");
    return;
  }
  joint_command_msg_ = msg;
  cv_.notify_one();
}

bool RobotCommander::deactivate()
{
  std::lock_guard<std::mutex> lk(m_);
  is_active_ = false;
  cv_.notify_one();  // interrupt updateCommand()
  return true;
}

}  // namespace kuka_sunrise
