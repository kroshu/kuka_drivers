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

#include "rclcpp/time.hpp"

#include "kuka_sunrise/robot_observer.hpp"

namespace kuka_sunrise
{

void RobotObserver::addBooleanInputObserver(std::string name)
{
  if (robot_control_node_->get_current_state().label() != "unconfigured") {
    return;  // TODO(resizoltan) handle other states
  }
  auto input_getter_func = [this](std::string name) -> bool {
      return this->robot_state_.getBooleanIOValue(name.c_str());
    };
  input_publishers_.emplace_back(
    std::make_unique<InputPublisher<bool, std_msgs::msg::Bool>>(
      name,
      input_getter_func, robot_control_node_));
}

void RobotObserver::addDigitalInputObserver(std::string name)
{
  if (robot_control_node_->get_current_state().label() != "unconfigured") {
    return;  // TODO(resizoltan) handle other states
  }
  auto input_getter_func = [this](std::string name) -> uint64_t {
      return this->robot_state_.getDigitalIOValue(name.c_str());
    };
  input_publishers_.emplace_back(
    std::make_unique<InputPublisher<uint64_t, std_msgs::msg::UInt64>>(
      name,
      input_getter_func, robot_control_node_));
}

void RobotObserver::addAnalogInputObserver(std::string name)
{
  if (robot_control_node_->get_current_state().label() != "unconfigured") {
    return;  // TODO(resizoltan) handle other states
  }
  auto input_getter_func = [this](std::string name) -> double {
      return this->robot_state_.getDigitalIOValue(name.c_str());
    };
  input_publishers_.emplace_back(
    std::make_unique<InputPublisher<double, std_msgs::msg::Float64>>(
      name,
      input_getter_func, robot_control_node_));
}

bool RobotObserver::activate()
{
  this->ActivatableInterface::activate();
  joint_state_publisher_->on_activate();
  tracking_performance_publisher_->on_activate();
  return true;
}

bool RobotObserver::deactivate()
{
  this->ActivatableInterface::deactivate();
  joint_state_publisher_->on_deactivate();
  tracking_performance_publisher_->on_deactivate();
  return true;
}

RobotObserver::RobotObserver(
  const KUKA::FRI::LBRState & robot_state,
  rclcpp_lifecycle::LifecycleNode::SharedPtr robot_control_node)
: robot_state_(robot_state), robot_control_node_(robot_control_node)
{
  joint_state_msg_.position.reserve(robot_state_.NUMBER_OF_JOINTS);
  joint_state_msg_.velocity.reserve(robot_state_.NUMBER_OF_JOINTS);
  joint_state_msg_.effort.reserve(robot_state_.NUMBER_OF_JOINTS);
  auto qos = rclcpp::QoS(rclcpp::KeepLast(1));
  qos.best_effort();
  joint_state_publisher_ = robot_control_node->create_publisher<
    sensor_msgs::msg::JointState>("lbr_joint_state", 1);
  // joint_state_publisher2_ =
  //    robot_control_node->create_publisher<sensor_msgs::msg::JointState>("lbr_joint_state2", qos);
  tracking_performance_publisher_ = robot_control_node->create_publisher<
    std_msgs::msg::Float64>("tracking_performance", qos);
  // joint_state_publisher_ =
  //    robot_control_node->create_publisher<sensor_msgs::msg::JointState>("lbr_joint_state", qos);
}

void RobotObserver::publishRobotState(const rclcpp::Time & stamp)
{
  if (robot_control_node_->get_current_state().label() != "inactive" &&
    robot_control_node_->get_current_state().label() != "active")
  {
    return;  // TODO(resizoltan) handle other states
  }

  joint_state_msg_.header.frame_id = "world";
  joint_state_msg_.header.stamp = stamp;  // TODO(resizoltan) catch exceptions

  /*const double *joint_positions_measured = robot_state_.getMeasuredJointPosition();
   const double *joint_torques_measured = robot_state_.getMeasuredTorque();

   joint_state_msg_.velocity.clear();
   joint_state_msg_.position.assign(joint_positions_measured, joint_positions_measured + robot_state_.NUMBER_OF_JOINTS);
   joint_state_msg_.effort.assign(joint_torques_measured, joint_torques_measured + robot_state_.NUMBER_OF_JOINTS);
   joint_state_publisher2_->publish(joint_state_msg_);*/
  // RCLCPP_INFO(robot_control_node_->get_logger(), "%u", robot_state_.getSessionState());
  if (robot_state_.getSessionState() == KUKA::FRI::COMMANDING_WAIT ||
    robot_state_.getSessionState() == KUKA::FRI::COMMANDING_ACTIVE)
  {
    joint_state_msg_.name = std::vector<std::string> {"URDFLBRiiwa14Joint1",
      "URDFLBRiiwa14Joint2", "URDFLBRiiwaJoint3", "URDFLBRiiwaJoint4",
      "URDFLBRiiwaJoint5", "URDFLBRiiwaJoint6", "URDFLBRiiwaJoint7"};
    const double * joint_positions_measured =
      robot_state_.getMeasuredJointPosition();
    // TODO(resizoltan) external vs measured torque?

    const double * joint_torques_external = robot_state_.getExternalTorque();
    joint_state_msg_.velocity.clear();
    joint_state_msg_.position.assign(
      joint_positions_measured,
      joint_positions_measured + robot_state_.NUMBER_OF_JOINTS);
    joint_state_msg_.effort.assign(
      joint_torques_external,
      joint_torques_external + robot_state_.NUMBER_OF_JOINTS);
    joint_state_publisher_->publish(joint_state_msg_);
    const double & tracking_performance = robot_state_.getTrackingPerformance();
    std_msgs::msg::Float64 tracking_perf_msg;
    tracking_perf_msg.data = tracking_performance;
    tracking_performance_publisher_->publish(tracking_perf_msg);
  }
  // TODO(resizoltan) double check this:
  for (auto i = std::next(input_publishers_.begin());
    i != input_publishers_.end(); i++)
  {
    (*i)->publishInputValue();
  }
}

}  // namespace kuka_sunrise
