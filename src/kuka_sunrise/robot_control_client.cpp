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

#include "kuka_sunrise/robot_control_client.hpp"
#include "kuka_sunrise/robot_commander.hpp"
#include "kuka_sunrise/robot_observer.hpp"

namespace kuka_sunrise
{

RobotControlClient::RobotControlClient(
  rclcpp_lifecycle::LifecycleNode::SharedPtr robot_control_node)
: robot_control_node_(robot_control_node), receive_multiplier_(1), receive_counter_(0)
{
  robot_observer_ = std::make_unique<RobotObserver>(robotState(), robot_control_node);
  robot_commander_ = std::make_unique<RobotCommander>(
    robotCommand(), robotState(),
    robot_control_node);
  auto command_srv_callback = [this](
    const std::shared_ptr<rmw_request_id_t> request_header,
    kuka_sunrise_interfaces::srv::SetInt::Request::SharedPtr request,
    kuka_sunrise_interfaces::srv::SetInt::Response::SharedPtr response) {
      (void)request_header;
      if (this->setReceiveMultiplier(request->data)) {
        response->success = true;
      } else {
        response->success = false;
      }
    };
  set_receive_multiplier_service_ = robot_control_node_->create_service<
    kuka_sunrise_interfaces::srv::SetInt>("set_receive_multiplier", command_srv_callback);
}

RobotControlClient::~RobotControlClient()
{
  robot_commander_->deactivate();
}

bool RobotControlClient::activate()
{
  this->ActivatableInterface::activate();
  robot_commander_->activate();
  robot_observer_->activate();
  return true;  // TODO(resizoltan) check if successful
}

bool RobotControlClient::deactivate()
{
  this->ActivatableInterface::activate();
  robot_commander_->deactivate();
  robot_observer_->activate();
  return true;  // TODO(resizoltan) check if successful
}

void RobotControlClient::monitor()
{
  rclcpp::Time stamp = ros_clock_.now();
  robot_observer_->publishRobotState(stamp);
}

void RobotControlClient::waitForCommand()
{
  rclcpp::Time stamp = ros_clock_.now();
  robot_observer_->publishRobotState(stamp);
  if (++receive_counter_ == receive_multiplier_) {
    robot_commander_->updateCommand(stamp);
    receive_counter_ = 0;
  }
  // RCLCPP_INFO(robot_control_node_->get_logger(), "waitforcommand finished");
}

void RobotControlClient::command()
{
  rclcpp::Time stamp = ros_clock_.now();
  robot_observer_->publishRobotState(stamp);
  if (++receive_counter_ == receive_multiplier_) {
    robot_commander_->updateCommand(stamp);
    receive_counter_ = 0;
  }
  // RCLCPP_INFO(robot_control_node_->get_logger(), "command finished");
}

bool RobotControlClient::setReceiveMultiplier(int receive_multiplier)
{
  if (robot_control_node_->get_current_state().label() == "unconfigured") {
    receive_multiplier_ = receive_multiplier;
    return true;
  } else {
    return false;
  }
}

}  // namespace kuka_sunrise
