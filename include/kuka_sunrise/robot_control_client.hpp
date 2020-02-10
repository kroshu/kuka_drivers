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

#ifndef KUKA_SUNRISE__ROBOT_CONTROL_CLIENT_HPP_
#define KUKA_SUNRISE__ROBOT_CONTROL_CLIENT_HPP_

#include <condition_variable>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "kuka_sunrise_interfaces/srv/set_int.hpp"

#include "kuka_sunrise/internal/activatable_interface.hpp"
#include "fri_client/friLBRClient.h"



namespace kuka_sunrise
{

class RobotObserver;
class RobotCommander;

class RobotControlClient : public KUKA::FRI::LBRClient, public ActivatableInterface
{
public:
  explicit RobotControlClient(rclcpp_lifecycle::LifecycleNode::SharedPtr robot_control_node_);
  ~RobotControlClient();
  bool activate();
  bool deactivate();
  bool setReceiveMultiplier(int receive_multiplier);

  virtual void monitor();
  virtual void waitForCommand();
  virtual void command();

private:
  std::unique_ptr<RobotObserver> robot_observer_;
  std::unique_ptr<RobotCommander> robot_commander_;

  rclcpp_lifecycle::LifecycleNode::SharedPtr robot_control_node_;
  rclcpp::Service<kuka_sunrise_interfaces::srv::SetInt>::SharedPtr set_receive_multiplier_service_;
  rclcpp::Clock ros_clock_;
  int receive_multiplier_;
  int receive_counter_;
};

}  // namespace kuka_sunrise

#endif  // KUKA_SUNRISE__ROBOT_CONTROL_CLIENT_HPP_
