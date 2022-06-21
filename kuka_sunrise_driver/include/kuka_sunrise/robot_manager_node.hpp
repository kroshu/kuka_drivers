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

#ifndef KUKA_SUNRISE__ROBOT_MANAGER_NODE_HPP_
#define KUKA_SUNRISE__ROBOT_MANAGER_NODE_HPP_

#include <atomic>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/client.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "std_msgs/msg/bool.hpp"

#include "kuka_sunrise/robot_manager.hpp"
#include "kuka_sunrise/configuration_manager.hpp"
#include "kuka_sunrise/internal/activatable_interface.hpp"

#include "kroshu_ros2_core/ROS2BaseLCNode.hpp"

namespace kuka_sunrise
{

class RobotManagerNode : public kroshu_ros2_core::ROS2BaseLCNode, public ActivatableInterface
{
public:
  RobotManagerNode();

  virtual rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State &);

  virtual rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State &);

  virtual rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_shutdown(const rclcpp_lifecycle::State &);

  virtual rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &);

  virtual rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &);

  bool activate();
  bool deactivate();

private:
  std::shared_ptr<RobotManager> robot_manager_;
  std::unique_ptr<ConfigurationManager> configuration_manager_;
  rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr change_robot_control_state_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr set_parameter_client_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr set_commanding_state_client_;
  rclcpp::CallbackGroup::SharedPtr cbg_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr change_robot_commanding_state_service_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Bool>::SharedPtr
    command_state_changed_publisher_;

  bool requestRobotControlNodeStateTransition(std::uint8_t transition);
  bool setRobotControlNodeCommandState(bool active);
  void handleControlEndedError();
  void handleFRIEndedError();
};

}  // namespace kuka_sunrise

#endif  // KUKA_SUNRISE__ROBOT_MANAGER_NODE_HPP_
