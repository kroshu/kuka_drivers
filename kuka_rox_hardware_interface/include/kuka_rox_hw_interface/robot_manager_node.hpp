// Copyright 2022 Komáromi Sándor
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

#ifndef KUKA_ROX_HW_INTERFACE__ROBOT_MANAGER_NODE_HPP_
#define KUKA_ROX_HW_INTERFACE__ROBOT_MANAGER_NODE_HPP_

#include <string>
#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/client.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "controller_manager_msgs/srv/set_hardware_component_state.hpp"
#include "controller_manager_msgs/srv/switch_controller.hpp"
#include "std_msgs/msg/bool.hpp"

#include "kuka_sunrise/internal/service_tools.hpp"

#include "kroshu_ros2_core/ROS2BaseLCNode.hpp"

namespace kuka_rox
{

class RobotManagerNode : public kroshu_ros2_core::ROS2BaseLCNode
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

private:
  rclcpp::Client<controller_manager_msgs::srv::SetHardwareComponentState>::SharedPtr
    change_hardware_state_client_;
  rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr
    change_controller_state_client_;
  rclcpp::CallbackGroup::SharedPtr cbg_;
  std::vector<std::string> controller_names_;

  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Bool>> is_configured_pub_;
  std_msgs::msg::Bool is_configured_msg_;

  static constexpr bool is_joint_imp_contr_ = true;
};

}  // namespace kuka_rox


#endif  // KUKA_ROX_HW_INTERFACE__ROBOT_MANAGER_NODE_HPP_
