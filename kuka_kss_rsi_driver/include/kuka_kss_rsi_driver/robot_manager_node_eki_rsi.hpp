// Copyright 2025 Kristof Pasztor
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

#ifndef KUKA_KSS_RSI_DRIVER__ROBOT_MANAGER_NODE_EKI_RSI_HPP_
#define KUKA_KSS_RSI_DRIVER__ROBOT_MANAGER_NODE_EKI_RSI_HPP_

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "controller_manager_msgs/srv/set_hardware_component_state.hpp"
#include "controller_manager_msgs/srv/switch_controller.hpp"
#include "rclcpp/client.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/u_int32.hpp"
#include "std_msgs/msg/u_int8.hpp"

#include "kuka_drivers_core/controller_handler.hpp"
#include "kuka_drivers_core/ros2_base_lc_node.hpp"

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace kuka_rsi
{
class RobotManagerNode : public kuka_drivers_core::ROS2BaseLCNode
{
public:
  RobotManagerNode();
  ~RobotManagerNode() = default;

  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;

  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override;

  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;

private:
  void EventSubscriptionCallback(const std_msgs::msg::UInt8::SharedPtr message);

  bool OnControlModeChangeRequest(const int control_mode);

  bool OnRobotModelChangeRequest(const std::string & robot_model);

  rclcpp::Client<controller_manager_msgs::srv::SetHardwareComponentState>::SharedPtr
    change_hardware_state_client_;
  rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr
    change_controller_state_client_;
  rclcpp::CallbackGroup::SharedPtr cbg_;
  std::string robot_model_;

  kuka_drivers_core::ControllerHandler controller_handler_;
  kuka_drivers_core::ControlMode control_mode_ =
    kuka_drivers_core::ControlMode::CONTROL_MODE_UNSPECIFIED;

  std::atomic<bool> terminate_{false};

  std::condition_variable control_mode_cv_;
  std::mutex control_mode_cv_m_;
  bool control_mode_change_finished_ = false;
  rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr control_mode_pub_;

  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Bool>> is_configured_pub_;
  std_msgs::msg::Bool is_configured_msg_;

  rclcpp::CallbackGroup::SharedPtr event_callback_group_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr event_subscriber_;
};
}  // namespace kuka_rsi

#endif  // KUKA_KSS_RSI_DRIVER__ROBOT_MANAGER_NODE_EKI_RSI_HPP_
