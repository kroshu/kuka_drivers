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

#ifndef CONTROL_SYSTEM__SYSTEM_MANAGER_HPP_
#define CONTROL_SYSTEM__SYSTEM_MANAGER_HPP_

#include <vector>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "kuka_sunrise_interfaces/srv/get_state.hpp"
#include "kuka_sunrise/internal/service_tools.hpp"

#include "kroshu_ros2_core/ROS2BaseLCNode.hpp"

namespace control_system
{
class SystemManager : public kroshu_ros2_core::ROS2BaseLCNode
{
public:
  SystemManager(const std::string & node_name, const rclcpp::NodeOptions & options);

  void activateControl();
  void deactivateControl();

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State &) final;
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State &) final;
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_shutdown(const rclcpp_lifecycle::State &) final;
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &) final;
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &) final;

private:
  bool changeState(const std::string & node_name, std::uint8_t transition);
  bool changeRobotCommandingState(bool is_active);
  void robotCommandingStateChanged(bool is_active);
  void getFRIState();
  lifecycle_msgs::msg::State getState(
    const std::string & node_name);
  void monitoringLoop();
  bool robot_control_active_ = false;
  bool stop_ = false;
  int lbr_state_ = 0;
  const std::chrono::milliseconds sleeping_time_ms_ = std::chrono::milliseconds(
    200);
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_;

  std::thread polling_thread_;

  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::JointState>::SharedPtr
    reference_joint_state_publisher_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr robot_commanding_state_subscription_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Bool>::SharedPtr manage_processing_publisher_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr change_robot_manager_state_client_;
  rclcpp::Client<kuka_sunrise_interfaces::srv::GetState>::SharedPtr get_state_client_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr trigger_change_service_;
  rclcpp::CallbackGroup::SharedPtr cbg_;
  rclcpp::QoS qos_ = rclcpp::QoS(rclcpp::KeepLast(10));

  const std::string JOINT_CONTROLLER = "joint_controller";
  const std::string ROBOT_INTERFACE = "robot_manager";
  const std::string CONTROL_LOGIC = "keyboard_control";
  bool control_logic_;
};
}  // namespace control_system

#endif  // CONTROL_SYSTEM__SYSTEM_MANAGER_HPP_
