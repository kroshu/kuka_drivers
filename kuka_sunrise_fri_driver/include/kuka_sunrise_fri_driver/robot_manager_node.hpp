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

#ifndef KUKA_SUNRISE_FRI_DRIVER__ROBOT_MANAGER_NODE_HPP_
#define KUKA_SUNRISE_FRI_DRIVER__ROBOT_MANAGER_NODE_HPP_

#include <atomic>
#include <memory>
#include <string_view>
#include <vector>

#include "controller_manager_msgs/srv/set_hardware_component_state.hpp"
#include "controller_manager_msgs/srv/switch_controller.hpp"
#include "rclcpp/client.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/u_int32.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "std_srvs/srv/set_bool.hpp"

#include "kuka_driver_interfaces/msg/fri_configuration.hpp"
#include "kuka_drivers_core/control_mode.hpp"
#include "kuka_drivers_core/ros2_base_lc_node.hpp"

#include "kuka_sunrise_fri_driver/fri_connection.hpp"

namespace kuka_sunrise_fri_driver
{

class RobotManagerNode : public kuka_drivers_core::ROS2BaseLCNode
{
public:
  RobotManagerNode();

  virtual rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State &);

  virtual rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State &);

  virtual rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State &);

  virtual rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State &);

private:
  std::shared_ptr<FRIConnection> fri_connection_;
  rclcpp::Client<controller_manager_msgs::srv::SetHardwareComponentState>::SharedPtr
    change_hardware_state_client_;
  rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr
    change_controller_state_client_;
  rclcpp::CallbackGroup::SharedPtr cbg_;
  rclcpp::CallbackGroup::SharedPtr event_cbg_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Bool>> is_configured_pub_;
  std_msgs::msg::Bool is_configured_msg_;
  rclcpp::Publisher<kuka_driver_interfaces::msg::FriConfiguration>::SharedPtr fri_config_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr control_mode_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr joint_imp_pub_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr event_subscriber_;
  std_msgs::msg::UInt32 control_mode_msg_;

  int receive_multiplier_ = 0;
  int send_period_ms_ = 0;
  std::string robot_model_;
  std::string joint_pos_controller_name_;
  std::string joint_torque_controller_name_;
  std::vector<double> joint_stiffness_ = std::vector<double>(7, 100.0);
  std::vector<double> joint_damping_ = std::vector<double>(7, 0.7);

  std::string GetControllerName() const;
  bool onControlModeChangeRequest(int control_mode);
  bool onRobotModelChangeRequest(const std::string & robot_model);
  bool onSendPeriodChangeRequest(int send_period);
  bool setReceiveMultiplier(int receive_multiplier);
  bool onReceiveMultiplierChangeRequest(const int & receive_multiplier);
  bool ValidateIPAdress(std::string_view controller_ip) const;
  bool onControllerNameChangeRequest(
    std::string_view controller_name, kuka_drivers_core::ControllerType controller_type);
  bool onJointDampingChangeRequest(const std::vector<double> & joint_damping);
  bool onJointStiffnessChangeRequest(const std::vector<double> & joint_stiffness);
  void setFriConfiguration(int send_period_ms, int receive_multiplier) const;

  void EventSubscriptionCallback(const std_msgs::msg::UInt8::SharedPtr msg);
};

}  // namespace kuka_sunrise_fri_driver

#endif  // KUKA_SUNRISE_FRI_DRIVER__ROBOT_MANAGER_NODE_HPP_
