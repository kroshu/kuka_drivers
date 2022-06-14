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

#ifndef CONTROL_LOGIC__KEYBOARD_CONTROL_HPP_
#define CONTROL_LOGIC__KEYBOARD_CONTROL_HPP_

#include <string>
#include <vector>
#include <map>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "kroshu_ros2_core/ROS2BaseLCNode.hpp"

namespace teleop_guided_robot
{
class KeyboardControl : public kroshu_ros2_core::ROS2BaseLCNode
{
public:
  KeyboardControl(const std::string & node_name, const rclcpp::NodeOptions & options);

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State &) final;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &) final;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &) final;

private:
  void messageReceivedCallback(geometry_msgs::msg::Twist::SharedPtr msg);
  bool onLowerLimitsChangeRequest(const std::vector<double> & lower_limits);
  bool onUpperLimitsChangeRequest(const std::vector<double> & upper_limits);

  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::JointState>::SharedPtr
    reference_joint_state_publisher_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr key_teleop_subscription_;
  sensor_msgs::msg::JointState reference_joint_state_;
  const rclcpp::Duration elapsed_time_treshold_ = rclcpp::Duration(100000000);
  rclcpp::Time last_time_ = rclcpp::Time(RCL_ROS_TIME);

  std::vector<double> lower_limits_rad_ = std::vector<double>(7);
  std::vector<double> upper_limits_rad_ = std::vector<double>(7);

  int active_joint_;
  bool changing_joint_ = false;
  const double turning_velocity_increment_ = 3.0 * M_PI / 180;

  const struct
  {
    double X_POS = 1.25;
    double X_NEG = 2;
    double Z = 1;
  } WEIGHTS;
};

}  // namespace teleop_guided_robot

#endif  // CONTROL_LOGIC__KEYBOARD_CONTROL_HPP_
