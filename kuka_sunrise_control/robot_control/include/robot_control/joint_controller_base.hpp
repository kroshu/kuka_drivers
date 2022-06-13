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

#ifndef ROBOT_CONTROL__JOINT_CONTROLLER_BASE_HPP_
#define ROBOT_CONTROL__JOINT_CONTROLLER_BASE_HPP_

#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "kuka_sunrise_interfaces/srv/set_double.hpp"
#include "kuka_sunrise_interfaces/srv/set_int.hpp"

#include "kroshu_ros2_core/ROS2BaseLCNode.hpp"

namespace robot_control
{
class JointControllerBase : public kroshu_ros2_core::ROS2BaseLCNode
{
public:
  JointControllerBase(
    const std::string & node_name,
    const rclcpp::NodeOptions & options);

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State &) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State &) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &) override;

protected:
  uint8_t n_dof_ = 7;
  void jointStateMeasurementsCallback(sensor_msgs::msg::JointState::SharedPtr measured_joint_state);
  virtual void controlLoopCallback(
    sensor_msgs::msg::JointState::SharedPtr measured_joint_state) = 0;
  void updateMaxPositionDifference();
  bool onVelocityFactorsChangeRequest(const std::vector<double> & vel_factor);

  const std::vector<double> & maxPosDiff() const;
  const std::vector<double> & lowerLimitsRad() const;
  const std::vector<double> & upperLimitsRad() const;
  const int & loopPeriod() const;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::JointState>::SharedPtr
  jointCommandPublisher()
  const;

  sensor_msgs::msg::JointState joint_command_;

  static constexpr int ms_in_sec_ = 1000;

private:
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::JointState>::SharedPtr
    joint_command_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr measured_joint_state_listener_;
  rclcpp::Service<kuka_sunrise_interfaces::srv::SetInt>::SharedPtr sync_send_period_service_;
  rclcpp::Service<kuka_sunrise_interfaces::srv::SetInt>::SharedPtr sync_receive_multiplier_service_;

  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Bool>::SharedPtr
    joint_controller_is_active_publisher_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_;
  std_msgs::msg::Bool joint_controller_is_active_;

  std::vector<double> max_velocities_radPs_ = std::vector<double>(n_dof_);
  std::vector<double> velocity_limits_radPs_ = std::vector<double>(n_dof_);
  std::vector<double> lower_limits_rad_ = std::vector<double>(n_dof_);
  std::vector<double> upper_limits_rad_ = std::vector<double>(n_dof_);
  std::vector<double> max_position_difference_ = std::vector<double>(n_dof_);

  int send_period_ms_ = 10;
  int receive_multiplier_ = 1;
  int loop_count_ = 0;
  int loop_period_ms_ = 10;
};
}  // namespace robot_control


#endif  // ROBOT_CONTROL__JOINT_CONTROLLER_BASE_HPP_
