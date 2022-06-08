// Copyright 2022 Aron Svastits
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

#ifndef KUKA_RSI_HW_INTERFACE__ROBOT_CONTROL_NODE_HPP_
#define KUKA_RSI_HW_INTERFACE__ROBOT_CONTROL_NODE_HPP_

#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "kuka_rsi_hw_interface/kuka_hardware_interface.hpp"
#include "kroshu_ros2_core/ROS2BaseLCNode.hpp"
#include "sensor_msgs/msg/joint_state.hpp"


namespace kuka_rsi_hw_interface
{

class RobotControlNode : public kroshu_ros2_core::ROS2BaseLCNode
{
public:
  RobotControlNode(const std::string & node_name, const rclcpp::NodeOptions & options);

private:
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State &) override;
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State &) override;
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &) override;
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &) override;

  std::unique_ptr<KukaHardwareInterface> kuka_rsi_hw_interface_;

  void commandReceivedCallback(sensor_msgs::msg::JointState::ConstSharedPtr msg);
  void Controlloop();

  std::thread control_thread_;
};

}  // namespace kuka_rsi_hw_interface
#endif  // KUKA_RSI_HW_INTERFACE__ROBOT_CONTROL_NODE_HPP_
