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
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "kuka_rsi_hw_interface/kuka_hardware_interface.hpp"
#include "kroshu_ros2_core/ROS2BaseLCNode.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#define DEFAULT_N_DOF 6

namespace kuka_rsi_hw_interface
{
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class RobotControlNode : public kroshu_ros2_core::ROS2BaseLCNode
{
public:
RobotControlNode(const std::string & node_name, const rclcpp::NodeOptions & options);

private:
CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
CallbackReturn on_cleanup(const rclcpp_lifecycle::State& previous_state) override;
CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

std::unique_ptr<KukaRSIHardwareInterface> kuka_rsi_hw_interface_;

void commandReceivedCallback(sensor_msgs::msg::JointState::SharedPtr msg);
bool onRSIIPAddressChange(const std::string & rsi_ip_address);
bool onRSIPortAddressChange(int rsi_port);
bool onNDOFChange(uint8_t n_dof);

void ControlLoop();

std::thread control_thread_;

std::string rsi_ip_address_ = "";
int rsi_port_ = 0;
uint8_t n_dof_ = DEFAULT_N_DOF;
std::vector<std::string> controller_joint_names_;

rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::JointState>::SharedPtr
        joint_state_publisher_;
rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_command_subscription_;
rclcpp::CallbackGroup::SharedPtr cbg_;
sensor_msgs::msg::JointState::SharedPtr joint_command_msg_;
sensor_msgs::msg::JointState joint_state_msg_;
std::mutex m_;
std::condition_variable cv_;
};

}  // namespace kuka_rsi_hw_interface
#endif  // KUKA_RSI_HW_INTERFACE__ROBOT_CONTROL_NODE_HPP_
