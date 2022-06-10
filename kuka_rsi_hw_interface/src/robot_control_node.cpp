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

#include <string>
#include <memory>
#include <chrono>
#include <thread>

#include "rclcpp/rclcpp.hpp"

#include "kuka_rsi_hw_interface/robot_control_node.hpp"
#include "kroshu_ros2_core/ROS2BaseLCNode.hpp"

namespace kuka_rsi_hw_interface
{
  RobotControlNode::RobotControlNode(
      const std::string &node_name,
      const rclcpp::NodeOptions &options)
      : kroshu_ros2_core::ROS2BaseLCNode(node_name, options)
  {
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1));
    auto callback =
        [this](sensor_msgs::msg::JointState::SharedPtr msg)
    { this->commandReceivedCallback(msg); };

    joint_command_subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "rsi_joint_command", qos, callback);

    joint_state_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("rsi_joint_state", 1);
    joint_command_msg_ = std::make_shared<sensor_msgs::msg::JointState>();
  }

  CallbackReturn RobotControlNode::on_configure(const rclcpp_lifecycle::State &)
  {
    registerParameter<std::string>("rsi_ip_address_", "127.0.0.1",
                                   kroshu_ros2_core::ParameterSetAccessRights{true, true, false, false},
                                   [this](const std::string &rsi_ip_address)
                                   { return this->onRSIIPAddressChange(rsi_ip_address); });

    registerParameter<int>("rsi_port_", 59152,
                           kroshu_ros2_core::ParameterSetAccessRights{true, true, false, false},
                           [this](int rsi_port)
                           { return this->onRSIPortAddressChange(rsi_port); });

    registerParameter<int>("n_dof_", 6,
                           kroshu_ros2_core::ParameterSetAccessRights{true, true, false, false},
                           [this](int n_dof)
                           { return this->onNDOFChange(n_dof); });

    kuka_rsi_hw_interface_ = std::make_unique<KukaHardwareInterface>(rsi_ip_address_, rsi_port_, n_dof_);
    return SUCCESS;
  }
  CallbackReturn RobotControlNode::on_activate(const rclcpp_lifecycle::State &)
  {
    kuka_rsi_hw_interface_->start(joint_state_msg_.position, joint_command_msg_->position, initial_joint_pos_);

    joint_state_publisher_->on_activate();
    joint_state_publisher_->publish(joint_state_msg_);

    control_thread_ = std::thread(&RobotControlNode::ControlLoop, this);
    return SUCCESS;
  }

  CallbackReturn RobotControlNode::on_deactivate(const rclcpp_lifecycle::State &)
  {
    kuka_rsi_hw_interface_->stop(joint_pos_correction_deg_);
    joint_state_publisher_->on_deactivate();
    if (control_thread_.joinable())
    {
      control_thread_.join();
    }

    return SUCCESS;
  }

  CallbackReturn RobotControlNode::on_cleanup(const rclcpp_lifecycle::State &)
  {
    kuka_rsi_hw_interface_ = nullptr;
    return SUCCESS;
  }

  void RobotControlNode::ControlLoop()
  {
    while (kuka_rsi_hw_interface_->isActive())
    {
      std::unique_lock<std::mutex> lock(m_);
      if (!kuka_rsi_hw_interface_->read(joint_state_msg_.position))
      {
        RCLCPP_ERROR(get_logger(), "Failed to read state from robot. Shutting down!");
        rclcpp::shutdown();
        return;
      }
      joint_state_publisher_->publish(joint_state_msg_);

      cv_.wait(lock); // wait for new command -> (Marton) is it really needed?

      for (size_t i = 0; i < n_dof_; i++)
      {
        joint_pos_correction_deg_[i] = (joint_command_msg_->position[i] - initial_joint_pos_[i]) * RobotControlNode::R2D;
      }

      kuka_rsi_hw_interface_->write(joint_pos_correction_deg_);
    }
  }

  void RobotControlNode::commandReceivedCallback(sensor_msgs::msg::JointState::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(m_);
    joint_command_msg_ = msg;
    cv_.notify_one();
  }

  bool RobotControlNode::onRSIIPAddressChange(const std::string &rsi_ip_address)
  {
    rsi_ip_address_ = rsi_ip_address;
    return true;
  }

  bool RobotControlNode::onRSIPortAddressChange(int rsi_port)
  {
    rsi_port_ = rsi_port;
    return true;
  }

  bool RobotControlNode::onNDOFChange(int n_dof)
  {
    n_dof_ = n_dof;
    return true;
  }

} // namespace kuka_rsi_hw_interface

int main(int argc, char *argv[])
{
  setvbuf(stdout, nullptr, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);

  rclcpp::executors::MultiThreadedExecutor executor;
  auto node = std::make_shared<kuka_rsi_hw_interface::RobotControlNode>(
      "robot_control_node",
      rclcpp::NodeOptions());
  executor.add_node(node->get_node_base_interface());
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
