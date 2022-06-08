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
#include <stdexcept>
#include <string>

#include "kuka_rsi_hw_interface/kuka_hardware_interface.hpp"


namespace kuka_rsi_hw_interface
{

KukaHardwareInterface::KukaHardwareInterface(
  rclcpp_lifecycle::LifecycleNode::SharedPtr robot_control_node)
: control_node_(robot_control_node)
{
  in_buffer_.resize(1024);
  out_buffer_.resize(1024);

  auto qos = rclcpp::QoS(rclcpp::KeepLast(1));
  auto callback =
    [this](sensor_msgs::msg::JointState::SharedPtr msg)
    {this->commandReceivedCallback(msg);};

  cbg_ = control_node_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::SubscriptionOptions options;
  options.callback_group = cbg_;
  joint_command_subscription_ = control_node_->create_subscription<
    sensor_msgs::msg::JointState>(
    "rsi_joint_command", qos, callback, options);

  joint_state_publisher_ = control_node_->create_publisher<
    sensor_msgs::msg::JointState>("rsi_joint_state", 1);
  joint_command_msg_ = std::make_shared<sensor_msgs::msg::JointState>();
}

void KukaHardwareInterface::commandReceivedCallback(sensor_msgs::msg::JointState::SharedPtr msg)
{
  RCLCPP_INFO(control_node_->get_logger(), "got cmd");
  std::lock_guard<std::mutex> lock(m_);
  joint_command_msg_ = msg;
  for (size_t i = 0; i < n_dof_; i++) {
    RCLCPP_ERROR(control_node_->get_logger(), "command%i: %lf", i, joint_command_msg_->position[i]);
  }
  cv_.notify_one();
}

bool KukaHardwareInterface::read()
{
  RCLCPP_INFO(control_node_->get_logger(), "read function");
  in_buffer_.resize(1024);

  if (server_->recv(in_buffer_) == 0) {
    return false;
  }

  RCLCPP_INFO(control_node_->get_logger(), "got msg");

  rsi_state_ = RSIState(in_buffer_);
  for (std::size_t i = 0; i < n_dof_; ++i) {
    joint_state_msg_.position[i] = rsi_state_.positions[i] * KukaHardwareInterface::D2R;
  }
  joint_state_publisher_->publish(joint_state_msg_);
  ipoc_ = rsi_state_.ipoc;

  return true;
}

void KukaHardwareInterface::write()
{
  RCLCPP_INFO(control_node_->get_logger(), "write func");
  out_buffer_.resize(1024);  //TODO(Svastits): is this necessary?
  std::unique_lock<std::mutex> lock(m_);
  //cv_.wait(lock);
  if (!is_active_) {
    RCLCPP_INFO(control_node_->get_logger(), "Controller deactivated");
    return;
  }
  for (size_t i = 0; i < n_dof_; i++) {
      joint_pos_correction_deg_[i] = (joint_command_msg_->position[i] - initial_joint_pos_[i]) * KukaHardwareInterface::R2D;
    //RCLCPP_INFO(control_node_->get_logger(), "command%i: %lf", i, joint_pos_correction_deg_);
  }
  out_buffer_ = RSICommand(joint_pos_correction_deg_, ipoc_).xml_doc;
  server_->send(out_buffer_);
}

void KukaHardwareInterface::start()
{
  // Wait for connection from robot
  server_.reset(new UDPServer(local_host_, local_port_));

  RCLCPP_INFO(control_node_->get_logger(), "Waiting for robot!");

  int bytes = server_->recv(in_buffer_);

  // Drop empty <rob> frame with RSI <= 2.3
  if (bytes < 100) {
    bytes = server_->recv(in_buffer_);
  }

  rsi_state_ = RSIState(in_buffer_);
  for (std::size_t i = 0; i < n_dof_; ++i) {
    joint_state_msg_.position[i] = rsi_state_.positions[i] * KukaHardwareInterface::D2R;
    joint_command_msg_->position[i] = joint_state_msg_.position[i];
    initial_joint_pos_[i] = rsi_state_.initial_positions[i] * KukaHardwareInterface::D2R;
    //RCLCPP_INFO(control_node_->get_logger(), "initial%i: %lf", i, initial_joint_pos_[i]);
    //RCLCPP_INFO(control_node_->get_logger(), "actual%i: %lf", i, joint_state_msg_.position[i]);
  }
  ipoc_ = rsi_state_.ipoc;
  out_buffer_ = RSICommand(joint_pos_correction_deg_, ipoc_).xml_doc;
  server_->send(out_buffer_);
  // Set receive timeout to 1 second
  server_->set_timeout(1000);
  RCLCPP_INFO(control_node_->get_logger(), "Got connection from robot");
  is_active_ = true;
  joint_state_publisher_->on_activate();
  joint_state_publisher_->publish(joint_state_msg_);
}

void KukaHardwareInterface::stop()
{
  std::lock_guard<std::mutex> lock(m_);
  out_buffer_ = RSICommand(joint_pos_correction_deg_, ipoc_, true).xml_doc;
  server_->send(out_buffer_);
  joint_state_publisher_->on_deactivate();
  server_.reset();
  is_active_ = false;
  RCLCPP_INFO(control_node_->get_logger(), "Connection to robot terminated");
  //cv_.notify_one();
}

void KukaHardwareInterface::configure()
{
  // TODO(Svastits): ip and port parameters
  local_host_ = "172.32.20.20";
  local_port_ = 59152;

  joint_state_msg_.position.resize(n_dof_);
  joint_command_msg_->position.resize(n_dof_);
}

void KukaHardwareInterface::cleanup()
{
  local_host_ = "";
  local_port_ = 0;
  const std::string param_port = "";
}

}  // namespace namespace kuka_rsi_hw_interface
