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
  const std::string & node_name,
  const rclcpp::NodeOptions & options)
: kroshu_ros2_core::ROS2BaseLCNode(node_name, options)
{}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn RobotControlNode::
on_configure(
  const rclcpp_lifecycle::State &)
{
  kuka_rsi_hw_interface_ = std::make_unique<KukaHardwareInterface>(this->shared_from_this());
  kuka_rsi_hw_interface_->configure();
  return SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn RobotControlNode::
on_activate(
  const rclcpp_lifecycle::State &)
{
  kuka_rsi_hw_interface_->start();

  control_thread_ = std::thread(&RobotControlNode::Controlloop, this);
  return SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn RobotControlNode::
on_deactivate(
  const rclcpp_lifecycle::State &)
{
  kuka_rsi_hw_interface_->stop();
  if (control_thread_.joinable()) {control_thread_.join();}

  return SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn RobotControlNode::
on_cleanup(
  const rclcpp_lifecycle::State &)
{
  kuka_rsi_hw_interface_->cleanup();
  return SUCCESS;
}

void RobotControlNode::Controlloop()
{
  while (this->get_current_state().label() == "active") {
    if (!kuka_rsi_hw_interface_->read()) {
      RCLCPP_ERROR(get_logger(), "Failed to read state from robot. Shutting down!");
      rclcpp::shutdown();
      return;
    }
    kuka_rsi_hw_interface_->write();
  }
}
}  // namespace kuka_rsi_hw_interface


int main(int argc, char * argv[])
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
