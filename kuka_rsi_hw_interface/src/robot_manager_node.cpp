// Copyright 2023 Svastits Áron
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

#include "kuka_rsi_hw_interface/robot_manager_node.hpp"
#include "kroshu_ros2_core/ControlMode.hpp"


using namespace controller_manager_msgs::srv;  // NOLINT
using namespace lifecycle_msgs::msg;  // NOLINT

namespace kuka_rsi
{
RobotManagerNode::RobotManagerNode()
: kroshu_ros2_core::ROS2BaseLCNode("kss_robot_manager")
{
  auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
  qos.reliable();
  cbg_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  change_hardware_state_client_ =
    this->create_client<SetHardwareComponentState>(
    "controller_manager/set_hardware_component_state", qos.get_rmw_qos_profile(), cbg_
    );
  change_controller_state_client_ =
    this->create_client<SwitchController>(
    "controller_manager/switch_controller", qos.get_rmw_qos_profile(), cbg_
    );

  auto is_configured_qos = rclcpp::QoS(rclcpp::KeepLast(1));
  is_configured_qos.best_effort();

  is_configured_pub_ = this->create_publisher<std_msgs::msg::Bool>(
    "robot_manager/is_configured",
    is_configured_qos);

  this->registerStaticParameter<std::string>(
    "robot_model", "kr6_r700_sixx",
    kroshu_ros2_core::ParameterSetAccessRights{true, false,
      false, false, false}, [this](const std::string & robot_model) {
      return this->onRobotModelChangeRequest(robot_model);
    });
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
RobotManagerNode::on_configure(const rclcpp_lifecycle::State &)
{
  // Configure hardware interface
  auto hw_request =
    std::make_shared<SetHardwareComponentState::Request>();
  hw_request->name = robot_model_;
  hw_request->target_state.id = State::PRIMARY_STATE_INACTIVE;
  auto hw_response =
    kroshu_ros2_core::sendRequest<SetHardwareComponentState::Response>(
    change_hardware_state_client_, hw_request, 0, 2000);
  if (!hw_response || !hw_response->ok) {
    RCLCPP_ERROR(get_logger(), "Could not configure hardware interface");
    return FAILURE;
  }
  RCLCPP_INFO(get_logger(), "Successfully configured hardware interface");

  is_configured_pub_->on_activate();
  is_configured_msg_.data = true;
  is_configured_pub_->publish(is_configured_msg_);
  return SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
RobotManagerNode::on_cleanup(const rclcpp_lifecycle::State &)
{
  // Clean up hardware interface
  auto hw_request =
    std::make_shared<SetHardwareComponentState::Request>();
  hw_request->name = robot_model_;
  hw_request->target_state.id = State::PRIMARY_STATE_UNCONFIGURED;
  auto hw_response =
    kroshu_ros2_core::sendRequest<SetHardwareComponentState::Response>(
    change_hardware_state_client_, hw_request, 0, 2000);
  if (!hw_response || !hw_response->ok) {
    RCLCPP_ERROR(get_logger(), "Could not clean up hardware interface");
    return FAILURE;
  }

  if (is_configured_pub_->is_activated()) {
    is_configured_msg_.data = false;
    is_configured_pub_->publish(is_configured_msg_);
    is_configured_pub_->on_deactivate();
  }
  // TODO(Svastits): add else branch, and throw exception(?)
  return SUCCESS;
}

// TODO(Svastits): rollback in case of failures
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
RobotManagerNode::on_activate(const rclcpp_lifecycle::State &)
{
  // Activate hardware interface
  auto hw_request =
    std::make_shared<SetHardwareComponentState::Request>();
  hw_request->name = robot_model_;
  hw_request->target_state.id = State::PRIMARY_STATE_ACTIVE;
  auto hw_response =
    kroshu_ros2_core::sendRequest<SetHardwareComponentState::Response>(
    change_hardware_state_client_, hw_request, 0, 10000);
  if (!hw_response || !hw_response->ok) {
    RCLCPP_ERROR(get_logger(), "Could not activate hardware interface");
    return FAILURE;
  }


  // Activate RT controller(s)
  auto controller_request =
    std::make_shared<SwitchController::Request>();
  controller_request->strictness = SwitchController::Request::STRICT;
  controller_request->activate_controllers =
  {"joint_state_broadcaster", "joint_trajectory_controller"};

  auto controller_response =
    kroshu_ros2_core::sendRequest<SwitchController::Response>(
    change_controller_state_client_, controller_request, 0, 2000
    );
  if (!controller_response || !controller_response->ok) {
    RCLCPP_ERROR(get_logger(), "Could not  activate controller");
    // TODO(Svastits): this can be removed if rollback is implemented properly
    this->on_deactivate(get_current_state());
    return FAILURE;
  }

  RCLCPP_INFO(get_logger(), "Successfully activated controllers");
  return SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
RobotManagerNode::on_deactivate(const rclcpp_lifecycle::State &)
{
  // Deactivate hardware interface
  auto hw_request =
    std::make_shared<SetHardwareComponentState::Request>();
  hw_request->name = robot_model_;
  hw_request->target_state.id = State::PRIMARY_STATE_INACTIVE;
  auto hw_response =
    kroshu_ros2_core::sendRequest<SetHardwareComponentState::Response>(
    change_hardware_state_client_, hw_request, 0, 3000);   // was not stable with 2000 ms timeout
  if (!hw_response || !hw_response->ok) {
    RCLCPP_ERROR(get_logger(), "Could not deactivate hardware interface");
    return ERROR;
  }

  RCLCPP_INFO(get_logger(), "Deactivated hardware interface");


  // Stop RT controllers
  auto controller_request =
    std::make_shared<SwitchController::Request>();
  // With best effort strictness, deactivation succeeds if specific controller is not active
  controller_request->strictness =
    SwitchController::Request::BEST_EFFORT;
  controller_request->deactivate_controllers =
  {"joint_state_broadcaster", "joint_trajectory_controller"};
  auto controller_response =
    kroshu_ros2_core::sendRequest<SwitchController::Response>(
    change_controller_state_client_, controller_request, 0, 2000
    );
  if (!controller_response || !controller_response->ok) {
    RCLCPP_ERROR(get_logger(), "Could not stop controllers");
    return ERROR;
  }

  RCLCPP_INFO(get_logger(), "Successfully stopped controllers");
  return SUCCESS;
}

bool RobotManagerNode::onRobotModelChangeRequest(const std::string & robot_model)
{
  robot_model_ = robot_model;
  return true;
}
}  // namespace kuka_rsi

int main(int argc, char * argv[])
{
  setvbuf(stdout, nullptr, _IONBF, BUFSIZ);

  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor;
  auto node = std::make_shared<kuka_rsi::RobotManagerNode>();
  executor.add_node(node->get_node_base_interface());
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
