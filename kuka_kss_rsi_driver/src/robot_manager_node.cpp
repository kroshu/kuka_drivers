// Copyright 2023 Aron Svastits
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

#include "communication_helpers/ros2_control_tools.hpp"
#include "communication_helpers/service_tools.hpp"

#include "kuka_drivers_core/controller_names.hpp"
#include "kuka_drivers_core/hardware_event.hpp"
#include "kuka_kss_rsi_driver/robot_manager_node.hpp"

using namespace controller_manager_msgs::srv;  // NOLINT
using namespace lifecycle_msgs::msg;           // NOLINT

namespace kuka_rsi
{
RobotManagerNode::RobotManagerNode() : kuka_drivers_core::ROS2BaseLCNode("robot_manager")
{
  auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
  qos.reliable();
  cbg_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  event_cbg_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  change_hardware_state_client_ = this->create_client<SetHardwareComponentState>(
    "controller_manager/set_hardware_component_state", qos.get_rmw_qos_profile(), cbg_);
  change_controller_state_client_ = this->create_client<SwitchController>(
    "controller_manager/switch_controller", qos.get_rmw_qos_profile(), cbg_);

  auto is_configured_qos = rclcpp::QoS(rclcpp::KeepLast(1));
  is_configured_qos.best_effort();

  is_configured_pub_ =
    this->create_publisher<std_msgs::msg::Bool>("robot_manager/is_configured", is_configured_qos);

  rclcpp::SubscriptionOptions sub_options;
  sub_options.callback_group = event_cbg_;
  event_subscriber_ = this->create_subscription<std_msgs::msg::UInt8>(
    "event_broadcaster/hardware_event", rclcpp::SystemDefaultsQoS(),
    [this](const std_msgs::msg::UInt8::SharedPtr msg) { this->EventSubscriptionCallback(msg); },
    sub_options);

  this->registerStaticParameter<std::string>(
    "robot_model", "kr6_r700_sixx", kuka_drivers_core::ParameterSetAccessRights{false, false},
    [this](const std::string & robot_model)
    { return this->onRobotModelChangeRequest(robot_model); });
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
RobotManagerNode::on_configure(const rclcpp_lifecycle::State &)
{
  // Configure hardware interface
  if (!kuka_drivers_core::changeHardwareState(
        change_hardware_state_client_, robot_model_, State::PRIMARY_STATE_INACTIVE))
  {
    RCLCPP_ERROR(get_logger(), "Could not configure hardware interface");
    return FAILURE;
  }

  // Activate control mode handler and event broadcaster
  if (!kuka_drivers_core::changeControllerState(
        change_controller_state_client_, {kuka_drivers_core::EVENT_BROADCASTER}, {}))
  {
    RCLCPP_ERROR(get_logger(), "Could not activate event broadcaster");
    return FAILURE;
  }

  is_configured_pub_->on_activate();
  is_configured_msg_.data = true;
  is_configured_pub_->publish(is_configured_msg_);
  return SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
RobotManagerNode::on_cleanup(const rclcpp_lifecycle::State &)
{
  // Deactivate event broadcaster
  if (!kuka_drivers_core::changeControllerState(
        change_controller_state_client_, {}, {kuka_drivers_core::EVENT_BROADCASTER}))
  {
    RCLCPP_ERROR(get_logger(), "Could not deactivate event broadcaster");
  }

  // Clean up hardware interface
  if (!kuka_drivers_core::changeHardwareState(
        change_hardware_state_client_, robot_model_, State::PRIMARY_STATE_UNCONFIGURED))
  {
    RCLCPP_ERROR(get_logger(), "Could not clean up hardware interface");
    return FAILURE;
  }

  if (is_configured_pub_->is_activated())
  {
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
  if (!kuka_drivers_core::changeHardwareState(
        change_hardware_state_client_, robot_model_, State::PRIMARY_STATE_ACTIVE, 10000))
  {
    RCLCPP_ERROR(get_logger(), "Could not activate hardware interface");
    return FAILURE;
  }

  // Activate RT controller(s)
  if (!kuka_drivers_core::changeControllerState(
        change_controller_state_client_,
        {kuka_drivers_core::JOINT_STATE_BROADCASTER,
         kuka_drivers_core::JOINT_TRAJECTORY_CONTROLLER},
        {}))
  {
    RCLCPP_ERROR(get_logger(), "Could not activate RT controllers");
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
  if (!kuka_drivers_core::changeHardwareState(
        change_hardware_state_client_, robot_model_, State::PRIMARY_STATE_INACTIVE))
  {
    RCLCPP_ERROR(get_logger(), "Could not deactivate hardware interface");
    return ERROR;
  }

  // Stop RT controllers
  // With best effort strictness, deactivation succeeds if specific controller is not active
  if (!kuka_drivers_core::changeControllerState(
        change_controller_state_client_, {},
        {kuka_drivers_core::JOINT_STATE_BROADCASTER,
         kuka_drivers_core::JOINT_TRAJECTORY_CONTROLLER},
        SwitchController::Request::BEST_EFFORT))
  {
    RCLCPP_ERROR(get_logger(), "Could not stop controllers");
    return ERROR;
  }

  RCLCPP_INFO(get_logger(), "Successfully stopped controllers");
  return SUCCESS;
}

bool RobotManagerNode::onRobotModelChangeRequest(const std::string & robot_model)
{
  auto ns = std::string(this->get_namespace());
  // Remove '/' from namespace (even empty namespace contains one '/')
  ns.erase(ns.begin());

  // Add '_' to prefix
  if (ns.size() > 0)
  {
    ns += "_";
  }
  robot_model_ = ns + robot_model;
  return true;
}

void RobotManagerNode::EventSubscriptionCallback(const std_msgs::msg::UInt8::SharedPtr msg)
{
  switch (static_cast<kuka_drivers_core::HardwareEvent>(msg->data))
  {
    case kuka_drivers_core::HardwareEvent::ERROR:
      RCLCPP_INFO(get_logger(), "External control stopped");
      if (this->get_current_state().id() == State::PRIMARY_STATE_ACTIVE)
      {
        this->deactivate();
      }
      else if (this->get_current_state().id() == State::TRANSITION_STATE_ACTIVATING)
      {
        // Handle case, when error is received while still activating
        this->on_deactivate(get_current_state());
      }
      break;
    default:
      break;
  }
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
