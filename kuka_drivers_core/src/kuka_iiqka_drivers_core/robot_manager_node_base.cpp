// Copyright 2022 Komáromi Sándor
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

#include <grpcpp/create_channel.h>

#include "communication_helpers/ros2_control_tools.hpp"
//#include "communication_helpers/service_tools.hpp"

#include "kuka_iiqka_drivers_core/robot_manager_node_base.hpp"

#include "kuka_drivers_core/control_mode.hpp"
#include "kuka_drivers_core/controller_names.hpp"
#include "kuka_drivers_core/hardware_event.hpp"

using namespace controller_manager_msgs::srv;  // NOLINT
using namespace lifecycle_msgs::msg;           // NOLINT

namespace kuka_eac
{
RobotManagerNodeBase::RobotManagerNodeBase() : kuka_drivers_core::ROS2BaseLCNode("robot_manager")
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

  control_mode_pub_ = this->create_publisher<std_msgs::msg::UInt32>(
    "control_mode_handler/control_mode", rclcpp::SystemDefaultsQoS());

  rclcpp::SubscriptionOptions sub_options;
  sub_options.callback_group = event_cbg_;

  event_subscriber_ = this->create_subscription<std_msgs::msg::UInt8>(
    "kuka_event_broadcaster/hardware_event", rclcpp::SystemDefaultsQoS(),
    [this](const std_msgs::msg::UInt8::SharedPtr msg) { this->EventSubscriptionCallback(msg); },
    sub_options);

  
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
RobotManagerNodeBase::on_configure(const rclcpp_lifecycle::State &)
{
  // Publish control mode parameter to notify kuka_control_mode_handler of initial control mode
  auto message = std_msgs::msg::UInt32();
  message.data = static_cast<int>(control_mode_);
  control_mode_pub_->publish(message);

  // Configure hardware interface
  if (!kuka_drivers_core::changeHardwareState(
        change_hardware_state_client_, robot_model_, State::PRIMARY_STATE_INACTIVE))
  {
    RCLCPP_ERROR(get_logger(), "Could not configure hardware interface");
    return FAILURE;
  }

  // Publish message about HWIF configuration
  // TODO(Svastits): this can be removed in the future, if all drivers do simple blocking waits in
  // read with msg_received_ flag
  is_configured_msg_.data = true;
  is_configured_pub_->publish(is_configured_msg_);

  // Activate control mode handler and event broadcaster
  if (!kuka_drivers_core::changeControllerState(
        change_controller_state_client_,
        {kuka_drivers_core::CONTROL_MODE_HANDLER, kuka_drivers_core::EVENT_BROADCASTER}, {}))
  {
    RCLCPP_ERROR(get_logger(), "Could not activate control mode handler or event broadcaster");
    // Rollback
    this->on_cleanup(get_current_state());
    return FAILURE;
  }

  RCLCPP_INFO(get_logger(), "Activated control mode handler and event broadcaster");

  return SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
RobotManagerNodeBase::on_cleanup(const rclcpp_lifecycle::State &)
{
  // Deactivate control mode handler and event broadcaster
  if (!kuka_drivers_core::changeControllerState(
        change_controller_state_client_, {},
        {kuka_drivers_core::CONTROL_MODE_HANDLER, kuka_drivers_core::EVENT_BROADCASTER}))
  {
    RCLCPP_ERROR(get_logger(), "Could not deactivate control mode handler and event broadcaster");
  }

  // Clean up hardware interface
  if (!kuka_drivers_core::changeHardwareState(
        change_hardware_state_client_, robot_model_, State::PRIMARY_STATE_UNCONFIGURED))
  {
    RCLCPP_ERROR(get_logger(), "Could not clean up hardware interface");
    return FAILURE;
  }

  is_configured_msg_.data = false;
  is_configured_pub_->publish(is_configured_msg_);
  return SUCCESS;
}

void RobotManagerNodeBase::EventSubscriptionCallback(const std_msgs::msg::UInt8::SharedPtr msg)
{
  switch (static_cast<kuka_drivers_core::HardwareEvent>(msg->data))
  {
    case kuka_drivers_core::HardwareEvent::CONTROL_STARTED:
    {
      RCLCPP_INFO(get_logger(), "External control started");
      // Notify lock after control mode change
      {
        std::lock_guard<std::mutex> lk(control_mode_cv_m_);
        control_mode_change_finished_ = true;
      }
      control_mode_cv_.notify_all();
      break;
    }
    case kuka_drivers_core::HardwareEvent::CONTROL_STOPPED:
    case kuka_drivers_core::HardwareEvent::ERROR:
      RCLCPP_INFO(get_logger(), "External control stopped");
      terminate_ = true;
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
    case kuka_drivers_core::HardwareEvent::CONTROL_MODE_SWITCH:
      control_mode_change_finished_ = false;
      break;
    default:
      break;
  }
}


bool RobotManagerNodeBase::onRobotModelChangeRequest(const std::string & robot_model)
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
}  // namespace kuka_eac
