// Copyright 2025 KUKA Hungaria Kft.
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

#include <cstdio>
#include <functional>

#include "communication_helpers/ros2_control_tools.hpp"
#include "communication_helpers/service_tools.hpp"

#include "kuka_drivers_core/control_mode.hpp"
#include "kuka_drivers_core/controller_names.hpp"
#include "kuka_drivers_core/hardware_event.hpp"
#include "kuka_kss_rsi_driver/robot_manager_node.hpp"

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace kuka_kss_rsi_driver
{

RobotManagerNode::RobotManagerNode() : kuka_drivers_core::ROS2BaseLCNode("robot_manager")
{
  const auto qos = rclcpp::QoS{rclcpp::KeepLast(10)}.reliable();
  cbg_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  event_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  change_hardware_state_client_ =
    create_client<controller_manager_msgs::srv::SetHardwareComponentState>(
      "controller_manager/set_hardware_component_state", qos, cbg_);

  change_controller_state_client_ = create_client<controller_manager_msgs::srv::SwitchController>(
    "controller_manager/switch_controller", qos, cbg_);

  is_configured_pub_ = create_publisher<std_msgs::msg::Bool>(
    "robot_manager/is_configured", rclcpp::QoS{rclcpp::KeepLast{1}}.best_effort());

  control_mode_pub_ = create_publisher<std_msgs::msg::UInt32>(
    "control_mode_handler/control_mode", rclcpp::SystemDefaultsQoS{});

  using namespace std::placeholders;
  rclcpp::SubscriptionOptions sub_options;
  sub_options.callback_group = event_callback_group_;
  event_subscriber_ = create_subscription<std_msgs::msg::UInt8>(
    "event_broadcaster/hardware_event", rclcpp::SystemDefaultsQoS(),
    std::bind(&RobotManagerNode::HardwareEventSubscriptionCallback, this, _1), sub_options);

  registerParameter<std::string>(
    "position_controller_name", kuka_drivers_core::JOINT_TRAJECTORY_CONTROLLER,
    kuka_drivers_core::ParameterSetAccessRights{true, false},
    std::bind(
      &kuka_drivers_core::ControllerHandler::UpdateControllerName, &controller_handler_,
      kuka_drivers_core::ControllerType::JOINT_POSITION_CONTROLLER_TYPE, _1));

  registerParameter<int>(
    "control_mode", static_cast<int>(kuka_drivers_core::ControlMode::JOINT_POSITION_CONTROL),
    kuka_drivers_core::ParameterSetAccessRights{true, true},
    std::bind(&RobotManagerNode::ControlModeChangeRequestedCallback, this, _1));

  registerStaticParameter<std::string>(
    "robot_model", "kr6_r700_sixx", kuka_drivers_core::ParameterSetAccessRights{false, false},
    std::bind(&RobotManagerNode::RobotModelChangeRequestedCallback, this, _1));
}

CallbackReturn RobotManagerNode::on_configure(const rclcpp_lifecycle::State &)
{
  // Publish the initial control mode
  std_msgs::msg::UInt32 message;
  message.data = static_cast<int>(control_mode_);
  control_mode_pub_->publish(message);

  // Configure hardware interface
  if (!kuka_drivers_core::changeHardwareState(
        change_hardware_state_client_, robot_model_,
        lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE))
  {
    RCLCPP_ERROR(get_logger(), "Failed to configure hardware interface");
    return FAILURE;
  }

  // Activate non-real-time controllers
  if (!kuka_drivers_core::changeControllerState(
        change_controller_state_client_,
        {
          kuka_drivers_core::CONTROL_MODE_HANDLER,
          kuka_drivers_core::NRT_MESSAGE_HANDLER,
          kuka_drivers_core::EVENT_BROADCASTER,
        },
        {}))
  {
    RCLCPP_ERROR(get_logger(), "Failed to activate non-real-time controllers");
    on_cleanup(get_current_state());
    return FAILURE;
  }

  // Publish that the robot manager is now configured
  is_configured_pub_->on_activate();
  std_msgs::msg::Bool is_configured_msg;
  is_configured_msg.data = true;
  is_configured_pub_->publish(is_configured_msg);

  return SUCCESS;
}

CallbackReturn RobotManagerNode::on_activate(const rclcpp_lifecycle::State &)
{
  terminate_ = false;

  // Activate hardware interface
  if (!kuka_drivers_core::changeHardwareState(
        change_hardware_state_client_, robot_model_,
        lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE, HARDWARE_INTERFACE_ACTIVATION_TIMEOUT_MS))
  {
    RCLCPP_ERROR(get_logger(), "Failed to activate hardware interface");
    return FAILURE;
  }

  // Activate real-time controllers
  auto activate_controllers = controller_handler_.GetControllersForMode(control_mode_);
  activate_controllers.push_back(kuka_drivers_core::JOINT_STATE_BROADCASTER);
  if (!kuka_drivers_core::changeControllerState(
        change_controller_state_client_, activate_controllers, {}))
  {
    RCLCPP_ERROR(get_logger(), "Failed to activate real-time controllers");
    on_deactivate(get_current_state());
    return FAILURE;
  }

  // Check if a driver error occurred during activation
  if (terminate_)
  {
    RCLCPP_ERROR(get_logger(), "Failed to activate robot manager due to a driver error");
    return FAILURE;
  }

  return SUCCESS;
}

CallbackReturn RobotManagerNode::on_deactivate(const rclcpp_lifecycle::State &)
{
  // Deactivate real-time controllers
  auto deactivate_controllers = controller_handler_.GetControllersForMode(control_mode_);
  deactivate_controllers.push_back(kuka_drivers_core::JOINT_STATE_BROADCASTER);
  if (!kuka_drivers_core::changeControllerState(
        change_controller_state_client_, {}, deactivate_controllers,
        controller_manager_msgs::srv::SwitchController::Request::BEST_EFFORT,
        REAL_TIME_CONTROLLER_DEACTIVATION_TIMEOUT_SEC))
  {
    RCLCPP_ERROR(get_logger(), "Failed to deactivate real-time controllers");
    return FAILURE;
  }

  // Deactivate hardware interface
  if (!kuka_drivers_core::changeHardwareState(
        change_hardware_state_client_, robot_model_,
        lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE))
  {
    RCLCPP_ERROR(get_logger(), "Failed to deactivate hardware interface");
    return ERROR;
  }

  return SUCCESS;
}

CallbackReturn RobotManagerNode::on_cleanup(const rclcpp_lifecycle::State &)
{
  // Publish that the robot manager is no longer configured
  if (is_configured_pub_->is_activated())
  {
    std_msgs::msg::Bool is_configured_msg;
    is_configured_msg.data = false;
    is_configured_pub_->publish(is_configured_msg);
    is_configured_pub_->on_deactivate();
  }

  // Deactivate non-real-time controllers
  if (!kuka_drivers_core::changeControllerState(
        change_controller_state_client_, {},
        {
          kuka_drivers_core::CONTROL_MODE_HANDLER,
          kuka_drivers_core::NRT_MESSAGE_HANDLER,
          kuka_drivers_core::EVENT_BROADCASTER,
        }))
  {
    RCLCPP_ERROR(get_logger(), "Failed to deactivate non-real-time controllers");
    return FAILURE;
  }

  // Clean up hardware interface
  if (!kuka_drivers_core::changeHardwareState(
        change_hardware_state_client_, robot_model_,
        lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED))
  {
    RCLCPP_ERROR(get_logger(), "Failed to clean up hardware interface");
    return FAILURE;
  }

  return SUCCESS;
}

void RobotManagerNode::HardwareEventSubscriptionCallback(
  const std_msgs::msg::UInt8::SharedPtr message)
{
  switch (static_cast<kuka_drivers_core::HardwareEvent>(message->data))
  {
    case kuka_drivers_core::HardwareEvent::CONTROL_STARTED:
      RCLCPP_INFO(get_logger(), "External control started");
      {
        std::lock_guard<std::mutex> lk{control_mode_cv_m_};
        control_mode_change_finished_ = true;
      }
      control_mode_cv_.notify_all();
      break;
    case kuka_drivers_core::HardwareEvent::CONTROL_STOPPED:
    case kuka_drivers_core::HardwareEvent::ERROR:
      RCLCPP_INFO(get_logger(), "External control stopped");
      terminate_ = true;
      switch (get_current_state().id())
      {
        case lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE:
          deactivate();
          break;
        case lifecycle_msgs::msg::State::TRANSITION_STATE_ACTIVATING:
          on_deactivate(get_current_state());
          break;
        default:
          break;
      }
      __attribute__((fallthrough));
    case kuka_drivers_core::HardwareEvent::CONTROL_MODE_SWITCH:
      control_mode_change_finished_ = false;
      break;
    default:
      break;
  }
}

bool RobotManagerNode::ControlModeChangeRequestedCallback(const int control_mode)
{
  const auto target_control_mode = static_cast<kuka_drivers_core::ControlMode>(control_mode);

  if (control_mode_ == target_control_mode)
  {
    RCLCPP_WARN(
      get_logger(), "Requested control mode change ignored: already using the specified mode");
    return true;
  }

  if (target_control_mode != kuka_drivers_core::ControlMode::JOINT_POSITION_CONTROL)
  {
    RCLCPP_ERROR(
      get_logger(), "Control mode change failed: requested control mode is not supported");
    return false;
  }

  // Publish new control mode
  std_msgs::msg::UInt32 control_mode_msg;
  control_mode_msg.data = control_mode;
  control_mode_pub_->publish(control_mode_msg);

  // Change real-time controllers for the new control mode
  if (get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
  {
    if (!kuka_drivers_core::changeControllerState(
          change_controller_state_client_, {},
          controller_handler_.GetControllersForMode(control_mode_)))
    {
      RCLCPP_ERROR(
        get_logger(), "Failed to deactivate controllers associated with the previous control mode");
      on_deactivate(get_current_state());
      return false;
    }

    std::unique_lock<std::mutex> control_mode_lk(control_mode_cv_m_);
    if (!control_mode_cv_.wait_for(
          control_mode_lk, CONTROL_MODE_CHANGE_TIMEOUT_MS,
          [this]() { return control_mode_change_finished_; }))
    {
      RCLCPP_ERROR(
        get_logger(), "Timeout occured while waiting for the robot switch to the new control mode");
      on_deactivate(get_current_state());
      return false;
    }
    control_mode_change_finished_ = false;
    control_mode_lk.unlock();

    if (!kuka_drivers_core::changeControllerState(
          change_controller_state_client_,
          controller_handler_.GetControllersForMode(target_control_mode), {}))
    {
      RCLCPP_ERROR(get_logger(), "Failed to activate controllers for the new control mode");
      on_deactivate(get_current_state());
      return false;
    }
  }

  control_mode_ = target_control_mode;
  RCLCPP_INFO(get_logger(), "Control mode changed");
  return true;
}

bool RobotManagerNode::RobotModelChangeRequestedCallback(const std::string & robot_model)
{
  std::string ns{get_namespace()};
  ns.erase(ns.begin());
  if (ns.size() > 0)
  {
    ns += "_";
  }
  robot_model_ = ns + robot_model;
  return true;
}

}  // namespace kuka_kss_rsi_driver

int main(int argc, char * argv[])
{
  if (std::setvbuf(stdout, nullptr, _IONBF, BUFSIZ) != 0)
  {
    std::perror("setvbuf failed");
    return EXIT_FAILURE;
  }

  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor;
  auto node = std::make_shared<kuka_kss_rsi_driver::RobotManagerNode>();
  executor.add_node(node->get_node_base_interface());
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
