// Copyright 2023 KUKA Hungaria Kft.
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

#include "kuka_drivers_core/control_mode.hpp"
#include "kuka_drivers_core/controller_names.hpp"
#include "kuka_drivers_core/hardware_event.hpp"
#include "kuka_rsi_driver/robot_manager_base.hpp"

using namespace controller_manager_msgs::srv;  // NOLINT
using namespace lifecycle_msgs::msg;           // NOLINT

namespace kuka_rsi_driver
{
RobotManagerBase::RobotManagerBase() : kuka_drivers_core::ROS2BaseLCNode("robot_manager")
{
  auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
  qos.reliable();

  cbg_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  event_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  change_hardware_state_client_ = this->create_client<SetHardwareComponentState>(
    "controller_manager/set_hardware_component_state", qos, cbg_);

  change_controller_state_client_ =
    this->create_client<SwitchController>("controller_manager/switch_controller", qos, cbg_);

  auto is_configured_qos = rclcpp::QoS(rclcpp::KeepLast(1));
  is_configured_qos.best_effort();
  is_configured_pub_ =
    this->create_publisher<std_msgs::msg::Bool>("robot_manager/is_configured", is_configured_qos);

  // Subscribe to event_broadcaster/hardware_event
  rclcpp::SubscriptionOptions sub_options;
  sub_options.callback_group = event_callback_group_;
  event_subscriber_ = create_subscription<std_msgs::msg::UInt8>(
    "event_broadcaster/hardware_event", rclcpp::SystemDefaultsQoS(),
    [this](const std_msgs::msg::UInt8::SharedPtr message) { EventSubscriptionCallback(message); },
    sub_options);

  // Register parameters
  this->registerParameter<std::string>(
    "position_controller_name", kuka_drivers_core::JOINT_TRAJECTORY_CONTROLLER,
    kuka_drivers_core::ParameterSetAccessRights{true, false},
    [this](const std::string & controller_name)
    {
      return controller_handler_.UpdateControllerName(
        kuka_drivers_core::ControllerType::JOINT_POSITION_CONTROLLER_TYPE, controller_name);
    });

  this->registerParameter<int>(
    "control_mode", static_cast<int>(kuka_drivers_core::ControlMode::JOINT_POSITION_CONTROL),
    kuka_drivers_core::ParameterSetAccessRights{true, true},
    [this](int control_mode) { return OnControlModeChangeRequest(control_mode); });

  this->registerStaticParameter<std::string>(
    "robot_model", "kr6_r700_sixx", kuka_drivers_core::ParameterSetAccessRights{false, false},
    [this](const std::string & robot_model) { return onRobotModelChangeRequest(robot_model); });

  this->registerStaticParameter<bool>(
    "use_gpio", false, kuka_drivers_core::ParameterSetAccessRights{false, false},
    [this](const bool use_gpio)
    {
      use_gpio_ = use_gpio;
      return true;
    });
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
RobotManagerBase::configure_driver(const std::vector<std::string> & controllers_to_activate)
{
  // Configure hardware interface
  if (!kuka_drivers_core::changeHardwareState(
        change_hardware_state_client_, robot_model_, State::PRIMARY_STATE_INACTIVE))
  {
    RCLCPP_ERROR(get_logger(), "Could not configure hardware interface");
    return FAILURE;
  }

  // Activate event broadcaster
  const bool controller_activation_successful = kuka_drivers_core::changeControllerState(
    change_controller_state_client_, controllers_to_activate, {});
  if (!controller_activation_successful)
  {
    RCLCPP_ERROR(get_logger(), "Could not activate configuration controllers");
    on_cleanup(get_current_state());
    return FAILURE;
  }

  is_configured_pub_->on_activate();
  is_configured_msg_.data = true;
  is_configured_pub_->publish(is_configured_msg_);
  return SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
RobotManagerBase::cleanup_driver(const std::vector<std::string> & controllers_to_deactivate)
{
  // Deactivate configuration controllers/broadcaster
  const bool controller_deactivation_successful = kuka_drivers_core::changeControllerState(
    change_controller_state_client_, {}, controllers_to_deactivate);
  if (!controller_deactivation_successful)
  {
    RCLCPP_ERROR(get_logger(), "Could not deactivate configuration controllers");
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
  return SUCCESS;
}

// TODO(Svastits): rollback in case of failures
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
RobotManagerBase::on_activate(const rclcpp_lifecycle::State &)
{
  const auto logger = get_logger();
  terminate_ = false;

  // Activate hardware interface
  const bool hw_state_change_successful = kuka_drivers_core::changeHardwareState(
    change_hardware_state_client_, robot_model_, State::PRIMARY_STATE_ACTIVE,
    RobotManagerBase::HARDWARE_ACTIVATION_TIMEOUT_MS);
  if (!hw_state_change_successful)
  {
    RCLCPP_ERROR(logger, "Could not activate hardware interface");
    return FAILURE;
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(20));

  // Activate RT controller(s)
  auto activate_controllers = controller_handler_.GetControllersForMode(control_mode_);
  activate_controllers.push_back(kuka_drivers_core::JOINT_STATE_BROADCASTER);
  if (use_gpio_)
  {
    activate_controllers.push_back(kuka_drivers_core::GPIO_CONTROLLER);
  }

  if (!kuka_drivers_core::changeControllerState(
        change_controller_state_client_, activate_controllers, {}))
  {
    RCLCPP_ERROR(logger, "Could not activate RT controllers");
    this->on_deactivate(get_current_state());
    return FAILURE;
  }

  if (terminate_)
  {
    RCLCPP_ERROR(logger, "Error occurred during driver activation");
    return FAILURE;
  }

  return SUCCESS;
}

CallbackReturn RobotManagerBase::on_deactivate(const rclcpp_lifecycle::State &)
{
  const auto logger = get_logger();

  // Deactivate hardware interface
  if (!kuka_drivers_core::changeHardwareState(
        change_hardware_state_client_, robot_model_, State::PRIMARY_STATE_INACTIVE,
        RobotManagerBase::HARDWARE_DEACTIVATION_TIMEOUT_MS))
  {
    RCLCPP_ERROR(logger, "Could not deactivate hardware interface");
    return ERROR;
  }

  // Stop real-time controllers with best effort strictness
  // Deactivation should also succeed if some controllers are not active
  auto deactivate_controllers = controller_handler_.GetControllersForMode(control_mode_);
  deactivate_controllers.push_back(kuka_drivers_core::JOINT_STATE_BROADCASTER);
  if (use_gpio_)
  {
    deactivate_controllers.push_back(kuka_drivers_core::GPIO_CONTROLLER);
  }

  if (!kuka_drivers_core::changeControllerState(
        change_controller_state_client_, {}, deactivate_controllers,
        SwitchController::Request::BEST_EFFORT))
  {
    RCLCPP_ERROR(get_logger(), "Could not stop controllers");
    return ERROR;
  }

  RCLCPP_INFO(logger, "Successfully stopped controllers");
  return SUCCESS;
}

bool RobotManagerBase::onRobotModelChangeRequest(const std::string & robot_model)
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

void RobotManagerBase::EventSubscriptionCallback(const std_msgs::msg::UInt8::SharedPtr message)
{
  const auto logger = get_logger();

  const auto event = static_cast<kuka_drivers_core::HardwareEvent>(message->data);
  if (event == kuka_drivers_core::HardwareEvent::ERROR)
  {
    RCLCPP_INFO(logger, "External control stopped by an error");
    terminate_ = true;
    if (get_current_state().id() == State::PRIMARY_STATE_ACTIVE)
    {
      deactivate();
    }
    else if (get_current_state().id() == State::TRANSITION_STATE_ACTIVATING)
    {
      on_deactivate(get_current_state());
    }
  }
}

bool RobotManagerBase::OnControlModeChangeRequest(const int control_mode)
{
  const auto logger = get_logger();

  kuka_drivers_core::ControlMode target_control_mode =
    static_cast<kuka_drivers_core::ControlMode>(control_mode);

  if (control_mode_ == target_control_mode)
  {
    RCLCPP_WARN(logger, "Tried to change control mode to the one currently used");
    return true;
  }

  RCLCPP_INFO(logger, "Control mode change requested");
  if (target_control_mode != kuka_drivers_core::ControlMode::JOINT_POSITION_CONTROL)
  {
    RCLCPP_ERROR(logger, "Tried to change to a not implemented control mode");
    return false;
  }

  if (get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
  {
    RCLCPP_ERROR(
      logger, "Changing control mode during active control is not supported by plain RSI driver");
    return false;
  }

  control_mode_ = target_control_mode;

  RCLCPP_INFO(logger, "Successfully changed control mode to %i", control_mode);

  return true;
}

}  // namespace kuka_rsi_driver
