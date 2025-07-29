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

#include "communication_helpers/ros2_control_tools.hpp"
#include "communication_helpers/service_tools.hpp"

#include "kuka_drivers_core/control_mode.hpp"
#include "kuka_drivers_core/controller_names.hpp"
#include "kuka_drivers_core/hardware_event.hpp"
#include "kuka_rsi_driver/robot_manager_node_eki_rsi.hpp"

using controller_manager_msgs::srv::SetHardwareComponentState;
using controller_manager_msgs::srv::SwitchController;
using lifecycle_msgs::msg::State;

namespace kuka_rsi_driver
{
RobotManagerNodeEkiRsi::RobotManagerNodeEkiRsi()
: kuka_drivers_core::ROS2BaseLCNode("robot_manager")
{
  auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
  qos.reliable();

  cbg_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  event_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  change_hardware_state_client_ = create_client<SetHardwareComponentState>(
    "controller_manager/set_hardware_component_state", qos, cbg_);

  change_controller_state_client_ =
    create_client<SwitchController>("controller_manager/switch_controller", qos, cbg_);

  auto is_configured_qos = rclcpp::QoS(rclcpp::KeepLast(1));
  is_configured_qos.best_effort();
  is_configured_pub_ =
    create_publisher<std_msgs::msg::Bool>("robot_manager/is_configured", is_configured_qos);

  control_mode_pub_ = create_publisher<std_msgs::msg::UInt32>(
    "control_mode_handler/control_mode", rclcpp::SystemDefaultsQoS());

  // Subscribe to event_broadcaster/hardware_event
  rclcpp::SubscriptionOptions sub_options;
  sub_options.callback_group = event_callback_group_;
  event_subscriber_ = create_subscription<std_msgs::msg::UInt8>(
    "event_broadcaster/hardware_event", rclcpp::SystemDefaultsQoS(),
    [this](const std_msgs::msg::UInt8::SharedPtr message) { EventSubscriptionCallback(message); },
    sub_options);

  // Register parameters
  registerParameter<std::string>(
    "position_controller_name", kuka_drivers_core::JOINT_TRAJECTORY_CONTROLLER,
    kuka_drivers_core::ParameterSetAccessRights{true, false},
    [this](const std::string & controller_name)
    {
      return controller_handler_.UpdateControllerName(
        kuka_drivers_core::ControllerType::JOINT_POSITION_CONTROLLER_TYPE, controller_name);
    });

  registerParameter<int>(
    "control_mode", static_cast<int>(kuka_drivers_core::ControlMode::JOINT_POSITION_CONTROL),
    kuka_drivers_core::ParameterSetAccessRights{true, true},
    [this](int control_mode) { return OnControlModeChangeRequest(control_mode); });

  registerStaticParameter<std::string>(
    "robot_model", "kr6_r700_sixx", kuka_drivers_core::ParameterSetAccessRights{false, false},
    [this](const std::string & robot_model) { return OnRobotModelChangeRequest(robot_model); });
}

CallbackReturn RobotManagerNodeEkiRsi::on_configure(const rclcpp_lifecycle::State &)
{
  const auto logger = get_logger();

  // Publish initial control mode
  std_msgs::msg::UInt32 message;
  message.data = static_cast<int>(control_mode_);
  control_mode_pub_->publish(message);

  // Configure hardware interface
  const bool hw_state_change_successful = kuka_drivers_core::changeHardwareState(
    change_hardware_state_client_, robot_model_, State::PRIMARY_STATE_INACTIVE);
  if (!hw_state_change_successful)
  {
    RCLCPP_ERROR(logger, "Could not configure hardware interface");
    return FAILURE;
  }

  // Activate control mode handler and event broadcaster controllers
  std::vector<std::string> controllers_to_activate{
    kuka_drivers_core::CONTROL_MODE_HANDLER,
    kuka_drivers_core::KSS_MESSAGE_HANDLER,
    kuka_drivers_core::EVENT_BROADCASTER,
  };
  const bool controller_activation_successful = kuka_drivers_core::changeControllerState(
    change_controller_state_client_, controllers_to_activate, {});
  if (!controller_activation_successful)
  {
    RCLCPP_ERROR(logger, "Could not activate control mode handler or event broadcaster controller");
    on_cleanup(get_current_state());
    return FAILURE;
  }

  // Publish that the hardware interface is now configured
  // This will start the control loop
  is_configured_pub_->on_activate();
  is_configured_msg_.data = true;
  is_configured_pub_->publish(is_configured_msg_);

  return SUCCESS;
}

CallbackReturn RobotManagerNodeEkiRsi::on_cleanup(const rclcpp_lifecycle::State &)
{
  const auto logger = get_logger();

  // Deactivate control mode handler and event broadcaster
  std::vector<std::string> controllers_to_deactivate{
    kuka_drivers_core::CONTROL_MODE_HANDLER,
    kuka_drivers_core::KSS_MESSAGE_HANDLER,
    kuka_drivers_core::EVENT_BROADCASTER,
  };
  const bool controller_deactivation_successful = kuka_drivers_core::changeControllerState(
    change_controller_state_client_, {}, controllers_to_deactivate);
  if (!controller_deactivation_successful)
  {
    RCLCPP_ERROR(logger, "Could not deactivate control mode handler and event broadcaster");
  }

  // Clean up hardware interface
  const bool hw_state_change_successful = kuka_drivers_core::changeHardwareState(
    change_hardware_state_client_, robot_model_, State::PRIMARY_STATE_UNCONFIGURED);
  if (!hw_state_change_successful)
  {
    RCLCPP_ERROR(logger, "Could not clean up hardware interface");
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

CallbackReturn RobotManagerNodeEkiRsi::on_activate(const rclcpp_lifecycle::State &)
{
  const auto logger = get_logger();
  terminate_ = false;

  // Activate hardware interface
  const bool hw_state_change_successful = kuka_drivers_core::changeHardwareState(
    change_hardware_state_client_, robot_model_, State::PRIMARY_STATE_ACTIVE,
    RobotManagerNodeEkiRsi::HARDWARE_ACTIVATION_TIMEOUT_MS);
  if (!hw_state_change_successful)
  {
    RCLCPP_ERROR(logger, "Could not activate hardware interface");
    return FAILURE;
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(20));

  // Activate RT controller(s)
  auto activate_controllers = controller_handler_.GetControllersForMode(control_mode_);
  activate_controllers.push_back(kuka_drivers_core::JOINT_STATE_BROADCASTER);
  activate_controllers.push_back(kuka_drivers_core::GPIO_CONTROLLER);

  const bool controller_activation_successful = kuka_drivers_core::changeControllerState(
    change_controller_state_client_, activate_controllers, {});
  if (!controller_activation_successful)
  {
    RCLCPP_ERROR(logger, "Could not activate RT controllers");
    on_deactivate(get_current_state());
    return FAILURE;
  }

  if (terminate_)
  {
    RCLCPP_ERROR(logger, "Error occurred during driver activation");
    return FAILURE;
  }

  return SUCCESS;
}

CallbackReturn RobotManagerNodeEkiRsi::on_deactivate(const rclcpp_lifecycle::State &)
{
  const auto logger = get_logger();

  // Deactivate hardware interface
  bool deactivation_successful = kuka_drivers_core::changeHardwareState(
    change_hardware_state_client_, robot_model_, State::PRIMARY_STATE_INACTIVE);
  if (!deactivation_successful)
  {
    RCLCPP_ERROR(logger, "Could not deactivate hardware interface");
    return ERROR;
  }

  // Stop real-time controllers with best effort strictness
  // Deactivation should also succeed if some controllers are not active
  auto deactivate_controllers = controller_handler_.GetControllersForMode(control_mode_);
  deactivate_controllers.push_back(kuka_drivers_core::JOINT_STATE_BROADCASTER);
  deactivate_controllers.push_back(kuka_drivers_core::GPIO_CONTROLLER);

  auto controller_request = std::make_shared<SwitchController::Request>();
  controller_request->strictness = SwitchController::Request::BEST_EFFORT;
  controller_request->deactivate_controllers = deactivate_controllers;
  controller_request->timeout.sec = 5;

  auto controller_response = kuka_drivers_core::sendRequest<SwitchController::Response>(
    change_controller_state_client_, controller_request, 0, SWITCH_RESPONSE_TIMEOUT_MS);

  if (!controller_response || !controller_response->ok)
  {
    RCLCPP_ERROR(logger, "Could not stop controllers");
    return ERROR;
  }

  RCLCPP_INFO(logger, "Successfully stopped controllers");
  return SUCCESS;
}

void RobotManagerNodeEkiRsi::EventSubscriptionCallback(
  const std_msgs::msg::UInt8::SharedPtr message)
{
  const auto logger = get_logger();

  const auto event = static_cast<kuka_drivers_core::HardwareEvent>(message->data);
  switch (event)
  {
    case kuka_drivers_core::HardwareEvent::CONTROL_STARTED:
      RCLCPP_INFO(logger, "External control started");
      {
        std::lock_guard<std::mutex> lk(control_mode_cv_m_);
        control_mode_change_finished_ = true;
      }
      control_mode_cv_.notify_all();
      break;
    case kuka_drivers_core::HardwareEvent::CONTROL_STOPPED:
    case kuka_drivers_core::HardwareEvent::ERROR:
      RCLCPP_INFO(logger, "External control stopped");
      terminate_ = true;
      if (get_current_state().id() == State::PRIMARY_STATE_ACTIVE)
      {
        deactivate();
      }
      else if (get_current_state().id() == State::TRANSITION_STATE_ACTIVATING)
      {
        on_deactivate(get_current_state());
      }
      __attribute__((fallthrough));
    case kuka_drivers_core::HardwareEvent::CONTROL_MODE_SWITCH:
      control_mode_change_finished_ = false;
      break;
    default:
      break;
  }
}

bool RobotManagerNodeEkiRsi::OnControlModeChangeRequest(const int control_mode)
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

  // Publish control mode change to control mode handler
  std_msgs::msg::UInt32 message;
  message.data = control_mode;
  control_mode_pub_->publish(message);

  if (get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
  {
    const auto & controller_to_deactivate =
      controller_handler_.GetControllersForMode(control_mode_);
    bool deactivation_successful = kuka_drivers_core::changeControllerState(
      change_controller_state_client_, {}, controller_to_deactivate);
    if (!deactivation_successful)
    {
      RCLCPP_ERROR(logger, "Could not deactivate controllers of previous control mode");
      on_deactivate(get_current_state());
      return false;
    }

    std::unique_lock<std::mutex> control_mode_lk(control_mode_cv_m_);
    if (!control_mode_cv_.wait_for(
          control_mode_lk, std::chrono::milliseconds(3000),
          [this]() { return control_mode_change_finished_; }))
    {
      RCLCPP_ERROR(logger, "Timeout reached while waiting for robot to change control mode.");
      on_deactivate(get_current_state());
      return false;
    }
    control_mode_change_finished_ = false;
    control_mode_lk.unlock();
    RCLCPP_INFO(logger, "Robot Controller finished control mode change");

    std::this_thread::sleep_for(std::chrono::milliseconds(30));

    const auto & controllers_to_activate =
      controller_handler_.GetControllersForMode(target_control_mode);
    bool activation_successful = kuka_drivers_core::changeControllerState(
      change_controller_state_client_, controllers_to_activate, {});
    if (!activation_successful)
    {
      RCLCPP_ERROR(logger, "Could not activate controllers for new control mode");
      on_deactivate(get_current_state());
      return false;
    }
  }

  control_mode_ = target_control_mode;

  RCLCPP_INFO(logger, "Successfully changed control mode to %i", control_mode);

  return true;
}

bool RobotManagerNodeEkiRsi::OnRobotModelChangeRequest(const std::string & robot_model)
{
  auto ns = std::string(get_namespace());
  ns.erase(ns.begin());

  if (ns.size() > 0)
  {
    ns += "_";
  }
  robot_model_ = ns + robot_model;
  return true;
}
}  // namespace kuka_rsi_driver

int main(int argc, char * argv[])
{
  setvbuf(stdout, nullptr, _IONBF, BUFSIZ);

  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor;
  auto node = std::make_shared<kuka_rsi_driver::RobotManagerNodeEkiRsi>();
  executor.add_node(node->get_node_base_interface());
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
