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
#include "kuka_rsi_driver/robot_manager_node_extended.hpp"

using controller_manager_msgs::srv::SetHardwareComponentState;
using controller_manager_msgs::srv::SwitchController;
using lifecycle_msgs::msg::State;

namespace kuka_rsi_driver
{
RobotManagerNodeEkiRsi::RobotManagerNodeEkiRsi()
{
  control_mode_pub_ = create_publisher<std_msgs::msg::UInt32>(
    "control_mode_handler/control_mode", rclcpp::SystemDefaultsQoS());
}

CallbackReturn RobotManagerNodeEkiRsi::on_configure(const rclcpp_lifecycle::State &)
{
  const auto logger = get_logger();

  // Publish initial control mode
  std_msgs::msg::UInt32 message;
  message.data = static_cast<int>(control_mode_);
  control_mode_pub_->publish(message);

  return RobotManagerBase::configure(configuration_controllers_);
}

CallbackReturn RobotManagerNodeEkiRsi::on_cleanup(const rclcpp_lifecycle::State &)
{
  return RobotManagerBase::cleanup(configuration_controllers_);
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
