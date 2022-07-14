// Copyright 2020 Zoltán Rési
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

#include <memory>

#include "kuka_sunrise/kuka_fri_hardware_interface.hpp"

namespace kuka_sunrise
{
CallbackReturn KUKAFRIHardwareInterface::on_init(
  const hardware_interface::HardwareInfo & system_info)
{
  if (hardware_interface::SystemInterface::on_init(system_info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }
  hw_states_.resize(info_.joints.size());
  hw_commands_.resize(info_.joints.size());
  hw_torques_.resize(info_.joints.size());
  hw_torques_ext_.resize(info_.joints.size());
  hw_effort_command_.resize(info_.joints.size());

  for (const hardware_interface::ComponentInfo & joint : info_.joints) {
    if (joint.command_interfaces.size() != 2) {
      RCLCPP_FATAL(
        rclcpp::get_logger("RobotControlClient"),
        "expecting exactly 2 command interface");
      return CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
      RCLCPP_FATAL(
        rclcpp::get_logger(
          "RobotControlClient"), "expecting POSITION command interface as first");
      return CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[1].name != hardware_interface::HW_IF_EFFORT) {
      RCLCPP_FATAL(
        rclcpp::get_logger(
          "RobotControlClient"), "expecting EFFORT command interface as second");
      return CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 3) {
      RCLCPP_FATAL(rclcpp::get_logger("RobotControlClient"), "expecting exactly 3 state interface");
      return CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
      RCLCPP_FATAL(
        rclcpp::get_logger(
          "RobotControlClient"), "expecting POSITION state interface as first");
      return CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_EFFORT) {
      RCLCPP_FATAL(
        rclcpp::get_logger(
          "RobotControlClient"), "expecting EFFORT state interface as second");
      return CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[2].name != "external_torque") {
      RCLCPP_FATAL(
        rclcpp::get_logger(
          "RobotControlClient"), "expecting 'external torque' state interface as third");
      return CallbackReturn::ERROR;
    }
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn KUKAFRIHardwareInterface::on_configure(const rclcpp_lifecycle::State &)
{
  return CallbackReturn::SUCCESS;
}

CallbackReturn KUKAFRIHardwareInterface::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger("HWIF"), "activating client");
  if (!client_application_.connect(30200, nullptr)) {
    RCLCPP_ERROR(rclcpp::get_logger("HWIF"), "could not connect");
    return CallbackReturn::FAILURE;
  }
  this->ActivatableInterface::activate();
  RCLCPP_INFO(rclcpp::get_logger("HWIF"), "activated client");
  return CallbackReturn::SUCCESS;
}

CallbackReturn KUKAFRIHardwareInterface::on_deactivate(const rclcpp_lifecycle::State &)
{
  client_application_.disconnect();
  this->ActivatableInterface::deactivate();
  return CallbackReturn::SUCCESS;
}

void KUKAFRIHardwareInterface::waitForCommand()
{
  // TODO(Svastits): is this really the purpose of waitForCommand?
  rclcpp::Time stamp = ros_clock_.now();
  if (++receive_counter_ == receive_multiplier_) {
    updateCommand(stamp);
    receive_counter_ = 0;
  }
}

void KUKAFRIHardwareInterface::command()
{
  rclcpp::Time stamp = ros_clock_.now();
  if (++receive_counter_ == receive_multiplier_) {
    updateCommand(stamp);
    receive_counter_ = 0;
  }
}


hardware_interface::return_type KUKAFRIHardwareInterface::read(
  const rclcpp::Time &,
  const rclcpp::Duration &)
{
  if (!is_active_) {
    RCLCPP_ERROR(rclcpp::get_logger("ClientApplication"), "Controller not active");
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    return hardware_interface::return_type::ERROR;
  }

  if (!client_application_.client_app_read()) {
    RCLCPP_ERROR(rclcpp::get_logger("ClientApplication"), "Failed to read data from controller");
    return hardware_interface::return_type::ERROR;
  }

  // get the position and efforts and share them with exposed state interfaces
  const double * position = robotState().getMeasuredJointPosition();
  hw_states_.assign(position, position + KUKA::FRI::LBRState::NUMBER_OF_JOINTS);
  const double * torque = robotState().getMeasuredTorque();
  hw_torques_.assign(torque, torque + KUKA::FRI::LBRState::NUMBER_OF_JOINTS);
  const double * external_torque = robotState().getExternalTorque();
  hw_torques_ext_.assign(external_torque, external_torque + KUKA::FRI::LBRState::NUMBER_OF_JOINTS);

  tracking_performance_ = robotState().getTrackingPerformance();
  fri_state_ = robotState().getSessionState();
  connection_quality_ = robotState().getConnectionQuality();

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type KUKAFRIHardwareInterface::write(
  const rclcpp::Time &,
  const rclcpp::Duration &)
{
  if (!is_active_) {
    RCLCPP_INFO(rclcpp::get_logger("RobotControlClient"), "Controller deactivated");
    return hardware_interface::return_type::ERROR;
  }

  // Call the appropriate callback for the actual state (e.g. updateCommand)
  // this updates the command to be sent based on the output of the controller update
  client_application_.client_app_update();

  client_application_.client_app_write();

  return hardware_interface::return_type::OK;
}

void KUKAFRIHardwareInterface::updateCommand(const rclcpp::Time &)
{
  if (!is_active_) {
    printf("client deactivated, exiting updateCommand\n");
    return;
  }
  // TODO(Svastits): implement command mode switch
  if (torque_command_mode_) {
    const double * joint_torques_ = hw_effort_command_.data();
    robotCommand().setJointPosition(robotState().getIpoJointPosition());
    robotCommand().setTorque(joint_torques_);
  } else {
    const double * joint_positions_ = hw_commands_.data();
    robotCommand().setJointPosition(joint_positions_);
  }
  // TODO(Svastits): setDigitalIOValue and setAnalogIOValue
/*
  for (auto & output_subscription : output_subscriptions_) {
    output_subscription->updateOutput();
  }
  */
}

std::vector<hardware_interface::StateInterface> KUKAFRIHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  state_interfaces.emplace_back("state", "fri_state", &fri_state_);
  state_interfaces.emplace_back("state", "connection_quality", &connection_quality_);

  for (size_t i = 0; i < info_.joints.size(); i++) {
    state_interfaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION,
      &hw_states_[i]);

    state_interfaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_EFFORT,
      &hw_torques_[i]);

    state_interfaces.emplace_back(info_.joints[i].name, "external_torque", &hw_torques_ext_[i]);
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> KUKAFRIHardwareInterface::
export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  command_interfaces.emplace_back("timing", "receive_multiplier", &receive_multiplier_);
  for (size_t i = 0; i < info_.joints.size(); i++) {
    command_interfaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION,
      &hw_commands_[i]);
    command_interfaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_EFFORT,
      &hw_effort_command_[i]);
  }
  return command_interfaces;
}
}  // namespace kuka_sunrise

PLUGINLIB_EXPORT_CLASS(
  kuka_sunrise::KUKAFRIHardwareInterface,
  hardware_interface::SystemInterface
)
