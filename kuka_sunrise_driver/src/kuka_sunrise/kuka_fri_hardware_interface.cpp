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

  if (info_.gpios.size() != 1) {
    RCLCPP_FATAL(
      rclcpp::get_logger("KUKAFRIHardwareInterface"),
      "expecting exactly 1 GPIO");
    return CallbackReturn::ERROR;
  }

  if (info_.gpios[0].command_interfaces.size() > 10 ||
    info_.gpios[0].state_interfaces.size() > 10)
  {
    RCLCPP_FATAL(
      rclcpp::get_logger("KUKAFRIHardwareInterface"),
      "A maximum of 10 inputs and outputs can be registered to FRI");
    return CallbackReturn::ERROR;
  }

  for (const auto & state_if : info_.gpios[0].state_interfaces) {
    gpio_outputs_.emplace_back(state_if.name, getType(state_if.data_type), robotState());
  }

  for (const auto & command_if : info_.gpios[0].command_interfaces) {
    gpio_inputs_.emplace_back(
      command_if.name, getType(command_if.data_type),
      robotCommand(), std::stod(command_if.initial_value));
  }


  for (const hardware_interface::ComponentInfo & joint : info_.joints) {
    if (joint.command_interfaces.size() != 2) {
      RCLCPP_FATAL(
        rclcpp::get_logger("KUKAFRIHardwareInterface"),
        "expecting exactly 2 command interface");
      return CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
      RCLCPP_FATAL(
        rclcpp::get_logger(
          "KUKAFRIHardwareInterface"), "expecting POSITION command interface as first");
      return CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[1].name != hardware_interface::HW_IF_EFFORT) {
      RCLCPP_FATAL(
        rclcpp::get_logger(
          "KUKAFRIHardwareInterface"), "expecting EFFORT command interface as second");
      return CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 3) {
      RCLCPP_FATAL(
        rclcpp::get_logger(
          "KUKAFRIHardwareInterface"), "expecting exactly 3 state interface");
      return CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
      RCLCPP_FATAL(
        rclcpp::get_logger(
          "KUKAFRIHardwareInterface"), "expecting POSITION state interface as first");
      return CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_EFFORT) {
      RCLCPP_FATAL(
        rclcpp::get_logger(
          "KUKAFRIHardwareInterface"), "expecting EFFORT state interface as second");
      return CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[2].name != "external_torque") {
      RCLCPP_FATAL(
        rclcpp::get_logger(
          "KUKAFRIHardwareInterface"), "expecting 'external torque' state interface as third");
      return CallbackReturn::ERROR;
    }
  }

  struct sched_param param;
  param.sched_priority = 95;
  if (sched_setscheduler(0, SCHED_FIFO, &param) == -1) {
    RCLCPP_ERROR(rclcpp::get_logger("KukaRoXHardwareInterface"), "setscheduler error");
    RCLCPP_ERROR(rclcpp::get_logger("KukaRoXHardwareInterface"), strerror(errno));
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn KUKAFRIHardwareInterface::on_activate(const rclcpp_lifecycle::State &)
{
  if (!client_application_.connect(30200, nullptr)) {
    RCLCPP_ERROR(rclcpp::get_logger("KUKAFRIHardwareInterface"), "Could not connect");
    return CallbackReturn::FAILURE;
  }
  is_active_= true;
  return CallbackReturn::SUCCESS;
}

CallbackReturn KUKAFRIHardwareInterface::on_deactivate(const rclcpp_lifecycle::State &)
{
  client_application_.disconnect();
  is_active_= false;
  return CallbackReturn::SUCCESS;
}

void KUKAFRIHardwareInterface::waitForCommand()
{
  hw_commands_ = hw_states_;
  hw_effort_command_ = hw_torques_;
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
  // Read is called in inactive state, check is necessary
  if (!is_active_) {
    RCLCPP_DEBUG(rclcpp::get_logger("KUKAFRIHardwareInterface"), "Hardware interface not active");
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    return hardware_interface::return_type::OK;
  }

  if (!client_application_.client_app_read()) {
    RCLCPP_ERROR(
      rclcpp::get_logger(
        "KUKAFRIHardwareInterface"), "Failed to read data from controller");
    return hardware_interface::return_type::ERROR;
  }

  // get the position and efforts and share them with exposed state interfaces
  const double * position = robotState().getMeasuredJointPosition();
  hw_states_.assign(position, position + KUKA::FRI::LBRState::NUMBER_OF_JOINTS);
  const double * torque = robotState().getMeasuredTorque();
  hw_torques_.assign(torque, torque + KUKA::FRI::LBRState::NUMBER_OF_JOINTS);
  const double * external_torque = robotState().getExternalTorque();
  hw_torques_ext_.assign(external_torque, external_torque + KUKA::FRI::LBRState::NUMBER_OF_JOINTS);

  robot_state_.tracking_performance_ = robotState().getTrackingPerformance();
  robot_state_.session_state_ = robotState().getSessionState();
  robot_state_.connection_quality_ = robotState().getConnectionQuality();
  robot_state_.command_mode_ = robotState().getClientCommandMode();
  robot_state_.safety_state_ = robotState().getSafetyState();
  robot_state_.control_mode_ = robotState().getControlMode();
  robot_state_.operation_mode_ = robotState().getOperationMode();
  robot_state_.drive_state_ = robotState().getDriveState();
  robot_state_.overlay_type_ = robotState().getOverlayType();

  for (auto & output : gpio_outputs_) {
    output.getValue();
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type KUKAFRIHardwareInterface::write(
  const rclcpp::Time &,
  const rclcpp::Duration &)
{
  // Write is called in inactive state, check is necessary
  if (!is_active_) {
    RCLCPP_DEBUG(rclcpp::get_logger("KUKAFRIHardwareInterface"), "Hardware interface not active");
    return hardware_interface::return_type::OK;
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
    RCLCPP_ERROR(
      rclcpp::get_logger(
        "KUKAFRIHardwareInterface"), "Hardware inactive, exiting updateCommand");
    return;
  }
  if (robot_state_.command_mode_ == KUKA::FRI::EClientCommandMode::TORQUE) {
    const double * joint_torques_ = hw_effort_command_.data();
    robotCommand().setJointPosition(robotState().getIpoJointPosition());
    robotCommand().setTorque(joint_torques_);
  } else if (robot_state_.command_mode_ == KUKA::FRI::EClientCommandMode::POSITION) {
    const double * joint_positions_ = hw_commands_.data();
    robotCommand().setJointPosition(joint_positions_);
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("KUKAFRIHardwareInterface"), "Unsupported command mode");
  }
  for (auto & input : gpio_inputs_) {
    input.setValue();
  }
}

std::vector<hardware_interface::StateInterface> KUKAFRIHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  state_interfaces.emplace_back("state", "session_state", &robot_state_.session_state_);
  state_interfaces.emplace_back("state", "connection_quality", &robot_state_.connection_quality_);
  state_interfaces.emplace_back("state", "safety_state", &robot_state_.safety_state_);
  state_interfaces.emplace_back("state", "command_mode", &robot_state_.command_mode_);
  state_interfaces.emplace_back("state", "control_mode", &robot_state_.control_mode_);
  state_interfaces.emplace_back("state", "operation_mode", &robot_state_.operation_mode_);
  state_interfaces.emplace_back("state", "drive_state", &robot_state_.drive_state_);
  state_interfaces.emplace_back("state", "overlay_type", &robot_state_.overlay_type_);
  state_interfaces.emplace_back(
    "state", "tracking_performance",
    &robot_state_.tracking_performance_);

  // Register I/O outputs (read access)
  for (auto & output : gpio_outputs_) {
    state_interfaces.emplace_back(
      "gpio", output.getName(),
      &output.getData());
  }

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

  // Register I/O inputs (write access)
  for (auto & input : gpio_inputs_) {
    command_interfaces.emplace_back(
      "gpio", input.getName(),
      &input.getData());
  }

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
