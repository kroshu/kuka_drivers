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

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include "kuka_drivers_core/hardware_interface_types.hpp"

#include "kuka_sunrise_fri_driver/hardware_interface.hpp"

namespace kuka_sunrise_fri_driver
{
CallbackReturn KukaFRIHardwareInterface::on_init(
  const hardware_interface::HardwareInfo & system_info)
{
  if (hardware_interface::SystemInterface::on_init(system_info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }
  controller_ip_ = info_.hardware_parameters.at("controller_ip");

  hw_position_states_.resize(info_.joints.size());
  hw_position_commands_.resize(info_.joints.size());
  hw_torque_states_.resize(info_.joints.size());
  hw_ext_torque_states_.resize(info_.joints.size());
  hw_torque_commands_.resize(info_.joints.size());

  if (info_.gpios.size() != 1)
  {
    RCLCPP_FATAL(rclcpp::get_logger("KukaFRIHardwareInterface"), "expecting exactly 1 GPIO");
    return CallbackReturn::ERROR;
  }

  if (info_.gpios[0].command_interfaces.size() > 10 || info_.gpios[0].state_interfaces.size() > 10)
  {
    RCLCPP_FATAL(
      rclcpp::get_logger("KukaFRIHardwareInterface"),
      "A maximum of 10 inputs and outputs can be registered to FRI");
    return CallbackReturn::ERROR;
  }

  for (const auto & state_if : info_.gpios[0].state_interfaces)
  {
    gpio_outputs_.emplace_back(state_if.name, getType(state_if.data_type), robotState());
  }

  for (const auto & command_if : info_.gpios[0].command_interfaces)
  {
    gpio_inputs_.emplace_back(
      command_if.name, getType(command_if.data_type), robotCommand(),
      std::stod(command_if.initial_value));
  }

  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    if (joint.command_interfaces.size() != 4)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("KukaFRIHardwareInterface"), "expecting exactly 2 command interface");
      return CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("KukaFRIHardwareInterface"),
        "expecting POSITION command interface as first");
      return CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[3].name != hardware_interface::HW_IF_EFFORT)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("KukaFRIHardwareInterface"),
        "expecting EFFORT command interface as fourth");
      return CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 3)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("KukaFRIHardwareInterface"), "expecting exactly 3 state interface");
      return CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("KukaFRIHardwareInterface"),
        "expecting POSITION state interface as first");
      return CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_EFFORT)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("KukaFRIHardwareInterface"),
        "expecting EFFORT state interface as second");
      return CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[2].name != hardware_interface::HW_IF_EXTERNAL_TORQUE)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("KukaFRIHardwareInterface"),
        "expecting 'EXTERNAL_TORQUE' state interface as third");
      return CallbackReturn::ERROR;
    }
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn KukaFRIHardwareInterface::on_configure(const rclcpp_lifecycle::State &)
{
  if (!fri_connection_->isConnected())
  {
    if (!fri_connection_->connect(controller_ip_.c_str(), 30000))
    {
      RCLCPP_ERROR(
        rclcpp::get_logger("KukaFRIHardwareInterface"),
        "Could not initialize TCP connection to controller");
      return CallbackReturn::FAILURE;
    }
    RCLCPP_INFO(rclcpp::get_logger("KukaFRIHardwareInterface"), "Successfully connected to FRI");
  }
  else
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("KukaFRIHardwareInterface"),
      "FRI connection was already up before configuration");
    return CallbackReturn::ERROR;
  }

  if (!fri_connection_->setFRIConfig(30200, send_period_ms_, receive_multiplier_))
  {
    RCLCPP_ERROR(rclcpp::get_logger("KukaFRIHardwareInterface"), "Could not set FRI config");
    return CallbackReturn::FAILURE;
  }
  RCLCPP_INFO(rclcpp::get_logger("KukaFRIHardwareInterface"), "Successfully set FRI config");
  return CallbackReturn::SUCCESS;
}

CallbackReturn KukaFRIHardwareInterface::on_cleanup(const rclcpp_lifecycle::State &)
{
  if (!fri_connection_->disconnect())
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("KukaFRIHardwareInterface"),
      "Could not close TCP connection to controller");
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn KukaFRIHardwareInterface::on_activate(const rclcpp_lifecycle::State &)
{
  if (!client_application_.connect(30200, controller_ip_.c_str()))
  {
    RCLCPP_ERROR(rclcpp::get_logger("KukaFRIHardwareInterface"), "Could not connect to controller");
    return CallbackReturn::FAILURE;
  }

  // Start FRI (in monitoring mode)
  if (!fri_connection_->startFRI())
  {
    RCLCPP_ERROR(rclcpp::get_logger("KukaFRIHardwareInterface"), "Could not start FRI");
    return CallbackReturn::FAILURE;
  }
  RCLCPP_INFO(rclcpp::get_logger("KukaFRIHardwareInterface"), "Started FRI");

  // Start commanding mode
  if (!activateControl())
  {
    return CallbackReturn::FAILURE;
  }
  is_active_ = true;
  return CallbackReturn::SUCCESS;
}

CallbackReturn KukaFRIHardwareInterface::on_deactivate(const rclcpp_lifecycle::State &)
{
  if (!this->deactivateControl())
  {
    RCLCPP_ERROR(rclcpp::get_logger("KukaFRIHardwareInterface"), "Could not deactivate control");
    return CallbackReturn::ERROR;
  }

  if (!fri_connection_->endFRI())
  {
    RCLCPP_ERROR(rclcpp::get_logger("KukaFRIHardwareInterface"), "Could not end FRI");
    return CallbackReturn::ERROR;
  }

  client_application_.disconnect();
  is_active_ = false;
  return CallbackReturn::SUCCESS;
}

void KukaFRIHardwareInterface::waitForCommand()
{
  // Update first commmmand based on the actual state
  hw_position_commands_ = hw_position_states_;
  rclcpp::Time stamp = ros_clock_.now();
  updateCommand(stamp);
}

void KukaFRIHardwareInterface::command()
{
  rclcpp::Time stamp = ros_clock_.now();
  if (++receive_counter_ == receive_multiplier_)
  {
    updateCommand(stamp);
    receive_counter_ = 0;
  }
}

hardware_interface::return_type KukaFRIHardwareInterface::read(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  // Read is called in inactive state, check is necessary
  if (!is_active_)
  {
    active_read_ = false;
    RCLCPP_DEBUG(rclcpp::get_logger("KukaFRIHardwareInterface"), "Hardware interface not active");
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    return hardware_interface::return_type::OK;
  }
  active_read_ = true;

  if (!client_application_.client_app_read())
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("KukaFRIHardwareInterface"), "Failed to read data from controller");
    return hardware_interface::return_type::ERROR;
  }

  // get the position and efforts and share them with exposed state interfaces
  const double * position = robotState().getMeasuredJointPosition();
  hw_position_states_.assign(position, position + KUKA::FRI::LBRState::NUMBER_OF_JOINTS);
  const double * torque = robotState().getMeasuredTorque();
  hw_torque_states_.assign(torque, torque + KUKA::FRI::LBRState::NUMBER_OF_JOINTS);
  const double * external_torque = robotState().getExternalTorque();
  hw_ext_torque_states_.assign(
    external_torque, external_torque + KUKA::FRI::LBRState::NUMBER_OF_JOINTS);

  robot_state_.tracking_performance_ = robotState().getTrackingPerformance();
  robot_state_.session_state_ = robotState().getSessionState();
  robot_state_.connection_quality_ = robotState().getConnectionQuality();
  robot_state_.command_mode_ = robotState().getClientCommandMode();
  robot_state_.safety_state_ = robotState().getSafetyState();
  robot_state_.control_mode_ = robotState().getControlMode();
  robot_state_.operation_mode_ = robotState().getOperationMode();
  robot_state_.drive_state_ = robotState().getDriveState();
  robot_state_.overlay_type_ = robotState().getOverlayType();

  for (auto & output : gpio_outputs_)
  {
    output.getValue();
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type KukaFRIHardwareInterface::write(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  // Client app update and read must be called only if read has been called in current cycle
  if (!active_read_)
  {
    RCLCPP_DEBUG(rclcpp::get_logger("KukaFRIHardwareInterface"), "Hardware interface not active");
    return hardware_interface::return_type::OK;
  }

  // Call the appropriate callback for the actual state (e.g. updateCommand)
  //  this updates the command to be sent based on the output of the controller update
  client_application_.client_app_update();

  if (!client_application_.client_app_write() && is_active_)
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("KukaFRIHardwareInterface"), "Could not send command to controller");
    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}

void KukaFRIHardwareInterface::updateCommand(const rclcpp::Time &)
{
  if (!is_active_)
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("KukaFRIHardwareInterface"), "Hardware inactive, exiting updateCommand");
    return;
  }

  switch (static_cast<kuka_drivers_core::ControlMode>(control_mode_))
  {
    case kuka_drivers_core::ControlMode::JOINT_POSITION_CONTROL:
      [[fallthrough]];
    case kuka_drivers_core::ControlMode::JOINT_IMPEDANCE_CONTROL:
    {
      const double * joint_positions_ = hw_position_commands_.data();
      robotCommand().setJointPosition(joint_positions_);
      break;
    }
    case kuka_drivers_core::ControlMode::JOINT_TORQUE_CONTROL:
    {
      const double * joint_torques_ = hw_torque_commands_.data();
      robotCommand().setJointPosition(robotState().getIpoJointPosition());
      robotCommand().setTorque(joint_torques_);
      break;
    }
    default:
      RCLCPP_ERROR(
        rclcpp::get_logger("KukaFRIHardwareInterface"),
        "Unsupported control mode, exiting updateCommand");
      return;
  }

  for (auto & input : gpio_inputs_)
  {
    input.setValue();
  }
}

std::vector<hardware_interface::StateInterface> KukaFRIHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  state_interfaces.emplace_back(
    hardware_interface::FRI_STATE_PREFIX, hardware_interface::SESSION_STATE,
    &robot_state_.session_state_);
  state_interfaces.emplace_back(
    hardware_interface::FRI_STATE_PREFIX, hardware_interface::CONNECTION_QUALITY,
    &robot_state_.connection_quality_);
  state_interfaces.emplace_back(
    hardware_interface::FRI_STATE_PREFIX, hardware_interface::SAFETY_STATE,
    &robot_state_.safety_state_);
  state_interfaces.emplace_back(
    hardware_interface::FRI_STATE_PREFIX, hardware_interface::COMMAND_MODE,
    &robot_state_.command_mode_);
  state_interfaces.emplace_back(
    hardware_interface::FRI_STATE_PREFIX, hardware_interface::CONTROL_MODE,
    &robot_state_.control_mode_);
  state_interfaces.emplace_back(
    hardware_interface::FRI_STATE_PREFIX, hardware_interface::OPERATION_MODE,
    &robot_state_.operation_mode_);
  state_interfaces.emplace_back(
    hardware_interface::FRI_STATE_PREFIX, hardware_interface::DRIVE_STATE,
    &robot_state_.drive_state_);
  state_interfaces.emplace_back(
    hardware_interface::FRI_STATE_PREFIX, hardware_interface::OVERLAY_TYPE,
    &robot_state_.overlay_type_);
  state_interfaces.emplace_back(
    hardware_interface::FRI_STATE_PREFIX, hardware_interface::TRACKING_PERFORMANCE,
    &robot_state_.tracking_performance_);

  // Register I/O outputs (read access)
  for (auto & output : gpio_outputs_)
  {
    state_interfaces.emplace_back(
      hardware_interface::IO_PREFIX, output.getName(), &output.getData());
  }

  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_position_states_[i]);

    state_interfaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_torque_states_[i]);

    state_interfaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_EXTERNAL_TORQUE, &hw_ext_torque_states_[i]);
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
KukaFRIHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  command_interfaces.emplace_back(
    hardware_interface::CONFIG_PREFIX, hardware_interface::CONTROL_MODE, &control_mode_);
  command_interfaces.emplace_back(
    hardware_interface::CONFIG_PREFIX, hardware_interface::RECEIVE_MULTIPLIER,
    &receive_multiplier_);
  command_interfaces.emplace_back(
    hardware_interface::CONFIG_PREFIX, hardware_interface::SEND_PERIOD, &send_period_ms_);

  // Register I/O inputs (write access)
  for (auto & input : gpio_inputs_)
  {
    command_interfaces.emplace_back(
      hardware_interface::IO_PREFIX, input.getName(), &input.getData());
  }

  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_position_commands_[i]);
    command_interfaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_STIFFNESS, &hw_stiffness_commands_[i]);
    command_interfaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_DAMPING, &hw_damping_commands_[i]);
    command_interfaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_torque_commands_[i]);
  }
  return command_interfaces;
}

bool KukaFRIHardwareInterface::activateControl()
{
  if (!fri_connection_->isConnected())
  {
    RCLCPP_ERROR(rclcpp::get_logger("KukaFRIHardwareInterface"), "Not connected");
    return false;
  }

  if (!fri_connection_->activateControl())
  {
    RCLCPP_ERROR(rclcpp::get_logger("KukaFRIHardwareInterface"), "Could not activate control");
    return false;
  }
  return true;
}

bool KukaFRIHardwareInterface::deactivateControl()
{
  if (!fri_connection_->isConnected())
  {
    RCLCPP_ERROR(rclcpp::get_logger("KukaFRIHardwareInterface"), "Not connected");
    return false;
  }

  if (!fri_connection_->deactivateControl())
  {
    RCLCPP_ERROR(rclcpp::get_logger("KukaFRIHardwareInterface"), "Could not deactivate control");
    return false;
  }
  return true;
}

}  // namespace kuka_sunrise_fri_driver

PLUGINLIB_EXPORT_CLASS(
  kuka_sunrise_fri_driver::KukaFRIHardwareInterface, hardware_interface::SystemInterface)
