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
  fri_connection_ =
    std::make_shared<FRIConnection>([this] { this->onError(); }, [this] { this->onError(); });

  if (hardware_interface::SystemInterface::on_init(system_info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }
  controller_ip_ = info_.hardware_parameters.at("controller_ip");
  client_ip_ = info_.hardware_parameters.at("client_ip");
  client_port_ = std::stoi(info_.hardware_parameters.at("client_port"));

  hw_position_states_.resize(info_.joints.size());
  hw_position_commands_.resize(info_.joints.size());
  hw_stiffness_commands_.resize(info_.joints.size());
  hw_damping_commands_.resize(info_.joints.size());
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
        rclcpp::get_logger("KukaFRIHardwareInterface"), "expecting exactly 4 command interface");
      return CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("KukaFRIHardwareInterface"),
        "expecting 'POSITION' command interface as first");
      return CallbackReturn::ERROR;
    }
    if (joint.command_interfaces[1].name != hardware_interface::HW_IF_STIFFNESS)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("KukaFRIHardwareInterface"),
        "expecting 'STIFFNESS' command interface as second");
      return CallbackReturn::ERROR;
    }
    if (joint.command_interfaces[2].name != hardware_interface::HW_IF_DAMPING)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("KukaFRIHardwareInterface"),
        "expecting 'DAMPING' command interface as third");
      return CallbackReturn::ERROR;
    }
    if (joint.command_interfaces[3].name != hardware_interface::HW_IF_EFFORT)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("KukaFRIHardwareInterface"),
        "expecting 'EFFORT' command interface as fourth");
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
        "expecting 'POSITION' state interface as first");
      return CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_EFFORT)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("KukaFRIHardwareInterface"),
        "expecting 'EFFORT' state interface as second");
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

  RCLCPP_INFO(
    rclcpp::get_logger("KukaFRIHardwareInterface"),
    "Init successful with controller ip: %s and client ip: %s:%i", controller_ip_.c_str(),
    client_ip_.c_str(), client_port_);

  return CallbackReturn::SUCCESS;
}

CallbackReturn KukaFRIHardwareInterface::on_configure(const rclcpp_lifecycle::State &)
{
  // Set up UDP connection (UDP replier on client)
  if (!client_application_.connect(30200, controller_ip_.c_str()))
  {
    RCLCPP_ERROR(rclcpp::get_logger("KukaFRIHardwareInterface"), "Could not set up UDP connection");
    return CallbackReturn::FAILURE;
  }

  // Set up TCP connection necessary for configuration
  if (!fri_connection_->connect(controller_ip_.c_str(), TCP_SERVER_PORT))
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("KukaFRIHardwareInterface"),
      "Could not initialize TCP connection to controller");
    return CallbackReturn::FAILURE;
  }
  RCLCPP_INFO(
    rclcpp::get_logger("KukaFRIHardwareInterface"), "Successfully connected to FRI application");

  return CallbackReturn::SUCCESS;
}

CallbackReturn KukaFRIHardwareInterface::on_cleanup(const rclcpp_lifecycle::State &)
{
  client_application_.disconnect();

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
  // Set control mode before starting motion - not even the impedance attributes can be changed in
  // active state
  switch (static_cast<kuka_drivers_core::ControlMode>(control_mode_))
  {
    case kuka_drivers_core::ControlMode::JOINT_POSITION_CONTROL:
      fri_connection_->setPositionControlMode();
      fri_connection_->setClientCommandMode(ClientCommandModeID::POSITION_COMMAND_MODE);
      break;
    case kuka_drivers_core::ControlMode::JOINT_IMPEDANCE_CONTROL:
      fri_connection_->setJointImpedanceControlMode(hw_stiffness_commands_, hw_damping_commands_);
      fri_connection_->setClientCommandMode(ClientCommandModeID::POSITION_COMMAND_MODE);
      break;
    case kuka_drivers_core::ControlMode::JOINT_TORQUE_CONTROL:
      fri_connection_->setJointImpedanceControlMode(
        std::vector<double>(DOF, 0.0), std::vector<double>(DOF, 0.0));
      fri_connection_->setClientCommandMode(ClientCommandModeID::TORQUE_COMMAND_MODE);
      break;

    default:
      RCLCPP_ERROR(rclcpp::get_logger("KukaFRIHardwareInterface"), "Unsupported control mode");
      return CallbackReturn::ERROR;
  }

  // Start FRI (in monitoring mode)
  if (!fri_connection_->startFRI())
  {
    RCLCPP_ERROR(rclcpp::get_logger("KukaFRIHardwareInterface"), "Could not start FRI");
    return CallbackReturn::FAILURE;
  }
  RCLCPP_INFO(rclcpp::get_logger("KukaFRIHardwareInterface"), "Started FRI");

  // Switch to commanding mode
  if (!fri_connection_->activateControl())
  {
    RCLCPP_ERROR(rclcpp::get_logger("KukaFRIHardwareInterface"), "Could not activate control");
    return CallbackReturn::FAILURE;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn KukaFRIHardwareInterface::on_deactivate(const rclcpp_lifecycle::State &)
{
  if (!fri_connection_->deactivateControl())
  {
    RCLCPP_ERROR(rclcpp::get_logger("KukaFRIHardwareInterface"), "Could not deactivate control");
    return CallbackReturn::ERROR;
  }

  if (!fri_connection_->endFRI())
  {
    RCLCPP_ERROR(rclcpp::get_logger("KukaFRIHardwareInterface"), "Could not end FRI");
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

void KukaFRIHardwareInterface::waitForCommand()
{
  // In COMMANDING_WAIT state, the controller and client sync commanded positions
  // Therefore the control signal should not be modified in this callback
  // TODO(Svastits): check for torque/impedance mode, where state can change
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
  if ((active_read_ = client_application_.client_app_read() == true))
  {
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
  }

  // Modify state interface only in read
  std::lock_guard<std::mutex> lk(event_mutex_);
  server_state_ = static_cast<double>(last_event_);
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type KukaFRIHardwareInterface::write(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  // Client app update and read must be called only if read has been called in current cycle
  // FRI configuration can be modified only before cyclic communication is started
  if (!active_read_)
  {
    if (FRIConfigChanged())
    {
      // FRI config cannot be set during hardware interface configuration, as the controller cannot
      // modify the cmd interface until the hardware reached the configured state
      if (!fri_connection_->setFRIConfig(
            client_ip_, client_port_, static_cast<int>(send_period_ms_),
            static_cast<int>(receive_multiplier_)))
      {
        RCLCPP_ERROR(rclcpp::get_logger("KukaFRIHardwareInterface"), "Could not set FRI config");
        return hardware_interface::return_type::ERROR;
      }
      RCLCPP_INFO(rclcpp::get_logger("KukaFRIHardwareInterface"), "Successfully set FRI config");
    }

    return hardware_interface::return_type::OK;
  }

  // Call the appropriate callback for the actual state (e.g. updateCommand)
  //  in active state this updates the command to be sent based on the command interfaces
  client_application_.client_app_update();

  if (!client_application_.client_app_write())
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("KukaFRIHardwareInterface"), "Could not send command to controller");
    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}

void KukaFRIHardwareInterface::updateCommand(const rclcpp::Time &)
{
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
      const double * joint_pos = robotState().getMeasuredJointPosition();
      std::array<double, DOF> joint_pos_corr;
      std::copy(joint_pos, joint_pos + DOF, joint_pos_corr.begin());
      activateFrictionCompensation(joint_pos_corr.data());
      robotCommand().setJointPosition(joint_pos_corr.data());
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

  state_interfaces.emplace_back(
    hardware_interface::STATE_PREFIX, hardware_interface::SERVER_STATE, &server_state_);
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

// Friction compensation is activated only if the commanded and measured joint positions differ
void KukaFRIHardwareInterface::activateFrictionCompensation(double * values) const
{
  for (int i = 0; i < DOF; i++)
  {
    values[i] -= (values[i] / fabs(values[i]) * 0.1);
  }
}

void KukaFRIHardwareInterface::onError()
{
  std::lock_guard<std::mutex> lk(event_mutex_);
  last_event_ = kuka_drivers_core::HardwareEvent::ERROR;
  RCLCPP_ERROR(
    rclcpp::get_logger("KukaFRIHardwareInterface"), "External control stopped by an error");
}

bool KukaFRIHardwareInterface::FRIConfigChanged()
{
  // FRI config values are integers and only stored as doubles due to hwif constraints
  if (
    prev_period_ == static_cast<int>(send_period_ms_) &&
    prev_multiplier_ == static_cast<int>(receive_multiplier_))
  {
    return false;
  }
  else
  {
    prev_period_ = static_cast<int>(send_period_ms_);
    prev_multiplier_ = static_cast<int>(receive_multiplier_);
    return true;
  }
}
}  // namespace kuka_sunrise_fri_driver

PLUGINLIB_EXPORT_CLASS(
  kuka_sunrise_fri_driver::KukaFRIHardwareInterface, hardware_interface::SystemInterface)
