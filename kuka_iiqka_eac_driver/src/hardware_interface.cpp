// Copyright 2022 Aron Svastits
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
#include <chrono>
#include <stdexcept>
#include <string>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "kuka_drivers_core/control_mode.hpp"
#include "kuka_drivers_core/hardware_interface_types.hpp"

#include "kuka_iiqka_eac_driver/event_observer.hpp"
#include "kuka_iiqka_eac_driver/hardware_interface.hpp"

namespace kuka_eac
{
CallbackReturn KukaEACHardwareInterface::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  // Initialize control mode with 'undefined', which should be changed by the appropriate controller
  // during configuration
  hw_position_states_.resize(info_.joints.size(), 0.0);
  hw_torque_states_.resize(info_.joints.size(), 0.0);
  hw_position_commands_.resize(info_.joints.size(), 0.0);
  hw_torque_commands_.resize(info_.joints.size(), 0.0);
  hw_stiffness_commands_.resize(info_.joints.size(), 30);
  hw_damping_commands_.resize(info_.joints.size(), 0.7);

  signal_config_list_ptr_ =
    std::make_shared<std::vector<kuka::external::control::iiqka::Signal_Configuration>>();

  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    if (joint.command_interfaces.size() != 4)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("KukaEACHardwareInterface"), "expecting exactly 4 command interface");
      return CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("KukaEACHardwareInterface"),
        "expecting 'POSITION' command interface as first");
      return CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[1].name != hardware_interface::HW_IF_STIFFNESS)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("KukaEACHardwareInterface"),
        "expecting 'STIFFNESS' command interface as second");
      return CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[2].name != hardware_interface::HW_IF_DAMPING)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("KukaEACHardwareInterface"),
        "expecting 'DAMPING' command interface as third");
      return CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[3].name != hardware_interface::HW_IF_EFFORT)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("KukaEACHardwareInterface"),
        "expecting 'EFFORT' command interface as fourth");
      return CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 2)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("KukaEACHardwareInterface"), "expecting exactly 2 state interface");
      return CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("KukaEACHardwareInterface"),
        "expecting 'POSITION' state interface as first");
      return CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_EFFORT)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("KukaEACHardwareInterface"),
        "expecting 'EFFORT' state interface as second");
      return CallbackReturn::ERROR;
    }
  }

  // Checking GPIO config structure
  // Check gpio components size
  if (info_.gpios.size() != 1)
  {
    RCLCPP_FATAL(
      rclcpp::get_logger("KukaEACHardwareInterface"), "expecting exactly 1 gpio component");
    return CallbackReturn::ERROR;
  }
  auto & gpio = info_.gpios[0];
  // Check gpio component name
  if (gpio.name != hardware_interface::IO_PREFIX)
  {
    RCLCPP_FATAL(
      rclcpp::get_logger("KukaEACHardwareInterface"), "expecting 'GPIO' gpio component first");
    return ::CallbackReturn::ERROR;
  }
  // Check command interfaces size
  if (
    gpio.command_interfaces.size() >
    kuka::external::control::iiqka::kControlSignal_SignalValueMaxCount)
  {
    RCLCPP_FATAL(
      rclcpp::get_logger("KukaEACHardwareInterface"),
      "Can not be more then %ld command interface for the 'GPIO' component. There is %ld.",
      kuka::external::control::iiqka::kControlSignal_SignalValueMaxCount,
      gpio.command_interfaces.size());
    return CallbackReturn::ERROR;
  }
  // Check state interfaces size
  if (
    gpio.state_interfaces.size() > kuka::external::control::iiqka::kMotionState_SignalValueMaxCount)
  {
    RCLCPP_FATAL(
      rclcpp::get_logger("KukaEACHardwareInterface"),
      "Can not be more then %ld state interface for the 'GPIO' component. There is %ld.",
      kuka::external::control::iiqka::kMotionState_SignalValueMaxCount,
      gpio.state_interfaces.size());
    return CallbackReturn::ERROR;
  }
  if (gpio.command_interfaces.size() > gpio.state_interfaces.size())
  {
    RCLCPP_FATAL(
      rclcpp::get_logger("KukaEACHardwareInterface"),
      "Not every command interface paired with a state interface");
    return CallbackReturn::ERROR;
  }

  hw_signal_commands_.resize(gpio.command_interfaces.size(), 0.0);
  hw_signal_states_.resize(gpio.state_interfaces.size(), 0.0);

  // --- TODO: Delete later ---
  RCLCPP_INFO(
    rclcpp::get_logger("KukaEACHardwareInterface"),
    "GPIOS from xacro: (state interface number: %d)", gpio.state_interfaces.size());
  for (const hardware_interface::ComponentInfo & gpio : info_.gpios)
  {
    for (auto && cmds : gpio.command_interfaces)
    {
      RCLCPP_INFO(
        rclcpp::get_logger("KukaEACHardwareInterface"),
        "GPIO_Name: %s, GPIO_Type: %s, Command Interface - Name: %s data_type: %s, initial_value: "
        "%s, "
        "size: %d",
        gpio.name.c_str(), gpio.type.c_str(), cmds.name.c_str(), cmds.data_type.c_str(),
        cmds.initial_value.c_str(), cmds.size);
    }
    for (auto && states : gpio.state_interfaces)
    {
      RCLCPP_INFO(
        rclcpp::get_logger("KukaEACHardwareInterface"),
        "GPIO_Name: %s, GPIO_Type: %s, State Interface - Name: %s data_type: %s, initial_value: "
        "%s, "
        "size: %d",
        gpio.name.c_str(), gpio.type.c_str(), states.name.c_str(), states.data_type.c_str(),
        states.initial_value.c_str(), states.size);
    }

    for (auto && params : gpio.parameters)
    {
      RCLCPP_INFO(
        rclcpp::get_logger("KukaEACHardwareInterface"),
        "GPIO_Name: %s, GPIO_Type: %s, PARAM - Key: %s, Value: %s", gpio.name.c_str(),
        gpio.type.c_str(), params.first.c_str(), params.second.c_str());
    }
  }
  // --- Until this ---

  if (!SetupRobot())
  {
    return CallbackReturn::FAILURE;
  }
  // Receive Signal Configuration
  if (!GetSignalConfiguration())
  {
    return CallbackReturn::FAILURE;
  }
  // TODO: Delete later
  RCLCPP_INFO(rclcpp::get_logger("KukaEACHardwareInterface"), "Configured signal list:");
  for (auto && signal : *signal_config_list_ptr_)
  {
    RCLCPP_INFO(
      rclcpp::get_logger("KukaEACHardwareInterface"),
      "signal_%ld - name: %s, type: %d, direction: %d", signal.GetSignalId(),
      signal.GetName().c_str(), signal.GetValueType(), signal.GetDirection());
  }

  // signal_config_list_ptr_->at(11).SetSignalToUse(true);

  // Check all the states and commands, verify and add to the used signals
  for (auto && signal_config : *signal_config_list_ptr_)
  {
    signal_config.SetSignalToUse(false);
  }

  for (auto && state : gpio.state_interfaces)
  {
    // search for state in signal_config
    auto signal_config_it = find_if(
      signal_config_list_ptr_->begin(), signal_config_list_ptr_->end(),
      [&state](const kuka::external::control::iiqka::Signal_Configuration & obj)
      { return obj.GetName() == state.name; });

    if (signal_config_it != signal_config_list_ptr_->end())
    {
      // cancle to use the gpio until its confirmed to use
      signal_config_it->SetSignalToUse(false);

      // search for state in command_interfaces
      auto command_it = find_if(
        gpio.command_interfaces.begin(), gpio.command_interfaces.end(),
        [&state](const hardware_interface::InterfaceInfo & obj) { return obj.name == state.name; });

      if (command_it != gpio.command_interfaces.end())
      {
        // Found maching command interface

        // Check if signal is output
        if (
          signal_config_it->GetDirection() !=
          kuka::external::control::iiqka::Signal_Configuration::SignalDirection::OUTPUT)
        {
          RCLCPP_WARN(
            rclcpp::get_logger("KukaEACHardwareInterface"),
            "A command interface named %s found, but the GPIO is an INPUT",
            command_it->name.c_str());
        }
        // TODO (Komaromi): Check for Value type maching both state and command
        else if (false)
        {
          RCLCPP_WARN(rclcpp::get_logger("KukaEACHardwareInterface"), "");
        }
        else
        {
          if (!signal_config_it->IsSignalUsed())
          {
            signal_config_it->SetSignalToUse(true);
          }
        }
      }
      else
      {
        // Check if signal is input
        if (
          signal_config_it->GetDirection() !=
          kuka::external::control::iiqka::Signal_Configuration::SignalDirection::INPUT)
        {
          RCLCPP_WARN(
            rclcpp::get_logger("KukaEACHardwareInterface"),
            "No command interface found for the state iterface named %s, but the GPIO is an OUTPUT",
            state.name.c_str());
        }
        // TODO (Komaromi): Check for Value type maching both state and command
        else if (false)
        {
          RCLCPP_WARN(rclcpp::get_logger("KukaEACHardwareInterface"), "");
        }
        else
        {
          if (!signal_config_it->IsSignalUsed())
          {
            signal_config_it->SetSignalToUse(true);
          }
        }
      }
    }
    else
    {
      RCLCPP_WARN(
        rclcpp::get_logger("KukaEACHardwareInterface"),
        "No signal config found for state called %s", state.name.c_str());
    }
  }

  for (auto && signal_config : *signal_config_list_ptr_)
  {
    if (signal_config.IsSignalUsed())
    {
      RCLCPP_INFO(
        rclcpp::get_logger("KukaEACHardwareInterface"), "signal named %s set to use.",
        signal_config.GetName().c_str());
    }
  }

  RCLCPP_INFO(
    rclcpp::get_logger("KukaEACHardwareInterface"),
    "Init successful with controller ip: %s and client ip: %s",
    info_.hardware_parameters.at("controller_ip").c_str(),
    info_.hardware_parameters.at("client_ip").c_str());

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> KukaEACHardwareInterface::export_state_interfaces()
{
  RCLCPP_DEBUG(rclcpp::get_logger("KukaEACHardwareInterface"), "Export state interfaces");
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_position_states_[i]);

    state_interfaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_torque_states_[i]);
  }

  for (size_t i = 0; i < info_.gpios[0].state_interfaces.size(); i++)
  {
    state_interfaces.emplace_back(
      hardware_interface::IO_PREFIX, info_.gpios[0].state_interfaces[i].name,
      &hw_signal_states_[i]);
  }

  state_interfaces.emplace_back(
    hardware_interface::STATE_PREFIX, hardware_interface::SERVER_STATE, &server_state_);

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
KukaEACHardwareInterface::export_command_interfaces()
{
  RCLCPP_DEBUG(rclcpp::get_logger("KukaEACHardwareInterface"), "Export command interfaces");

  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_position_commands_[i]);

    command_interfaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_torque_commands_[i]);

    command_interfaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_STIFFNESS, &hw_stiffness_commands_[i]);

    command_interfaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_DAMPING, &hw_damping_commands_[i]);
  }

  for (size_t i = 0; i < info_.gpios[0].command_interfaces.size(); i++)
  {
    command_interfaces.emplace_back(
      hardware_interface::IO_PREFIX, info_.gpios[0].command_interfaces[i].name,
      &hw_signal_commands_[i]);
  }

  command_interfaces.emplace_back(
    hardware_interface::CONFIG_PREFIX, hardware_interface::CONTROL_MODE, &hw_control_mode_command_);

  return command_interfaces;
}

CallbackReturn KukaEACHardwareInterface::on_configure(const rclcpp_lifecycle::State &)
{
  // if (!SetupRobot())
  // {
  //   return CallbackReturn::FAILURE;
  // }

  if (!SetupQoS())
  {
    return CallbackReturn::FAILURE;
  }

  RCLCPP_INFO(
    rclcpp::get_logger("KukaEACHardwareInterface"),
    "Set QoS profile with %s consequent and %s packet losses allowed in %s milliseconds",
    info_.hardware_parameters.at("consequent_lost_packets").c_str(),
    info_.hardware_parameters.at("lost_packets_in_timeframe").c_str(),
    info_.hardware_parameters.at("timeframe_ms").c_str());

  return CallbackReturn::SUCCESS;
}

CallbackReturn KukaEACHardwareInterface::on_activate(const rclcpp_lifecycle::State &)
{
  kuka::external::control::Status create_event_observer =
    robot_ptr_->RegisterEventHandler(std::make_unique<KukaEACEventObserver>(this));
  if (create_event_observer.return_code == kuka::external::control::ReturnCode::ERROR)
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("KukaEACHardwareInterface"),
      "Creating event observer failed, error message: %s", create_event_observer.message);
  }

  kuka::external::control::Status start_control = robot_ptr_->StartControlling(
    static_cast<kuka::external::control::ControlMode>(hw_control_mode_command_));
  if (start_control.return_code == kuka::external::control::ReturnCode::ERROR)
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("KukaEACHardwareInterface"),
      "Starting external control failed, error message: %s", start_control.message);
    return CallbackReturn::FAILURE;
  }

  prev_control_mode_ = static_cast<kuka_drivers_core::ControlMode>(hw_control_mode_command_);

  RCLCPP_INFO(
    rclcpp::get_logger("KukaEACHardwareInterface"),
    "External control session started successfully");

  stop_requested_ = false;
  cycle_count_ = 0;
  return CallbackReturn::SUCCESS;
}

CallbackReturn KukaEACHardwareInterface::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger("KukaEACHardwareInterface"), "Deactivating hardware interface");

  stop_requested_ = true;

  return CallbackReturn::SUCCESS;
}

return_type KukaEACHardwareInterface::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  // Bigger timeout blocks controller configuration
  auto start = std::chrono::high_resolution_clock::now();
  kuka::external::control::Status receive_state =
    robot_ptr_->ReceiveMotionState(std::chrono::milliseconds(10));
  auto end = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();

  // RCLCPP_INFO(
  //   rclcpp::get_logger("KukaEACHardwareInterface"), "Duration: %ld, Receive state: %s", duration,
  //   receive_state.message);

  if ((msg_received_ = receive_state.return_code == kuka::external::control::ReturnCode::OK))
  {
    // RCLCPP_INFO(
    //   rclcpp::get_logger("KukaEACHardwareInterface"), "Duration: %ld, Receive state: %s",
    //   duration, receive_state.message);
    auto req_message = robot_ptr_->GetLastMotionState();
    std::copy(
      req_message.GetMeasuredPositions().begin(), req_message.GetMeasuredPositions().end(),
      hw_position_states_.begin());
    std::copy(
      req_message.GetMeasuredTorques().begin(), req_message.GetMeasuredTorques().end(),
      hw_torque_states_.begin());

    auto signal_values_size = req_message.GetSignalValuesSize();
    // // std::copy(
    // //   req_message.GetSignalValues().begin(),
    // //   req_message.GetSignalValues().end(), signal_values_.begin());
    RCLCPP_INFO(
      rclcpp::get_logger("KukaEACHardwareInterface"), "Signal size: %d", signal_values_size);
    for (int i = 0; i < signal_values_size; i++)
    {
      RCLCPP_INFO(
        rclcpp::get_logger("KukaEACHardwareInterface"), "Signal_%d - Value: %d, type: %d",
        req_message.GetSignalValues().at(i)->GetSignalID(),
        req_message.GetSignalValues().at(i)->GetBoolValue(),
        req_message.GetSignalValues().at(i)->GetValueType());
    }

    if (signal_values_size != hw_signal_states_.size())
    {
      // RCLCPP_WARN(
      //   rclcpp::get_logger("KukaEACHardwareInterface"),
      //   "Got GPIO size is not equal with expected gpio state interface size, using smaller
      //   number");
    }
    auto & gpio_values = req_message.GetSignalValues();
    for (size_t i = 0; i < hw_signal_states_.size(); i++)
    {
      switch (gpio_values.at(i)->GetValueType())
      {
        case kuka::external::control::BaseSignalValue::SignalValueType::BOOL_VALUE:
          hw_signal_states_[i] = static_cast<double>(gpio_values[i]->GetBoolValue());
          break;
        case kuka::external::control::BaseSignalValue::SignalValueType::DOUBLE_VALUE:
          hw_signal_states_[i] = static_cast<double>(gpio_values[i]->GetDoubleValue());
          break;
        case kuka::external::control::BaseSignalValue::SignalValueType::RAW_VALUE:
          hw_signal_states_[i] = static_cast<double>(gpio_values[i]->GetRawValue());
          break;
        case kuka::external::control::BaseSignalValue::SignalValueType::LONG_VALUE:
          hw_signal_states_[i] = static_cast<double>(gpio_values[i]->GetLongValue());
          break;
        default:
          RCLCPP_ERROR(
            rclcpp::get_logger("KukaEACHardwareInterface"),
            "No signal value type found. (Should be dead code)");
      }
    }

    for (auto && signal : hw_signal_states_)
    {
      RCLCPP_INFO(rclcpp::get_logger("KukaEACHardwareInterface"), "Signal value: %f", signal);
    }

    // UpdateSignalStates();

    if (cycle_count_ == 0)
    {
      std::copy(
        hw_position_states_.begin(), hw_position_states_.end(), hw_position_commands_.begin());
    }
    cycle_count_++;
  }

  // Modify state interface only in read
  std::lock_guard<std::mutex> lk(event_mutex_);
  server_state_ = static_cast<double>(last_event_);
  return return_type::OK;
}

return_type KukaEACHardwareInterface::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  // If control is not started or a request is missed, do not send back anything
  if (!msg_received_)
  {
    return return_type::OK;
  }

  // Creating change in signal value
  // for (auto && signal_value : signal_values_)
  // {
  //   if (
  //     signal_value->GetValueType() ==
  //     kuka::external::control::iiqka::SignalValue::SignalValueType::BOOL_VALUE)
  //   {
  //     bool value = !signal_value->GetBoolValue();
  //     signal_value->SetBoolValue(value);
  //     RCLCPP_INFO(
  //       rclcpp::get_logger("KukaEACHardwareInterface"), "Signal_%d - Value: %d",
  //       signal_value->GetSignalID(), signal_value->GetBoolValue());
  //   }
  // }

  // // TODO(Komaromi): Remove this
  // if (iter == 0)
  // {
  //   for (auto && signal_value : signal_values_)
  //   {
  //     RCLCPP_INFO(
  //       rclcpp::get_logger("KukaEACHardwareInterface"), "Signal_%d - Value: %d",
  //       signal_value.GetSignalID(), signal_value.GetBoolValue());
  //   }
  // }
  // else if (iter >= 251)
  // {
  //   for (auto && signal_value : signal_values_)
  //   {
  //     RCLCPP_INFO(
  //       rclcpp::get_logger("KukaEACHardwareInterface"), "Signal_%d - Value: %d",
  //       signal_value.GetSignalID(), signal_value.GetBoolValue());
  //   }
  //   iter = 0;
  // }
  // iter++;

  robot_ptr_->GetControlSignal().AddJointPositionValues(
    hw_position_commands_.begin(), hw_position_commands_.end());
  robot_ptr_->GetControlSignal().AddTorqueValues(
    hw_torque_commands_.begin(), hw_torque_commands_.end());
  robot_ptr_->GetControlSignal().AddStiffnessAndDampingValues(
    hw_stiffness_commands_.begin(), hw_stiffness_commands_.end(), hw_damping_commands_.begin(),
    hw_damping_commands_.end());

  // std::vector<double> command = {1.0};
  // for (auto && signal : command)
  // {
  //   RCLCPP_INFO(rclcpp::get_logger("KukaEACHardwareInterface"), "Signal value: %f", signal);
  // }
  robot_ptr_->GetControlSignal().AddSignalValues(
    robot_ptr_->GetLastMotionState().GetSignalValues(), hw_signal_commands_.begin(),
    hw_signal_commands_.end());

  for (auto && signal : robot_ptr_->GetControlSignal().GetSignalValues())
  {
    RCLCPP_INFO(
      rclcpp::get_logger("KukaEACHardwareInterface"), "Signal_%d - Value: %d",
      signal->GetBoolValue(), signal->GetBoolValue());
  }
  // for (auto && signal : hw_signal_commands_)
  // {
  //   RCLCPP_INFO(rclcpp::get_logger("KukaEACHardwareInterface"), "Signal value: %f", signal);
  // }

  kuka::external::control::Status send_reply;
  if (stop_requested_)
  {
    RCLCPP_INFO(rclcpp::get_logger("KukaEACHardwareInterface"), "Sending stop signal");
    send_reply = robot_ptr_->StopControlling();
  }
  else if (
    static_cast<kuka_drivers_core::ControlMode>(hw_control_mode_command_) != prev_control_mode_)
  {
    RCLCPP_INFO(rclcpp::get_logger("KukaEACHardwareInterface"), "Requesting control mode switch");
    send_reply = robot_ptr_->SwitchControlMode(
      static_cast<kuka::external::control::ControlMode>(hw_control_mode_command_));
    prev_control_mode_ = static_cast<kuka_drivers_core::ControlMode>(hw_control_mode_command_);
  }
  else
  {
    send_reply = robot_ptr_->SendControlSignal();
  }
  if (send_reply.return_code != kuka::external::control::ReturnCode::OK)
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("KukaEACHardwareInterface"), "Send reply failed, error message: %s",
      send_reply.message);
    throw std::runtime_error("Error sending reply");
  }
  RCLCPP_INFO(rclcpp::get_logger("KukaEACHardwareInterface"), "Reply sent: %s", send_reply.message);
  return return_type::OK;
}

bool KukaEACHardwareInterface::SetupRobot()
{
  kuka::external::control::iiqka::Configuration config;

  config.client_ip_address = info_.hardware_parameters.at("client_ip");
  config.koni_ip_address = info_.hardware_parameters.at("controller_ip");

  config.is_secure = false;
  config.dof = info_.joints.size();
  config.gpio_command_size = info_.gpios[0].command_interfaces.size();
  config.gpio_state_size = info_.gpios[0].state_interfaces.size();

  robot_ptr_ = std::make_unique<kuka::external::control::iiqka::Robot>(config);

  kuka::external::control::Status setup = robot_ptr_->Setup();

  if (setup.return_code != kuka::external::control::ReturnCode::OK)
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("KukaEACHardwareInterface"), "Setup failed, error message: %s",
      setup.message);
    return false;
  }

  return true;
}

bool KukaEACHardwareInterface::SetupQoS()
{
  kuka::external::control::iiqka::QoS_Configuration qos_config;
  qos_config.packet_loss_in_timeframe_limit =
    std::stoi(info_.hardware_parameters.at("lost_packets_in_timeframe"));
  qos_config.consecutive_packet_loss_limit =
    std::stoi(info_.hardware_parameters.at("consequent_lost_packets"));
  qos_config.timeframe_ms = std::stoi(info_.hardware_parameters.at("timeframe_ms"));

  kuka::external::control::Status set_qos_status = robot_ptr_->SetQoSProfile(qos_config);

  if (set_qos_status.return_code != kuka::external::control::ReturnCode::OK)
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("KukaEACHardwareInterface"), "QoS configuration failed, error message: %s",
      set_qos_status.message);
    return false;
  }

  return true;
}

bool KukaEACHardwareInterface::GetSignalConfiguration()
{
  RCLCPP_INFO(rclcpp::get_logger("KukaEACHardwareInterface"), "SignalConfig");
  kuka::external::control::Status get_signal_config_status =
    robot_ptr_->GetSignalConfiguration(signal_config_list_ptr_);

  if (get_signal_config_status.return_code != kuka::external::control::ReturnCode::OK)
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("KukaEACHardwareInterface"),
      "Failed to receive signal configuration, error message: %s",
      get_signal_config_status.message);
    return false;
  }
  else
  {
    RCLCPP_INFO(
      rclcpp::get_logger("KukaEACHardwareInterface"), "SignalConfig received, message: %s",
      get_signal_config_status.message);
  }

  return true;
}

bool KukaEACHardwareInterface::UpdateSignalStates()
{
  for (size_t i = 0; i < kuka::external::control::iiqka::kMotionState_SignalValueMaxCount; i++)
  {
    switch (signal_values_[i]->GetValueType())
    {
      case kuka::external::control::BaseSignalValue::SignalValueType::BOOL_VALUE:
        hw_signal_states_[i] = static_cast<double>(signal_values_[i]->GetBoolValue());
        break;
      case kuka::external::control::BaseSignalValue::SignalValueType::DOUBLE_VALUE:
        hw_signal_states_[i] = static_cast<double>(signal_values_[i]->GetDoubleValue());
        break;
      case kuka::external::control::BaseSignalValue::SignalValueType::RAW_VALUE:
        hw_signal_states_[i] = static_cast<double>(signal_values_[i]->GetRawValue());
        break;
      case kuka::external::control::BaseSignalValue::SignalValueType::LONG_VALUE:
        hw_signal_states_[i] = static_cast<double>(signal_values_[i]->GetLongValue());
        break;

      default:
        RCLCPP_ERROR(
          rclcpp::get_logger("KukaEACHardwareInterface"),
          "No signal value type found. (Should be dead code)");
        return false;
    }
  }
  return true;
}

void KukaEACHardwareInterface::set_server_event(kuka_drivers_core::HardwareEvent event)
{
  std::lock_guard<std::mutex> lk(event_mutex_);
  last_event_ = event;
}
}  // namespace kuka_eac

PLUGINLIB_EXPORT_CLASS(kuka_eac::KukaEACHardwareInterface, hardware_interface::SystemInterface)
