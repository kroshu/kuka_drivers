// Copyright 2022 Áron Svastits
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
#include <stdexcept>
#include <string>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "kuka_drivers_core/hardware_interface_types.hpp"

#include "nanopb-helpers/nanopb_serialization_helper.h"

#include "kuka_iiqka_eac_driver/hardware_interface.hpp"

using namespace kuka::ecs::v1;  // NOLINT

using os::core::udp::communication::Socket;

namespace kuka_eac
{
CallbackReturn KukaEACHardwareInterface::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  udp_replier_ = std::make_unique<os::core::udp::communication::Replier>(
    os::core::udp::communication::SocketAddress(info_.hardware_parameters.at("client_ip"), 44444));

#ifdef NON_MOCK_SETUP

  stub_ = ExternalControlService::NewStub(grpc::CreateChannel(
    info_.hardware_parameters.at("controller_ip") + ":49335", grpc::InsecureChannelCredentials()));

#endif
  // Initialize control mode with 'undefined', which should be changed by the appropriate controller
  // during configuration
  hw_control_mode_command_ = 0;
  hw_position_states_.resize(info_.joints.size(), 0.0);
  hw_torque_states_.resize(info_.joints.size(), 0.0);
  hw_position_commands_.resize(info_.joints.size(), 0.0);
  hw_torque_commands_.resize(info_.joints.size(), 0.0);
  hw_stiffness_commands_.resize(info_.joints.size(), 30);
  hw_damping_commands_.resize(info_.joints.size(), 0.7);
  motion_state_external_.header.ipoc = 0;
  control_signal_ext_.has_header = true;
  control_signal_ext_.has_control_signal = true;
  control_signal_ext_.control_signal.has_joint_command = true;
  control_signal_ext_.control_signal.joint_command.values_count = info_.joints.size();
  control_signal_ext_.control_signal.has_joint_torque_command = true;
  control_signal_ext_.control_signal.joint_torque_command.values_count = info_.joints.size();
  control_signal_ext_.control_signal.has_joint_attributes = true;
  control_signal_ext_.control_signal.joint_attributes.stiffness_count = info_.joints.size();
  control_signal_ext_.control_signal.joint_attributes.damping_count = info_.joints.size();
#ifdef NON_MOCK_SETUP
  if (udp_replier_->Setup() != Socket::ErrorCode::kSuccess)
  {
    RCLCPP_ERROR(rclcpp::get_logger("KukaEACHardwareInterface"), "Could not setup udp replier");
    return CallbackReturn::FAILURE;
  }
#else
  // Start from home position in mock mode
  hw_position_commands_[1] = -90 * (M_PI / 180);
  hw_position_commands_[2] = 90 * (M_PI / 180);
  hw_position_commands_[4] = 90 * (M_PI / 180);
#endif

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

  command_interfaces.emplace_back(
    hardware_interface::CONFIG_PREFIX, hardware_interface::CONTROL_MODE, &hw_control_mode_command_);

  return command_interfaces;
}

CallbackReturn KukaEACHardwareInterface::on_configure(const rclcpp_lifecycle::State &)
{
#ifdef NON_MOCK_SETUP
  SetQoSProfileRequest request;
  SetQoSProfileResponse response;
  grpc::ClientContext context;

  request.add_qos_profiles();

  request.mutable_qos_profiles()
    ->at(0)
    .mutable_rt_packet_loss_profile()
    ->set_consequent_lost_packets(
      std::stoi(info_.hardware_parameters.at("consequent_lost_packets")));
  request.mutable_qos_profiles()
    ->at(0)
    .mutable_rt_packet_loss_profile()
    ->set_lost_packets_in_timeframe(
      std::stoi(info_.hardware_parameters.at("lost_packets_in_timeframe")));
  request.mutable_qos_profiles()->at(0).mutable_rt_packet_loss_profile()->set_timeframe_ms(
    std::stoi(info_.hardware_parameters.at("timeframe_ms")));

  if (stub_->SetQoSProfile(&context, request, &response).error_code() != grpc::StatusCode::OK)
  {
    RCLCPP_ERROR(rclcpp::get_logger("KukaEACHardwareInterface"), "SetQoSProfile failed");
    return CallbackReturn::FAILURE;
  }

  RCLCPP_INFO(
    rclcpp::get_logger("KukaEACHardwareInterface"),
    "Set QoS profile with %s consequent and %s packet losses allowed in %s milliseconds",
    info_.hardware_parameters.at("consequent_lost_packets").c_str(),
    info_.hardware_parameters.at("lost_packets_in_timeframe").c_str(),
    info_.hardware_parameters.at("timeframe_ms").c_str());
#endif
  return CallbackReturn::SUCCESS;
}

CallbackReturn KukaEACHardwareInterface::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger("KukaEACHardwareInterface"), "Connecting to robot . . .");
  // Reset timeout to catch first tick message
  receive_timeout_ = std::chrono::milliseconds(100);
  control_signal_ext_.control_signal.stop_ipo = false;
#ifdef NON_MOCK_SETUP
  if (context_ != nullptr)
  {
    context_->TryCancel();
  }
#endif
  if (observe_thread_.joinable())
  {
    observe_thread_.join();
  }

#ifdef NON_MOCK_SETUP
  observe_thread_ = std::thread(&KukaEACHardwareInterface::ObserveControl, this);

  OpenControlChannelRequest request;
  OpenControlChannelResponse response;
  grpc::ClientContext context;

  request.set_ip_address(info_.hardware_parameters.at("client_ip"));
  request.set_timeout(5000);
  request.set_cycle_time(4);
  request.set_external_control_mode(
    kuka::motion::external::ExternalControlMode(hw_control_mode_command_));
  RCLCPP_INFO(
    rclcpp::get_logger("KukaEACHardwareInterface"), "Starting control in %s",
    kuka::motion::external::ExternalControlMode_Name(static_cast<int>(hw_control_mode_command_))
      .c_str());

  auto ret = stub_->OpenControlChannel(&context, request, &response);
  if (ret.error_code() != grpc::StatusCode::OK)
  {
    RCLCPP_ERROR(rclcpp::get_logger("KukaEACHardwareInterface"), "%s", ret.error_message().c_str());
    return CallbackReturn::FAILURE;
  }
#endif

  return CallbackReturn::SUCCESS;
}

CallbackReturn KukaEACHardwareInterface::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger("KukaEACHardwareInterface"), "Deactivating");

  control_signal_ext_.control_signal.stop_ipo = true;
  while (is_active_)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

#ifdef NON_MOCK_SETUP
  if (context_ != nullptr)
  {
    context_->TryCancel();
  }
#endif
  if (observe_thread_.joinable())
  {
    observe_thread_.join();
  }
  return CallbackReturn::SUCCESS;
}

return_type KukaEACHardwareInterface::read(const rclcpp::Time &, const rclcpp::Duration &)
{
#ifndef NON_MOCK_SETUP
  std::this_thread::sleep_for(std::chrono::microseconds(3900));
  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    hw_position_states_[i] = hw_position_commands_[i];
  }
  return return_type::OK;
#endif

  if (!is_active_)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
    msg_received_ = false;

    return return_type::OK;
  }

  if (udp_replier_->ReceiveRequestOrTimeout(receive_timeout_) == Socket::ErrorCode::kSuccess)
  {
    auto req_message = udp_replier_->GetRequestMessage();

    if (!nanopb::Decode<nanopb::kuka::ecs::v1::MotionStateExternal>(
          req_message.first, req_message.second, motion_state_external_))
    {
      RCLCPP_INFO(rclcpp::get_logger("KukaEACHardwareInterface"), "Decoding request failed");
      throw std::runtime_error("Decoding request failed");
    }
    control_signal_ext_.header.ipoc = motion_state_external_.header.ipoc;

    for (size_t i = 0; i < info_.joints.size(); i++)
    {
      hw_position_states_[i] = motion_state_external_.motion_state.measured_positions.values[i];
      hw_torque_states_[i] = motion_state_external_.motion_state.measured_torques.values[i];
      // This is necessary, as joint trajectory controller is initialized with 0 command values
      if (!msg_received_ && motion_state_external_.header.ipoc == 0)
      {
        hw_position_commands_[i] = hw_position_states_[i];
      }
    }

    if (motion_state_external_.motion_state.ipo_stopped)
    {
      RCLCPP_INFO(rclcpp::get_logger("KukaEACHardwareInterface"), "Motion stopped");
    }
    msg_received_ = true;
    receive_timeout_ = std::chrono::milliseconds(6);
  }
  else
  {
    RCLCPP_WARN(rclcpp::get_logger("KukaEACHardwareInterface"), "Request was missed");
    RCLCPP_WARN(
      rclcpp::get_logger("KukaEACHardwareInterface"), "Previous ipoc: %i",
      motion_state_external_.header.ipoc);
    msg_received_ = false;
  }
  return return_type::OK;
}

return_type KukaEACHardwareInterface::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  // If control is not started or a request is missed, do not send back anything
  if (!msg_received_)
  {
    return return_type::OK;
  }

  // TODO(Svastits): should we separate control modes somehow?
  std::copy(
    hw_position_commands_.begin(), hw_position_commands_.end(),
    control_signal_ext_.control_signal.joint_command.values);
  std::copy(
    hw_torque_commands_.begin(), hw_torque_commands_.end(),
    control_signal_ext_.control_signal.joint_torque_command.values);
  std::copy(
    hw_stiffness_commands_.begin(), hw_stiffness_commands_.end(),
    control_signal_ext_.control_signal.joint_attributes.stiffness);
  std::copy(
    hw_damping_commands_.begin(), hw_damping_commands_.end(),
    control_signal_ext_.control_signal.joint_attributes.damping);

  control_signal_ext_.control_signal.control_mode =
    kuka_motion_external_ExternalControlMode(static_cast<int>(hw_control_mode_command_));

  auto encoded_bytes = nanopb::Encode<nanopb::kuka::ecs::v1::ControlSignalExternal>(
    control_signal_ext_, out_buff_arr_, sizeof(out_buff_arr_));
  if (encoded_bytes < 0)
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("KukaEACHardwareInterface"),
      "Encoding of control signal to out_buffer failed.");
    throw std::runtime_error("Encoding of control signal to out_buffer failed.");
  }

  if (udp_replier_->SendReply(out_buff_arr_, encoded_bytes) != Socket::ErrorCode::kSuccess)
  {
    RCLCPP_ERROR(rclcpp::get_logger("KukaEACHardwareInterface"), "Error sending reply");
    throw std::runtime_error("Error sending reply");
  }
  return return_type::OK;
}

void KukaEACHardwareInterface::ObserveControl()
{
#ifdef NON_MOCK_SETUP
  RCLCPP_INFO(rclcpp::get_logger("KukaEACHardwareInterface"), "Observe control");
  context_ = std::make_unique<::grpc::ClientContext>();
  ObserveControlStateRequest obs_control;
  std::unique_ptr<grpc::ClientReader<CommandState>> reader(
    stub_->ObserveControlState(context_.get(), obs_control));

  CommandState response;

  while (reader->Read(&response))
  {
    command_state_ = response;
    RCLCPP_INFO(
      rclcpp::get_logger("KukaEACHardwareInterface"),
      "Event streamed from external control service: %s",
      CommandEvent_Name(response.event()).c_str());

    switch (static_cast<int>(response.event()))
    {
      case CommandEvent::COMMAND_READY:
        RCLCPP_INFO(rclcpp::get_logger("KukaEACHardwareInterface"), "Command accepted");
        break;
      case CommandEvent::SAMPLING:
        RCLCPP_INFO(rclcpp::get_logger("KukaEACHardwareInterface"), "External control is active");
        is_active_ = true;
        break;
      case CommandEvent::CONTROL_MODE_SWITCH:
        RCLCPP_INFO(
          rclcpp::get_logger("KukaEACHardwareInterface"), "Control mode switch is in progress");
        is_active_ = false;
        break;
      case CommandEvent::STOPPED:
        RCLCPP_INFO(rclcpp::get_logger("KukaEACHardwareInterface"), "External control finished");
        is_active_ = false;
        break;
      case CommandEvent::ERROR:
        RCLCPP_ERROR(
          rclcpp::get_logger("KukaEACHardwareInterface"), "External control stopped by an error");
        RCLCPP_ERROR(rclcpp::get_logger("KukaEACHardwareInterface"), response.message().c_str());
        is_active_ = false;
        break;
      default:
        break;
    }
  }
#endif
}

}  // namespace kuka_eac

PLUGINLIB_EXPORT_CLASS(kuka_eac::KukaEACHardwareInterface, hardware_interface::SystemInterface)
