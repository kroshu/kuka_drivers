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

#include "kuka_rox_hw_interface/rox_hardware_interface.hpp"

#include <sched.h>
#include <sys/mman.h>
#include <grpcpp/create_channel.h>

#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "nanopb-helpers/nanopb_serialization_helper.h"

using namespace kuka::ecs::v1;  // NOLINT

using os::core::udp::communication::UDPSocket;

namespace kuka_rox
{
CallbackReturn KukaRoXHardwareInterface::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  udp_replier_ = std::make_unique<os::core::udp::communication::UDPReplier>(
    os::core::udp::communication::SocketAddress(
      info_.hardware_parameters.at("client_ip"), 44444));

#ifdef NON_MOCK_SETUP

  stub_ =
    ExternalControlService::NewStub(
    grpc::CreateChannel(
      info_.hardware_parameters.at("controller_ip") + ":49335",
      grpc::InsecureChannelCredentials()));

#endif
  hw_states_.resize(info_.joints.size(), 0.0);
  hw_position_commands_.resize(info_.joints.size(), 0.0);
  hw_torque_commands_.resize(info_.joints.size(), 0.0);
  hw_stiffness_.resize(info_.joints.size(), 30);
  hw_damping_.resize(info_.joints.size(), 0.7);
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
  if (udp_replier_->Setup() != UDPSocket::ErrorCode::kSuccess) {
    RCLCPP_ERROR(rclcpp::get_logger("KukaRoXHardwareInterface"), "Could not setup udp replier");
    return CallbackReturn::ERROR;
  }
#endif

  // if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1) {
  //   RCLCPP_ERROR(rclcpp::get_logger("KukaRoXHardwareInterface"), "mlockall error");
  //   RCLCPP_ERROR(rclcpp::get_logger("KukaRoXHardwareInterface"), strerror(errno));
  //   return CallbackReturn::ERROR;
  // }

  struct sched_param param;
  param.sched_priority = 95;
  if (sched_setscheduler(0, SCHED_FIFO, &param) == -1) {
    RCLCPP_ERROR(rclcpp::get_logger("KukaRoXHardwareInterface"), "setscheduler error");
    RCLCPP_ERROR(rclcpp::get_logger("KukaRoXHardwareInterface"), strerror(errno));
    return CallbackReturn::ERROR;
  }
  RCLCPP_INFO(rclcpp::get_logger("KukaRoXHardwareInterface"), "Init successful");

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
KukaRoXHardwareInterface::export_state_interfaces()
{
  RCLCPP_INFO(rclcpp::get_logger("KukaRoXHardwareInterface"), "Export state interfaces");
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < info_.joints.size(); i++) {
    state_interfaces.emplace_back(
      info_.joints[i].name,
      hardware_interface::HW_IF_POSITION,
      &hw_states_[i]);
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> KukaRoXHardwareInterface::
export_command_interfaces()
{
  RCLCPP_INFO(rclcpp::get_logger("KukaRoXHardwareInterface"), "Export command interfaces");

  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < info_.joints.size(); i++) {
    command_interfaces.emplace_back(
      info_.joints[i].name,
      hardware_interface::HW_IF_POSITION,
      &hw_position_commands_[i]);

    command_interfaces.emplace_back(
      info_.joints[i].name,
      hardware_interface::HW_IF_EFFORT,
      &hw_torque_commands_[i]);

    command_interfaces.emplace_back(
      info_.joints[i].name,
      HW_IF_STIFFNESS,
      &hw_stiffness_[i]);

    command_interfaces.emplace_back(
      info_.joints[i].name,
      HW_IF_DAMPING,
      &hw_damping_[i]);
  }

  return command_interfaces;
}

CallbackReturn KukaRoXHardwareInterface::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger("KukaRoXHardwareInterface"), "Connecting to robot . . .");

#ifdef NON_MOCK_SETUP
  observe_thread_ = std::thread(&KukaRoXHardwareInterface::ObserveControl, this);

  OpenControlChannelRequest request;
  OpenControlChannelResponse response;
  grpc::ClientContext context;

  request.set_ip_address(info_.hardware_parameters.at("client_ip"));
  request.set_timeout(5000);
  request.set_cycle_time(4);
  request.set_external_control_mode(
    kuka::motion::external::ExternalControlMode(
      std::stoi(info_.hardware_parameters.at("control_mode"))));

  if (stub_->OpenControlChannel(
      &context, request,
      &response)
    .error_code() != grpc::StatusCode::OK)
  {
    return CallbackReturn::ERROR;
  }
#endif

  return CallbackReturn::SUCCESS;
}

CallbackReturn KukaRoXHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger("KukaRoXHardwareInterface"), "Deactivating");

  control_signal_ext_.control_signal.stop_ipo = true;
  while (is_active_) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  terminate_ = true;
#ifdef NON_MOCK_SETUP
  if (context_ != nullptr) {
    context_->TryCancel();
  }
#endif

  if (observe_thread_.joinable()) {
    observe_thread_.join();
  }
  return CallbackReturn::SUCCESS;
}

return_type KukaRoXHardwareInterface::read(
  const rclcpp::Time &,
  const rclcpp::Duration &)
{
#ifndef NON_MOCK_SETUP
  std::this_thread::sleep_for(std::chrono::microseconds(3900));
  for (size_t i = 0; i < info_.joints.size(); i++) {
    hw_states_[i] = hw_position_commands_[i];
  }
  return return_type::OK;
#endif

  if (!is_active_) {
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
    msg_received_ = false;

    return return_type::OK;
  }

  if (udp_replier_->ReceiveRequestOrTimeout(receive_timeout_) ==
    UDPSocket::ErrorCode::kSuccess)
  {
    auto req_message = udp_replier_->GetRequestMessage();

    if (!nanopb::Decode<nanopb::kuka::ecs::v1::MotionStateExternal>(
        req_message.first, req_message.second, motion_state_external_))
    {
      RCLCPP_INFO(rclcpp::get_logger("KukaRoXHardwareInterface"), "Decoding request failed");
      throw std::runtime_error("Decoding request failed");
    }
    control_signal_ext_.header.ipoc = motion_state_external_.header.ipoc;

    for (size_t i = 0; i < info_.joints.size(); i++) {
      hw_states_[i] = motion_state_external_.motion_state.measured_positions.values[i];
      // This is necessary, as joint trajectory controller is initialized with 0 command values
      if (!msg_received_ && motion_state_external_.header.ipoc == 0) {
        hw_position_commands_[i] = hw_states_[i];
      }
    }

    if (motion_state_external_.motion_state.ipo_stopped) {
      RCLCPP_INFO(rclcpp::get_logger("KukaRoXHardwareInterface"), "Motion stopped");
    }
    msg_received_ = true;
    receive_timeout_ = std::chrono::milliseconds(6);
  } else {
    RCLCPP_WARN(rclcpp::get_logger("KukaRoXHardwareInterface"), "Request was missed");
    RCLCPP_WARN(
      rclcpp::get_logger("KukaRoXHardwareInterface"),
      "Previous ipoc: %i", motion_state_external_.header.ipoc);
    msg_received_ = false;
  }
  return return_type::OK;
}

return_type KukaRoXHardwareInterface::write(
  const rclcpp::Time &,
  const rclcpp::Duration &)
{
  size_t MTU = 1500;
  uint8_t out_buff_arr[MTU];

  // If control is not started or a request is missed, do not send back anything
  if (!msg_received_) {
    return return_type::OK;
  }
  for (size_t i = 0; i < info_.joints.size(); i++) {
    control_signal_ext_.control_signal.joint_command.values[i] = hw_position_commands_[i];
    control_signal_ext_.control_signal.joint_torque_command.values[i] = hw_torque_commands_[i];
    // TODO(Svastits): should we separate control modes somehow?
    control_signal_ext_.control_signal.joint_attributes.stiffness[i] = hw_stiffness_[i];
    control_signal_ext_.control_signal.joint_attributes.damping[i] = hw_damping_[i];
  }

  auto encoded_bytes = nanopb::Encode<nanopb::kuka::ecs::v1::ControlSignalExternal>(
    control_signal_ext_, out_buff_arr, MTU);
  if (encoded_bytes < 0) {
    RCLCPP_ERROR(
      rclcpp::get_logger(
        "KukaRoXHardwareInterface"),
      "Encoding of control signal to out_buffer failed.");
    throw std::runtime_error("Encoding of control signal to out_buffer failed.");
  }

  if (udp_replier_->SendReply(out_buff_arr, encoded_bytes) !=
    UDPSocket::ErrorCode::kSuccess)
  {
    RCLCPP_ERROR(rclcpp::get_logger("KukaRoXHardwareInterface"), "Error sending reply");
    throw std::runtime_error("Error sending reply");
  }
  return return_type::OK;
}

bool KukaRoXHardwareInterface::isActive() const
{
  return is_active_;
}

void KukaRoXHardwareInterface::ObserveControl()
{
#ifdef NON_MOCK_SETUP
  RCLCPP_INFO(
    rclcpp::get_logger(
      "KukaRoXHardwareInterface"),
    "Observe control");
  while (!terminate_) {
    context_ = std::make_unique<::grpc::ClientContext>();
    ObserveControlStateRequest obs_control;
    std::unique_ptr<grpc::ClientReader<CommandState>> reader(
      stub_->ObserveControlState(context_.get(), obs_control));

    CommandState response;

    if (reader->Read(&response)) {
      RCLCPP_INFO(
        rclcpp::get_logger(
          "KukaRoXHardwareInterface"),
        "Event streamed from external control service");
      std::unique_lock<std::mutex> lck(observe_mutex_);    // TODO(Svastits): is this necessary?
      command_state_ = response;
      RCLCPP_INFO(
        rclcpp::get_logger("KukaRoXHardwareInterface"), "New state: %i",
        static_cast<int>(response.event()));

      switch (static_cast<int>(response.event())) {
        case CommandEvent::COMMAND_READY:
          RCLCPP_INFO(rclcpp::get_logger("KukaRoXHardwareInterface"), "Command accepted");
          break;
        case CommandEvent::SAMPLING:
          RCLCPP_INFO(rclcpp::get_logger("KukaRoXHardwareInterface"), "External control is active");
          is_active_ = true;
          break;
        case CommandEvent::STOPPED:
          RCLCPP_INFO(rclcpp::get_logger("KukaRoXHardwareInterface"), "External control finished");
          is_active_ = false;
          break;
        case CommandEvent::ERROR:
          RCLCPP_ERROR(
            rclcpp::get_logger(
              "KukaRoXHardwareInterface"),
            "External control stopped by an error");
          RCLCPP_ERROR(rclcpp::get_logger("KukaRoXHardwareInterface"), response.message().c_str());
          is_active_ = false;
          break;
        default:
          break;
      }
    } else {
      // WORKAROUND: Ec is starting later so we have some errors before stable work.
      std::this_thread::sleep_for(std::chrono::milliseconds(300));
    }
  }
#endif
}

}  // namespace namespace kuka_rox

PLUGINLIB_EXPORT_CLASS(
  kuka_rox::KukaRoXHardwareInterface,
  hardware_interface::SystemInterface)
