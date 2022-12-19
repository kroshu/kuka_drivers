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

#include <stdexcept>
#include <string>
#include <memory>
#include <vector>
#include <memory>
#include <sched.h>
#include <sys/mman.h>

#include <grpcpp/create_channel.h>

#include "kuka_rox_hw_interface/rox_hardware_interface.hpp"
#include "kuka/nanopb-helpers-0.0/nanopb-helpers/nanopb_serialization_helper.h"

using namespace kuka::ecs::v1;
using namespace kuka::motion::external;
using namespace os::core::udp::communication;

namespace kuka_rox
{
CallbackReturn KukaRoXHardwareInterface::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }
#if !MOCK_HW_ONLY
  stub_ =
    ExternalControlService::NewStub(
    grpc::CreateChannel(
      "10.36.60.227:49335",
      grpc::InsecureChannelCredentials()));
#endif
  hw_states_.resize(info_.joints.size(), 0);
  hw_commands_.resize(info_.joints.size(), 0.0);
  control_signal_ext_.has_header = true;
  control_signal_ext_.has_control_signal = true;
  control_signal_ext_.control_signal.has_joint_command = true;
  control_signal_ext_.control_signal.joint_command.values_count = 6;
#if !MOCK_HW_ONLY
  if (udp_replier_.Setup() != UDPSocket::ErrorCode::kSuccess) {
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

std::vector<hardware_interface::StateInterface> KukaRoXHardwareInterface::export_state_interfaces()
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
      &hw_commands_[i]);
  }
  return command_interfaces;
}

CallbackReturn KukaRoXHardwareInterface::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger("KukaRoXHardwareInterface"), "Connecting to robot . . .");
  

#if !MOCK_HW_ONLY
  observe_thread_ = std::thread(&KukaRoXHardwareInterface::ObserveControl, this);

  start_control_thread_ = std::thread(
      [this]()
      {
        OpenControlChannelRequest request;
        OpenControlChannelResponse response;
        grpc::ClientContext context;

        request.set_timeout(5000);
        request.set_cycle_time(4);
        request.set_external_control_mode(ExternalControlMode::POSITION_CONTROL);
        stub_->OpenControlChannel(&context, request, &response);
  });
#endif

  return CallbackReturn::SUCCESS;
}

CallbackReturn KukaRoXHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger("KukaRoXHardwareInterface"), "Deactivating");

  control_signal_ext_.control_signal.stop_ipo = true;
  while (is_active_) std::this_thread::sleep_for(std::chrono::milliseconds(10));

  terminate_ = true;
  if (context_ != nullptr) {context_->TryCancel();}

  if (observe_thread_.joinable()) {
    observe_thread_.join();
  }
  return CallbackReturn::SUCCESS;
}

return_type KukaRoXHardwareInterface::read(
  const rclcpp::Time &,
  const rclcpp::Duration &)
{
  if(!is_active_){
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
    msg_received_ = false;

    return return_type::OK;
  }

#if MOCK_HW_ONLY
  std::this_thread::sleep_for(std::chrono::microseconds(3900));
  for (size_t i = 0; i < info_.joints.size(); i++) {
    hw_states_[i] = hw_commands_[i];
  }
  return return_type::OK;
#endif


  if (udp_replier_.ReceiveRequestOrTimeout(std::chrono::milliseconds(4000)) ==
    UDPSocket::ErrorCode::kSuccess)
  {
    auto req_message = udp_replier_.GetRequestMessage();

    if (!nanopb::Decode<nanopb::kuka::ecs::v1::MotionStateExternal>(
        req_message.first, req_message.second, motion_state_external_))
    {
      RCLCPP_INFO(rclcpp::get_logger("KukaRoXHardwareInterface"), "Decoding request failed");
      return return_type::ERROR;
    }
    control_signal_ext_.header.ipoc = motion_state_external_.header.ipoc;

    for (size_t i = 0; i < info_.joints.size(); i++) {
      hw_states_[i] = motion_state_external_.motion_state.measured_positions.values[i];
       // This is necessary, as joint trajectory controller is initialized with 0 command values
      if (!msg_received_) hw_commands_[i] = hw_states_[i];
    }

    if (motion_state_external_.motion_state.ipo_stopped) {
      RCLCPP_INFO(rclcpp::get_logger("KukaRoXHardwareInterface"), "Motion stopped");
    }
  }
  msg_received_ = true;
  return return_type::OK;
}

return_type KukaRoXHardwareInterface::write(
  const rclcpp::Time &,
  const rclcpp::Duration &)
{
  size_t MTU = 1500;
  uint8_t out_buff_arr[MTU];

  if (!msg_received_) {
    return return_type::OK;
  }
  for (size_t i = 0; i < info_.joints.size(); i++) {
    control_signal_ext_.control_signal.joint_command.values[i] = hw_commands_[i];
  }  

  auto encoded_bytes = nanopb::Encode<nanopb::kuka::ecs::v1::ControlSignalExternal>(
    control_signal_ext_, out_buff_arr, MTU);
  if (encoded_bytes < 0) {
    RCLCPP_ERROR(
      rclcpp::get_logger(
        "KukaRoXHardwareInterface"), "Encoding of control signal to out_buffer failed.");
    return return_type::ERROR;
  }

  if (udp_replier_.SendReply(out_buff_arr, encoded_bytes) !=
    UDPSocket::ErrorCode::kSuccess)
  {
    RCLCPP_ERROR(rclcpp::get_logger("KukaRoXHardwareInterface"), "Error sending reply");
    return return_type::ERROR;
  } 
  return return_type::OK;
}

bool KukaRoXHardwareInterface::isActive() const
{
  return is_active_;
}

void KukaRoXHardwareInterface::ObserveControl()
{
  RCLCPP_INFO(
    rclcpp::get_logger(
      "KukaRoXHardwareInterface"), "Observe control");
  while (!terminate_) {
    context_ = std::make_unique<::grpc::ClientContext>();
    ObserveControlStateRequest obs_control;
    std::unique_ptr<grpc::ClientReader<CommandState>> reader(
      stub_->ObserveControlState(context_.get(), obs_control));

    CommandState response;

    if (reader->Read(&response)) {
      RCLCPP_INFO(
        rclcpp::get_logger(
          "KukaRoXHardwareInterface"), "Event streamed from external control service");
      std::unique_lock<std::mutex> lck(observe_mutex_);  // TODO: is this necessary?
      command_state_ = response;
      RCLCPP_INFO(
        rclcpp::get_logger("KukaRoXHardwareInterface"), "New state: %i",
        static_cast<int>(response.event()));

      switch (static_cast<int>(response.event())) {
        case 2:
          RCLCPP_INFO(rclcpp::get_logger("KukaRoXHardwareInterface"), "Command accepted");
          break;
        case 3:
          RCLCPP_INFO(rclcpp::get_logger("KukaRoXHardwareInterface"), "External control is active");
          is_active_ = true;
          break;
        case 4:
          RCLCPP_INFO(rclcpp::get_logger("KukaRoXHardwareInterface"), "External control finished");
          is_active_ = false;
          break;
        case 6:
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
}

}  // namespace namespace kuka_rox

PLUGINLIB_EXPORT_CLASS(
  kuka_rox::KukaRoXHardwareInterface,
  hardware_interface::SystemInterface
)
