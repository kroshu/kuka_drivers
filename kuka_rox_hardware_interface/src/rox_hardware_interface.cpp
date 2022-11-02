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

#include "kuka_rox_hw_interface/rox_hardware_interface.hpp"
#include "kuka/nanopb-helpers-0.0/nanopb-helpers/nanopb_serialization_helper.h"

using namespace kuka::ecs::v1;

namespace kuka_rox
{
CallbackReturn KukaRoXHardwareInterface::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

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

  observe_thread_ = std::thread(&KukaRoXHardwareInterface::ObserveControl, this);

  StartControlRequest request;
  StartControlResponse response;
  grpc::ClientContext context;

  request.set_timeout(5000);
  request.set_cycle_time(4);

  // TODO: how to initialize this better?
  ExternalControlMode * mode_tmp = new ExternalControlMode();
  mode_tmp->set_control_type(ExternalControlType::POSITION_CONTROL);
  request.set_allocated_external_control_mode(mode_tmp);

  grpc::Status status = stub_->StartControl(&context, request, &response);
  return CallbackReturn::SUCCESS;
}

CallbackReturn KukaRoXHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State &)
{
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
  if (udp_replier_.WaitForAndReceiveRequest(std::chrono::milliseconds(4000)) ==
    os::core::udp::communication::State::kSuccess)
  {
    auto req_message = udp_replier_.GetRequestMessage();

    if (!nanopb::Decode<nanopb::kuka::ecs::v1::MotionStateExternal>(
        req_message.data(), req_message.size(), motion_state_external_))
    {
      RCLCPP_INFO(rclcpp::get_logger("KukaRoXHardwareInterface"), "Decoding request failed");
      return return_type::ERROR;
    }
    RCLCPP_INFO(
      rclcpp::get_logger(
        "KukaRoXHardwareInterface"), "Got motion state, decoded, ipoc: %i",
      motion_state_external_.header.ipoc);
    control_signal_ext_.header.ipoc = motion_state_external_.header.ipoc;

    for (size_t i = 0; i < info_.joints.size(); i++) {
      hw_states_[i] = motion_state_external_.motion_state.measured_positions.values[i];
    }

    if (motion_state_external_.motion_state.ipo_stopped) {
      terminate_replier_ = true;
      RCLCPP_INFO(rclcpp::get_logger("KukaRoXHardwareInterface"), "Motion stopped");
    }
  }
  return return_type::OK;
}

return_type KukaRoXHardwareInterface::write(
  const rclcpp::Time &,
  const rclcpp::Duration &)
{
  size_t MTU = 1500;
  uint8_t out_buff_arr[MTU];

  for (size_t i = 0; i < info_.joints.size(); i++) {
    control_signal_ext_.control_signal.joint_command.values[i] = hw_commands_[i];
  }

  auto encoded_bytes = nanopb::Encode<nanopb::kuka::ecs::v1::ControlSignalExternal>(
    control_signal_ext_, out_buff_arr, MTU);
  if (encoded_bytes < 0) {
    std::cout << "Encoding of control signal to out_buffer failed." << std::endl;
    return return_type::ERROR;
  }

  if (udp_replier_.SendReply(out_buff_arr, encoded_bytes) !=
    os::core::udp::communication::ReplyState::kSuccess)
  {
    std::cout << "Error sending reply" << std::endl;
  } else {
    std::cout << "Sent reply encoded, ipoc: " << control_signal_ext_.header.ipoc << std::endl;
  }

  return return_type::OK;
}

bool KukaRoXHardwareInterface::isActive() const
{
  return is_active_;
}

void KukaRoXHardwareInterface::ObserveControl()
{
  while (!terminate_) {
    RCLCPP_INFO(
      rclcpp::get_logger(
        "KukaRoXHardwareInterface"), "Observe control");
    context_ = std::make_unique<::grpc::ClientContext>();
    ObserveControlStateRequest obs_control;
    std::unique_ptr<grpc::ClientReader<CommandState>> reader(
      stub_->ObserveControlState(context_.get(), obs_control));

    CommandState response;

    if (reader->Read(&response)) {
      RCLCPP_INFO(
        rclcpp::get_logger(
          "KukaRoXHardwareInterface"), "Event streamed from external control service");
      std::unique_lock<std::mutex> lck(observe_mutex_);
      command_state_ = response;
      RCLCPP_INFO(
        rclcpp::get_logger("KukaRoXHardwareInterface"), "New state: %i",
        static_cast<int>(response.event()));

      switch (static_cast<int>(response.event())) {
        case 2:
          is_active_ = true;
          break;
        case 6:
          std::cout << response.message() << std::endl;
          is_active_ = true;
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
