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

#ifndef KUKA_ROX_HW_INTERFACE__ROX_HARDWARE_INTERFACE_HPP_
#define KUKA_ROX_HW_INTERFACE__ROX_HARDWARE_INTERFACE_HPP_

#include <vector>
#include <string>
#include <memory>
#include <chrono>
#include <cmath>
#include <mutex>
#include <thread>

#include "rclcpp/macros.hpp"

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "pluginlib/class_list_macros.hpp"

#include "kuka/ecs/v1/motion_services_ecs.grpc.pb.h"
#include "nanopb/kuka/core/motion/joint.pb.hh"
#include "nanopb/kuka/ecs/v1/control_signal_external.pb.hh"
#include "nanopb/kuka/ecs/v1/motion_state_external.pb.hh"
#include "os-core-udp-communication/udp_replier.h"

using hardware_interface::return_type;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace kuka_rox
{

class KukaRoXHardwareInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(KukaRoXHardwareInterface)

  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  bool isActive() const;

  void ObserveControl();

private:
  bool is_active_ = false;
  bool msg_received_ = false;
  std::string rsi_ip_address_ = "";
  int rsi_port_ = 0;

  std::vector<double> hw_commands_;
  std::vector<double> hw_states_;

  uint64_t ipoc_ = 0;
  std::unique_ptr<kuka::ecs::v1::ExternalControlService::Stub> stub_;
  unsigned char token_[16];
  int32_t timeout_;
  bool stopped_ = true;

  std::unique_ptr<grpc::ClientContext> context_;

  std::thread observe_thread_;
  std::atomic<bool> terminate_{false};
  kuka::ecs::v1::CommandState command_state_;
  std::mutex observe_mutex_;

  // insert port of your client instead of -1
  os::core::udp::communication::UDPReplier udp_replier_ = os::core::udp::communication::UDPReplier(
    os::core::udp::communication::SocketAddress("<insert ip of your client here>", -1));
  std::thread start_control_thread_;

  nanopb::kuka::ecs::v1::ControlSignalExternal control_signal_ext_{
    nanopb::kuka::ecs::v1::ControlSignalExternal_init_default};
  nanopb::kuka::ecs::v1::MotionStateExternal motion_state_external_{
    nanopb::kuka::ecs::v1::MotionStateExternal_init_default};

  nanopb::kuka::core::motion::JointPositions start_pos_{
    nanopb::kuka::core::motion::JointPositions_init_default};
};
}  // namespace kuka_rox

#endif  // KUKA_ROX_HW_INTERFACE__ROX_HARDWARE_INTERFACE_HPP_
