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

#ifndef KUKA_IIQKA_EAC_DRIVER__HARDWARE_INTERFACE_HPP_
#define KUKA_IIQKA_EAC_DRIVER__HARDWARE_INTERFACE_HPP_

#include <vector>
#include <string>
#include <memory>
#include <chrono>
#include <cmath>
#include <mutex>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "hardware_interface/system_interface.hpp"

#include "kuka/ecs/v1/motion_services_ecs.grpc.pb.h"
#include "nanopb/kuka/core/motion/joint.pb.hh"
#include "nanopb/kuka/ecs/v1/control_signal_external.pb.hh"
#include "nanopb/kuka/ecs/v1/motion_state_external.pb.hh"
#include "os-core-udp-communication/replier.h"

#include "kuka_iiqka_eac_driver/visibility_control.h"

using hardware_interface::return_type;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace kuka_eac
{

class KukaEACHardwareInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(KukaEACHardwareInterface)

  KUKA_IIQKA_EAC_DRIVER_PUBLIC CallbackReturn on_init(const hardware_interface::HardwareInfo & info)
  override;

  KUKA_IIQKA_EAC_DRIVER_PUBLIC std::vector<hardware_interface::StateInterface>
  export_state_interfaces() override;

  KUKA_IIQKA_EAC_DRIVER_PUBLIC std::vector<hardware_interface::CommandInterface>
  export_command_interfaces() override;

  KUKA_IIQKA_EAC_DRIVER_PUBLIC CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  KUKA_IIQKA_EAC_DRIVER_PUBLIC CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  KUKA_IIQKA_EAC_DRIVER_PUBLIC CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  KUKA_IIQKA_EAC_DRIVER_PUBLIC return_type read(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

  KUKA_IIQKA_EAC_DRIVER_PUBLIC return_type write(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

private:
  KUKA_IIQKA_EAC_DRIVER_LOCAL void ObserveControl();

  bool is_active_ = false;
  bool msg_received_ = false;

  std::vector<double> hw_position_commands_;
  std::vector<double> hw_torque_commands_;
  std::vector<double> hw_stiffness_commands_;
  std::vector<double> hw_damping_commands_;

  std::vector<double> hw_position_states_;
  std::vector<double> hw_torque_states_;

  double hw_control_mode_command_;

#ifdef NON_MOCK_SETUP
  kuka::ecs::v1::CommandState command_state_;
  std::unique_ptr<kuka::ecs::v1::ExternalControlService::Stub> stub_;
  std::unique_ptr<grpc::ClientContext> context_;
#endif

  std::thread observe_thread_;

  std::unique_ptr<os::core::udp::communication::Replier> udp_replier_;
  std::chrono::milliseconds receive_timeout_ {100};

  uint8_t out_buff_arr_[1500];

  nanopb::kuka::ecs::v1::ControlSignalExternal control_signal_ext_{
    nanopb::kuka::ecs::v1::ControlSignalExternal_init_default};
  nanopb::kuka::ecs::v1::MotionStateExternal motion_state_external_{
    nanopb::kuka::ecs::v1::MotionStateExternal_init_default};

};
}  // namespace kuka_eac

#endif  // KUKA_IIQKA_EAC_DRIVER__HARDWARE_INTERFACE_HPP_
