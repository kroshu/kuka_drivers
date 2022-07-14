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

#ifndef KUKA_SUNRISE__KUKA_FRI_HARDWARE_INTERFACE_HPP_
#define KUKA_SUNRISE__KUKA_FRI_HARDWARE_INTERFACE_HPP_

#include <condition_variable>
#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>

#include "kuka_sunrise_interfaces/srv/set_int.hpp"
#include "kuka_sunrise/internal/activatable_interface.hpp"
#include "fri/friLBRClient.h"
#include "fri/HWIFClientApplication.hpp"
#include "fri/friUdpConnection.h"

#include "pluginlib/class_list_macros.hpp"


using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace kuka_sunrise
{

class KUKAFRIHardwareInterface : public hardware_interface::SystemInterface,
  public KUKA::FRI::LBRClient,
  public ActivatableInterface  // TODO(Svastits): is this necessary in current state?
{
public:
  KUKAFRIHardwareInterface()
  : client_application_(udp_connection_, *this) {}
  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;
  hardware_interface::return_type write(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

  void updateCommand(const rclcpp::Time & stamp);
  bool setReceiveMultiplier(int receive_multiplier);

  void waitForCommand() final;
  void command() final;



private:
  KUKA::FRI::HWIFClientApplication client_application_;
  KUKA::FRI::UdpConnection udp_connection_;

  rclcpp::Service<kuka_sunrise_interfaces::srv::SetInt>::SharedPtr set_receive_multiplier_service_;
  rclcpp::Clock ros_clock_;

  // Command interface must be of type double, but controller can set only integers
  // this is a temporary solution, until runtime parameters are supported for hardware interfaces
  double receive_multiplier_ = 1;
  int receive_counter_ = 0;
  bool torque_command_mode_ = false;

  // State and command interfaces
  std::vector<double> hw_commands_;
  std::vector<double> hw_states_;
  std::vector<double> hw_torques_ext_;
  std::vector<double> hw_torques_;
  std::vector<double> hw_effort_command_;
  double tracking_performance_ = 1;
  double fri_state_ = 0;
  double connection_quality_ = 0;
};
}  // namespace kuka_sunrise

#endif  // KUKA_SUNRISE__KUKA_FRI_HARDWARE_INTERFACE_HPP_
