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

#ifndef KUKA_SUNRISE__ROBOT_CONTROL_CLIENT_HPP_
#define KUKA_SUNRISE__ROBOT_CONTROL_CLIENT_HPP_

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


using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace kuka_sunrise
{

class RobotControlClient : public hardware_interface::SystemInterface, public KUKA::FRI::LBRClient,
  public ActivatableInterface
{
public:
  RobotControlClient()
  : client_application_(udp_connection_, *this) {}
  ~RobotControlClient();
  bool activate();
  bool deactivate();
  bool setReceiveMultiplier(int receive_multiplier);

  virtual void waitForCommand();
  virtual void command();

  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;
  hardware_interface::return_type write(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

  void updateCommand(const rclcpp::Time & stamp);

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

private:
  // rclcpp_lifecycle::LifecycleNode::SharedPtr robot_control_node_;
  rclcpp::Service<kuka_sunrise_interfaces::srv::SetInt>::SharedPtr set_receive_multiplier_service_;
  rclcpp::Clock ros_clock_;
  int receive_multiplier_;
  int receive_counter_;

  // Store the command for the simulated robot
  std::vector<double> hw_commands_, hw_states_;
  std::vector<double> hw_torques_, hw_effort_command_;

  KUKA::FRI::HWIFClientApplication client_application_;
  KUKA::FRI::UdpConnection udp_connection_;

  bool torque_command_mode_ = false;
  double tracking_performance_;
  int fri_state_;
};

}  // namespace kuka_sunrise

#endif  // KUKA_SUNRISE__ROBOT_CONTROL_CLIENT_HPP_
