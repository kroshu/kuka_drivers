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

#include "kuka_controllers/joint_impedance_controller.hpp"

namespace kuka_controllers
{
controller_interface::CallbackReturn JointImpedanceController::on_init()
{
  stiffness_.resize(6);
  damping_.resize(6);
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration JointImpedanceController::
command_interface_configuration()
const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  // config.names.emplace_back("joint_impedance");
  config.names.emplace_back("joint_a1/stiffness");
  config.names.emplace_back("joint_a2/stiffness");
  config.names.emplace_back("joint_a3/stiffness");
  config.names.emplace_back("joint_a4/stiffness");
  config.names.emplace_back("joint_a5/stiffness");
  config.names.emplace_back("joint_a6/stiffness");
  config.names.emplace_back("joint_a1/damping");
  config.names.emplace_back("joint_a2/damping");
  config.names.emplace_back("joint_a3/damping");
  config.names.emplace_back("joint_a4/damping");
  config.names.emplace_back("joint_a5/damping");
  config.names.emplace_back("joint_a6/damping");
  return config;
}

controller_interface::InterfaceConfiguration JointImpedanceController::state_interface_configuration()
const
{
  return controller_interface::InterfaceConfiguration{controller_interface::
    interface_configuration_type::NONE};
}

controller_interface::CallbackReturn
JointImpedanceController::on_configure(const rclcpp_lifecycle::State &)
{
  joint_impedance_subscriber_ = get_node()->create_subscription<std_msgs::msg::Float64MultiArray>(
    "joint_impedance", rclcpp::SystemDefaultsQoS(),
    [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
      for (auto i = 0; i < 6; ++i) {
        stiffness_[i] = msg->data[i];
        damping_[i] = msg->data[i + 6];
      }
    });
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
JointImpedanceController::on_activate(const rclcpp_lifecycle::State &)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
JointImpedanceController::on_deactivate(const rclcpp_lifecycle::State &)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type JointImpedanceController::update(
  const rclcpp::Time &,
  const rclcpp::Duration &)
{

  for (auto index = 0; index < 6; ++index) {
    command_interfaces_[index].set_value(stiffness_[index]);
  }
  for (auto index = 6; index < 12; ++index) {
    command_interfaces_[index].set_value(damping_[index-6]);
  }
  return controller_interface::return_type::OK;
}

}  // namespace kuka_controllers

PLUGINLIB_EXPORT_CLASS(
  kuka_controllers::JointImpedanceController,
  controller_interface::ControllerInterface)
