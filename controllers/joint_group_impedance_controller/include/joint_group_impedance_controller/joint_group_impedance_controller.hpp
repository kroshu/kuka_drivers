// Copyright 2022 KUKA Hungaria Kft.
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

#ifndef JOINT_GROUP_IMPEDANCE_CONTROLLER__JOINT_GROUP_IMPEDANCE_CONTROLLER_HPP_
#define JOINT_GROUP_IMPEDANCE_CONTROLLER__JOINT_GROUP_IMPEDANCE_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "forward_command_controller/multi_interface_forward_command_controller.hpp"

#include "joint_group_impedance_controller/joint_group_impedance_controller_parameters.hpp"
#include "joint_group_impedance_controller/visibility_control.h"

namespace kuka_controllers
{
class JointGroupImpedanceController : public forward_command_controller::ForwardControllersBase
{
public:
  JOINT_GROUP_IMPEDANCE_CONTROLLER_PUBLIC JointGroupImpedanceController();
  JOINT_GROUP_IMPEDANCE_CONTROLLER_PUBLIC controller_interface::CallbackReturn on_init() override;
  JOINT_GROUP_IMPEDANCE_CONTROLLER_PUBLIC controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  JOINT_GROUP_IMPEDANCE_CONTROLLER_PUBLIC controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  JOINT_GROUP_IMPEDANCE_CONTROLLER_LOCAL void declare_parameters() override;
  JOINT_GROUP_IMPEDANCE_CONTROLLER_LOCAL controller_interface::CallbackReturn read_parameters()
    override;
  JOINT_GROUP_IMPEDANCE_CONTROLLER_LOCAL controller_interface::InterfaceConfiguration state_interface_configuration() const
    override;

  using Params = joint_group_impedance_controller::Params;
  using ParamListener = joint_group_impedance_controller::ParamListener;

  std::shared_ptr<ParamListener> param_listener_;
  Params params_;

  std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float64MultiArray>> commanded_joint_pos_publisher_;
  std_msgs::msg::Float64MultiArray commanded_joint_pos_;
 
};
}  // namespace kuka_controllers
#endif  // JOINT_GROUP_IMPEDANCE_CONTROLLER__JOINT_GROUP_IMPEDANCE_CONTROLLER_HPP_
