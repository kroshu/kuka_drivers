// Copyright 2022 Aron Svastits
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

#include "pluginlib/class_list_macros.hpp"

#include "kuka_drivers_core/hardware_interface_types.hpp"

#include "joint_group_impedance_controller/joint_group_impedance_controller.hpp"

namespace kuka_controllers
{

JointGroupImpedanceController::JointGroupImpedanceController() : ForwardControllersBase() {}

void JointGroupImpedanceController::declare_parameters()
{
  param_listener_ = std::make_shared<ParamListener>(get_node());
}

controller_interface::CallbackReturn JointGroupImpedanceController::read_parameters()
{
  if (!param_listener_)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Error encountered during init");
    return controller_interface::CallbackReturn::ERROR;
  }
  params_ = param_listener_->get_params();

  if (params_.joints.empty())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "'joints' parameter is empty");
    return controller_interface::CallbackReturn::ERROR;
  }

  if (params_.interface_names.empty())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "'interfaces' parameter is empty");
    return controller_interface::CallbackReturn::ERROR;
  }

  for (const auto & joint : params_.joints)
  {
    for (const auto & interface : params_.interface_names)
    {
      command_interface_types_.push_back(joint + "/" + interface);
    }
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn JointGroupImpedanceController::on_init()
{
  auto ret = ForwardControllersBase::on_init();
  if (ret != CallbackReturn::SUCCESS)
  {
    return ret;
  }

  try
  {
    // Explicitly set the interfaces parameter declared by the
    // forward_command_controller
    get_node()->set_parameter(rclcpp::Parameter(
      "interface_names",
      std::vector<std::string>{
        hardware_interface::HW_IF_STIFFNESS, hardware_interface::HW_IF_DAMPING}));
  }
  catch (const std::exception & e)
  {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}
}  // namespace kuka_controllers

PLUGINLIB_EXPORT_CLASS(
  kuka_controllers::JointGroupImpedanceController, controller_interface::ControllerInterface)
