// Copyright 2024 Mihaly Kristofi
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

#include "cart_pose_controller/cart_pose_controller.hpp"

namespace kuka_controllers
{

CartPoseController::CartPoseController() : ForwardControllersBase() {}

void CartPoseController::declare_parameters()
{
  param_listener_ = std::make_shared<ParamListener>(get_node());
}

controller_interface::CallbackReturn CartPoseController::read_parameters()
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
      command_interface_types_.emplace_back(joint + "/" + interface);
    }
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration CartPoseController::command_interface_configuration()
  const
{
  controller_interface::InterfaceConfiguration command_interfaces_config;
  command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  command_interfaces_config.names = command_interface_types_;
  return command_interfaces_config;
}

controller_interface::return_type CartPoseController::update(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  auto joint_commands = rt_command_ptr_.readFromRT();

  // no command received yet
  if (!joint_commands || !(*joint_commands))
  {
    return controller_interface::return_type::OK;
  }

  if ((*joint_commands)->data.size() != command_interfaces_.size())
  {
    RCLCPP_ERROR_THROTTLE(
      get_node()->get_logger(), *(get_node()->get_clock()), 1000,
      "command size (%zu) does not match number of interfaces (%zu)",
      (*joint_commands)->data.size(), command_interfaces_.size());
    return controller_interface::return_type::ERROR;
  }

  for (auto index = 0ul; index < command_interfaces_.size(); index++)
  {
    // RCLCPP_ERROR(get_node()->get_logger(), "cart joints %i: %f ",index,
    // (*joint_commands)->data[index]);
    command_interfaces_.at(index).set_value((*joint_commands)->data[index]);
  }

  return controller_interface::return_type::OK;
}

}  // namespace kuka_controllers

PLUGINLIB_EXPORT_CLASS(
  kuka_controllers::CartPoseController, controller_interface::ControllerInterface)
