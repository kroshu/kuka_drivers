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

using namespace joint_impedance_controller;

namespace kuka_controllers
{
controller_interface::CallbackReturn JointImpedanceController::on_init()
{
  try
  {
    // Create the parameter listener and get the parameters
    param_listener_ = std::make_shared<ParamListener>(get_node());
    params_ = param_listener_->get_params();
  }
  catch (const std::exception & e)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Exception thrown during init stage with message: %s \n", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration JointImpedanceController::
command_interface_configuration()
const
{

  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  if (dof_ == 0)
  {
    RCLCPP_ERROR(get_node()->get_logger(), 
      "During ros2_control interface configuration, degrees of freedom is not valid;"
      " it should be positive. Actual DOF is %zu\n",
      dof_);  
  }
  config.names.reserve(dof_ * command_interfaces_param.size());
  for (const auto & joint_name : command_joint_names_)
  {
    for (const auto & interface_type : command_interfaces_param)
    {
      config.names.push_back(joint_name + "/" + interface_type);
    }
  }
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
  // Check if ParaListner is working correctly
  if (!param_listener_)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Error encountered during init");
    return controller_interface::CallbackReturn::ERROR;
  }

  // update the dynamic map parameters
  param_listener_->refresh_dynamic_parameters();

  // get parameters from the listener in case they were updated
  params_ = param_listener_->get_params();

  // Check if the DoF has changed
  if ((dof_ > 0) && (params_.joints.size() != dof_))
  {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "The JointImpedanceController does not support restarting with a different number of DOF");
    return CallbackReturn::FAILURE;
  }

  if (params_.joints.empty())
  {
    RCLCPP_WARN(get_node()->get_logger(), "'joints' parameter is empty.");
  }

  // Set dof_
  dof_ = params_.joints.size();

  // Add joint_names
  command_joint_names_ = params_.joints;

  // Resize parameters
  stiffness_.resize(dof_, STIFFNESS_DEFAULT);
  damping_.resize(dof_, DAMPING_DEFAULT);

  joint_impedance_subscriber_ = get_node()->create_subscription<std_msgs::msg::Float64MultiArray>(
    "joint_impedance", rclcpp::SystemDefaultsQoS(),
    [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
      for (auto i = 0; i < (int) dof_; ++i) {
        stiffness_[i] = msg->data[i];
        damping_[i] = msg->data[i + dof_];
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

  for (auto index = 0; index < (int) dof_; ++index) {
    command_interfaces_[index].set_value(stiffness_[index]);
  }
  for (auto index = dof_; index < dof_*command_interfaces_param.size(); ++index) {
    command_interfaces_[index].set_value(damping_[index-dof_]);
  }
  return controller_interface::return_type::OK;
}

}  // namespace kuka_controllers

PLUGINLIB_EXPORT_CLASS(
  kuka_controllers::JointImpedanceController,
  controller_interface::ControllerInterface)
