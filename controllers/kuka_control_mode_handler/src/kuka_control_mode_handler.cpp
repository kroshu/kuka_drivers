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

#include "kuka_drivers_core/hardware_interface_types.hpp"

#include "kuka_control_mode_handler/kuka_control_mode_handler.hpp"

namespace kuka_controllers
{
controller_interface::CallbackReturn ControlModeHandler::on_init()
{
  param_listener_ = std::make_shared<ParamListener>(get_node());
  params_ = param_listener_->get_params();
  return controller_interface::CallbackReturn::SUCCESS;
}

std::string ControlModeHandler::ComposeInterfaceName(
  const std::string & robot_prefix, const std::string & interface_group,
  const std::string & interface_name)
{
  if (robot_prefix.empty())
  {
    return interface_group + "/" + interface_name;
  }
  return robot_prefix + "_" + interface_group + "/" + interface_name;
}

controller_interface::InterfaceConfiguration ControlModeHandler::command_interface_configuration()
  const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (const auto & robot_prefix : params_.robot_prefixes)
  {
    config.names.emplace_back(
      ComposeInterfaceName(
        robot_prefix, hardware_interface::CONFIG_PREFIX, hardware_interface::CONTROL_MODE));
  }

  return config;
}

controller_interface::InterfaceConfiguration ControlModeHandler::state_interface_configuration()
  const
{
  return controller_interface::InterfaceConfiguration{
    controller_interface::interface_configuration_type::NONE};
}

controller_interface::CallbackReturn ControlModeHandler::on_configure(
  const rclcpp_lifecycle::State &)
{
  // TODO(Svastits): consider server instead of simple subscription
  control_mode_subscriber_ = get_node()->create_subscription<std_msgs::msg::UInt32>(
    "~/control_mode", rclcpp::SystemDefaultsQoS(),
    [this](const std_msgs::msg::UInt32::SharedPtr msg)
    {
      control_mode_.store(kuka_drivers_core::ControlMode(msg->data));
      RCLCPP_INFO(get_node()->get_logger(), "Control mode changed to %u", msg->data);
    });
  RCLCPP_INFO(get_node()->get_logger(), "Control mode handler configured");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ControlModeHandler::on_activate(
  const rclcpp_lifecycle::State &)
{
  if (control_mode_.load() <= kuka_drivers_core::ControlMode::CONTROL_MODE_UNSPECIFIED)
  {
    throw std::runtime_error("Control mode unspecified");
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ControlModeHandler::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type ControlModeHandler::update(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  const auto control_mode = static_cast<double>(control_mode_.load());
  bool all_control_modes_set = true;

  for (auto & command_interface : command_interfaces_)
  {
    all_control_modes_set = command_interface.set_value(control_mode) && all_control_modes_set;
  }

  return all_control_modes_set ? controller_interface::return_type::OK
                               : controller_interface::return_type::ERROR;
}

}  // namespace kuka_controllers

PLUGINLIB_EXPORT_CLASS(
  kuka_controllers::ControlModeHandler, controller_interface::ControllerInterface)
