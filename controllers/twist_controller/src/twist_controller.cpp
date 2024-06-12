// Copyright 2023 Kristófi Mihály
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

#include "twist_controller.hpp"
using namespace hardware_interface;
namespace kuka_controllers
{
TwistController::TwistController() : ForwardControllersBase(),  twist_command_subscriber_(nullptr)
 {}
void TwistController::declare_parameters()
{
  param_listener_ = std::make_shared<ParamListener>(get_node());
}

controller_interface::CallbackReturn TwistController::read_parameters()
{
  if (!param_listener_)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Error encountered during init");
    return controller_interface::CallbackReturn::ERROR;
  }
  params_ = param_listener_->get_params();

  if (params_.interface_names.empty())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "'interfaces' parameter is empty");
    return controller_interface::CallbackReturn::ERROR;
  }


  for (const auto & interface : params_.interface_names)
    {
      command_interface_types_.push_back(interface);
    }
  
  return controller_interface::CallbackReturn::SUCCESS;
}
controller_interface::CallbackReturn TwistController::on_init()
{auto ret = ForwardControllersBase::on_init();
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
        std::string(TWIST_PREFIX) + "/" + std::string(LINEAR_PREFIX)+ "/" + std::string(HW_IF_X),
        std::string(TWIST_PREFIX) + "/" + std::string(LINEAR_PREFIX)+ "/" + std::string(HW_IF_Y),
        std::string(TWIST_PREFIX) + "/" + std::string(LINEAR_PREFIX)+ "/" + std::string(HW_IF_Z),
        std::string(TWIST_PREFIX) + "/" + std::string(ANGULAR_PREFIX)+ "/" + std::string(HW_IF_X),
        std::string(TWIST_PREFIX) + "/" + std::string(ANGULAR_PREFIX)+ "/" + std::string(HW_IF_Y),
        std::string(TWIST_PREFIX) + "/" + std::string(ANGULAR_PREFIX)+ "/" + std::string(HW_IF_Z)
      }));
  }
  catch (const std::exception & e)
  {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }
  command_ptr_ = std::make_shared<forward_command_controller::CmdType>();
  twist_command_subscriber_ = get_node()->create_subscription<geometry_msgs::msg::Twist>(
    "~/twist_commands", rclcpp::SystemDefaultsQoS(),
    [this](const geometry_msgs::msg::Twist msg) { 
      command_ptr_->data={msg.linear.x,msg.linear.y,msg.linear.z,msg.angular.x,msg.angular.y,msg.angular.z};
      rt_command_ptr_.writeFromNonRT(command_ptr_);
      });
  return controller_interface::CallbackReturn::SUCCESS;
}

}  // namespace kuka_controllers

PLUGINLIB_EXPORT_CLASS(
  kuka_controllers::TwistController,
  controller_interface::ControllerInterface)
