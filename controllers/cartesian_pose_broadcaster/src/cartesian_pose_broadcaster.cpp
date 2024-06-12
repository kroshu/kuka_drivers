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
#include "kuka_drivers_core/hardware_interface_types.hpp"

#include "cartesian_pose_broadcaster.hpp"
using namespace hardware_interface;
namespace kuka_controllers
{

controller_interface::CallbackReturn CartesianPoseBroadcaster::on_init()
{
  try
  {
    param_listener_ = std::make_shared<ParamListener>(get_node());
    params_ = param_listener_->get_params();
  }
  catch (const std::exception & e)
  {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  tf_broadcaster_=std::make_unique<tf2_ros::TransformBroadcaster>(get_node());
  
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration CartesianPoseBroadcaster::
command_interface_configuration()
const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::NONE;
  return config;
}

controller_interface::InterfaceConfiguration CartesianPoseBroadcaster::state_interface_configuration()
const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  config.names.emplace_back(std::string(POSE_PREFIX) + "/" + POSITION_PREFIX + "/" + HW_IF_X);
  config.names.emplace_back(std::string(POSE_PREFIX) + "/" + POSITION_PREFIX + "/" + HW_IF_Y);
  config.names.emplace_back(std::string(POSE_PREFIX) + "/" + POSITION_PREFIX + "/" + HW_IF_Z);
  config.names.emplace_back(std::string(POSE_PREFIX) + "/" + ORIENTATION_PREFIX + "/" + HW_IF_X);
  config.names.emplace_back(std::string(POSE_PREFIX) + "/" + ORIENTATION_PREFIX + "/" + HW_IF_Y);
  config.names.emplace_back(std::string(POSE_PREFIX) + "/" + ORIENTATION_PREFIX + "/" + HW_IF_Z);
  config.names.emplace_back(std::string(POSE_PREFIX) + "/" + ORIENTATION_PREFIX + "/" + HW_IF_W);
  return config;
}

controller_interface::CallbackReturn
CartesianPoseBroadcaster::on_configure(const rclcpp_lifecycle::State &)
{
  params_ = param_listener_->get_params();
  if (params_.world_frame_name.empty() && params_.robot_base_name.empty())
  {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "'world_frame_name' and 'robot_base_name' parameter has to be specified.");
    return controller_interface::CallbackReturn::ERROR;
  }
  else 
  {
    transformStamped_.header.frame_id=params_.world_frame_name;
    transformStamped_.child_frame_id=params_.robot_base_name;

  }
  
  return controller_interface::CallbackReturn::SUCCESS;

}

controller_interface::CallbackReturn
CartesianPoseBroadcaster::on_activate(const rclcpp_lifecycle::State &)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
CartesianPoseBroadcaster::on_deactivate(const rclcpp_lifecycle::State &)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type CartesianPoseBroadcaster::update(
  const rclcpp::Time &,
  const rclcpp::Duration &)
{
  transformStamped_.transform.translation.x=state_interfaces_[0].get_value();
  transformStamped_.transform.translation.y=state_interfaces_[1].get_value();
  transformStamped_.transform.translation.z=state_interfaces_[2].get_value();
 
  transformStamped_.transform.rotation.x=state_interfaces_[3].get_value();
  transformStamped_.transform.rotation.y=state_interfaces_[4].get_value();
  transformStamped_.transform.rotation.z=state_interfaces_[5].get_value();
  transformStamped_.transform.rotation.w=state_interfaces_[6].get_value(); 

  tf_broadcaster_->sendTransform(transformStamped_);
 
  return controller_interface::return_type::OK;
}

}  // namespace kuka_controllers

PLUGINLIB_EXPORT_CLASS(
  kuka_controllers::CartesianPoseBroadcaster,
  controller_interface::ControllerInterface)
