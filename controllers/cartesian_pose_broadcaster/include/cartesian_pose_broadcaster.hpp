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


#ifndef KUKA_CONTROLLERS__POSE_BROADCASTER_HPP_
#define KUKA_CONTROLLERS__POSE_BROADCASTER_HPP_

#include <memory>
#include <string>
#include <vector>
#include "controller_interface/controller_interface.hpp"

#include "visibility_control.h"

#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp/duration.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.h>
#include "cartesian_pose_broadcaster_parameters.hpp"

#include "pluginlib/class_list_macros.hpp"

namespace kuka_controllers
{
class CartesianPoseBroadcaster : public controller_interface::ControllerInterface
{
public:

  POSE_BROADCASTER_PUBLIC
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  POSE_BROADCASTER_PUBLIC
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  POSE_BROADCASTER_PUBLIC
  controller_interface::return_type update(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

  POSE_BROADCASTER_PUBLIC
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state)
  override;

  POSE_BROADCASTER_PUBLIC
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state)
  override;

  POSE_BROADCASTER_PUBLIC
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state)
  override;

  POSE_BROADCASTER_PUBLIC
  controller_interface::CallbackReturn on_init() override;

private:
  using Params = cartesian_pose_broadcaster::Params;
  using ParamListener = cartesian_pose_broadcaster::ParamListener;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_{nullptr};
  geometry_msgs::msg::TransformStamped transformStamped_;
  
  std::shared_ptr<ParamListener> param_listener_;
  Params params_;
};
}  // namespace kuka_controllers
#endif  // KUKA_CONTROLLERS__POSE_BROADCASTER_HPP_
