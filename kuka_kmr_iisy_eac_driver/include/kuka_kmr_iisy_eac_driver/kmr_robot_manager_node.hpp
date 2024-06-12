// Copyright 2022 Komáromi Sándor
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

#ifndef KUKA_KMR_IISY_EAC_DRIVER___ROBOT_MANAGER_NODE_HPP_
#define KUKA_KMR_IISY_EAC_DRIVER___ROBOT_MANAGER_NODE_HPP_

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "controller_manager_msgs/srv/set_hardware_component_state.hpp"
#include "controller_manager_msgs/srv/switch_controller.hpp"
#include "rclcpp/client.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/u_int32.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "kuka_iiqka_drivers_core/robot_manager_node_base.hpp"
#include "kuka_drivers_core/controller_handler.hpp"
#include "kuka_drivers_core/ros2_base_lc_node.hpp"

namespace kuka_eac
{
class MobileRobotManagerNode : public RobotManagerNodeBase
{
public:
  MobileRobotManagerNode();

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State &) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State &) override;

private:
  bool onControlModeChangeRequest(int control_mode);
  void EventSubscriptionCallback(const std_msgs::msg::UInt8::SharedPtr msg);

};

}  // namespace kuka_eac

#endif  // KUKA_KMR_IISY_EAC_DRIVER___ROBOT_MANAGER_NODE_HPP_
