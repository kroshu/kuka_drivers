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

#ifndef KUKA_IIQKA_EAC_DRIVER__ROBOT_MANAGER_NODE_HPP_
#define KUKA_IIQKA_EAC_DRIVER__ROBOT_MANAGER_NODE_HPP_

#include "kuka_iiqka_drivers_core/robot_manager_node_base.hpp"

namespace kuka_eac
{
class RobotManagerNode : public RobotManagerNodeBase
{
public:
  RobotManagerNode();



  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State &) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State &) override;

private:
  void EventSubscriptionCallback(const std_msgs::msg::UInt8::SharedPtr msg);
  bool onControlModeChangeRequest(int control_mode);

};

}  // namespace kuka_eac

#endif  // KUKA_IIQKA_EAC_DRIVER__ROBOT_MANAGER_NODE_HPP_
