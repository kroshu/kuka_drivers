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

#ifndef KUKA_ROX_HW_INTERFACE__ROBOT_MANAGER_NODE_HPP_
#define KUKA_ROX_HW_INTERFACE__ROBOT_MANAGER_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/client.hpp"

#include "kuka_rox_hardware_interface/internal/activatable_interface.hpp"

#include "kroshu_ros2_core/ROS2BaseLCNode.hpp"

namespace kuka_rox
{

class RobotManagerNode : public kroshu_ros2_core::ROS2BaseLCNode, public Activatable
{

}

} // namespace kuka_rox


#endif KUKA_ROX_HW_INTERFACE__ROBOT_MANAGER_NODE_HPP_