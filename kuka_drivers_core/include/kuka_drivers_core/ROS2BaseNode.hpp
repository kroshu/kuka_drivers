// Copyright 2020 Gergely Kovács
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

#ifndef KUKA_DRIVERS_CORE__ROS2BASENODE_HPP_
#define KUKA_DRIVERS_CORE__ROS2BASENODE_HPP_

#include <string>
#include <map>
#include <vector>
#include <memory>
#include <functional>

#include "rclcpp/node.hpp"
#include "lifecycle_msgs/msg/state.hpp"

#include "kuka_drivers_core/ParameterHandler.hpp"

namespace kuka_drivers_core
{
class ROS2BaseNode : public rclcpp::Node
{
public:
  explicit ROS2BaseNode(
    const std::string & node_name,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  const ParameterHandler & getParameterHandler() const;
  template<typename T>
  void registerParameter(
    const std::string & name, const T & value,
    std::function<bool(const T &)> on_change_callback)
  {
    param_handler_.registerParameter<T>(
      name, value,
      on_change_callback, this->get_node_parameters_interface());
  }

  template<typename T>
  void registerStaticParameter(
    const std::string & name, const T & value,
    std::function<bool(const T &)> on_change_callback)
  {
    param_handler_.registerParameter<T>(
      name, value,
      on_change_callback, this->get_node_parameters_interface(), true);
  }

protected:
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr ParamCallback() const;

private:
  ParameterHandler param_handler_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_;
};
}  // namespace kuka_drivers_core

#endif  // KUKA_DRIVERS_CORE__ROS2BASENODE_HPP_
