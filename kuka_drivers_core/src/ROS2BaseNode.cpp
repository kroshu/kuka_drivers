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

#include <string>
#include <vector>
#include <memory>

#include "kuka_drivers_core/ROS2BaseNode.hpp"


namespace kuka_drivers_core
{

ROS2BaseNode::ROS2BaseNode(const std::string & node_name, const rclcpp::NodeOptions & options)
: rclcpp::Node(node_name, options)
{
  param_callback_ = this->add_on_set_parameters_callback(
    [this](const std::vector<rclcpp::Parameter> & parameters) {
      return param_handler_.onParamChange(parameters);
    });
}

const ParameterHandler & ROS2BaseNode::getParameterHandler() const
{
  return param_handler_;
}

rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr ROS2BaseNode::ParamCallback()
const
{
  return param_callback_;
}
}  // namespace kuka_drivers_core
