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

#include <memory>
#include <string>
#include <vector>

#include "kuka_drivers_core/ros2_base_lc_node.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace kuka_drivers_core
{

ROS2BaseLCNode::ROS2BaseLCNode(const std::string & node_name, const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode(node_name, options)
{
  param_handler_ = ParameterHandler(this);
  param_callback_ =
    this->add_on_set_parameters_callback([this](const std::vector<rclcpp::Parameter> & parameters)
                                         { return param_handler_.onParamChange(parameters); });
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
ROS2BaseLCNode::on_configure(const rclcpp_lifecycle::State &)
{
  return SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
ROS2BaseLCNode::on_cleanup(const rclcpp_lifecycle::State &)
{
  return SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
ROS2BaseLCNode::on_shutdown(const rclcpp_lifecycle::State & state)
{
  auto result = SUCCESS;
  switch (state.id())
  {
    case lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE:
      result = this->on_deactivate(get_current_state());
      if (result != SUCCESS)
      {
        break;
      }
      result = this->on_cleanup(get_current_state());
      break;
    case lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE:
      result = this->on_cleanup(get_current_state());
      break;
    case lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED:
      break;
    default:
      break;
  }
  return result;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
ROS2BaseLCNode::on_activate(const rclcpp_lifecycle::State &)
{
  return SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
ROS2BaseLCNode::on_deactivate(const rclcpp_lifecycle::State &)
{
  return SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn ROS2BaseLCNode::on_error(
  const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "An error occurred");
  return SUCCESS;
}

const ParameterHandler & ROS2BaseLCNode::getParameterHandler() const { return param_handler_; }

rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr ROS2BaseLCNode::ParamCallback()
  const
{
  return param_callback_;
}
}  // namespace kuka_drivers_core
