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

#ifndef KUKA_DRIVERS_CORE__ROS2BASELCNODE_HPP_
#define KUKA_DRIVERS_CORE__ROS2BASELCNODE_HPP_

#include <string>
#include <map>
#include <vector>
#include <memory>

#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "lifecycle_msgs/msg/state.hpp"

#include "kuka_drivers_core/ParameterHandler.hpp"

namespace kuka_drivers_core
{
class ROS2BaseLCNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit ROS2BaseLCNode(
    const std::string & node_name,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State &) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State &) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_shutdown(const rclcpp_lifecycle::State & state) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_error(const rclcpp_lifecycle::State &) override;

  template<typename T>
  void registerParameter(
    const std::string & name, const T & value, const ParameterSetAccessRights & rights,
    std::function<bool(const T &)> on_change_callback)
  {
    param_handler_.registerParameter<T>(
      name, value, rights,
      on_change_callback, this->get_node_parameters_interface());
  }

  template<typename T>
  void registerStaticParameter(
    const std::string & name, const T & value, const ParameterSetAccessRights & rights,
    std::function<bool(const T &)> on_change_callback)
  {
    param_handler_.registerParameter<T>(
      name, value, rights,
      on_change_callback, this->get_node_parameters_interface(), true);
  }
  const ParameterHandler & getParameterHandler() const;

protected:
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr ParamCallback() const;
  static const rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn SUCCESS =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  static const rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn ERROR =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  static const rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn FAILURE =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;

private:
  ParameterHandler param_handler_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_;
};

}  // namespace kuka_drivers_core


#endif  // KUKA_DRIVERS_CORE__ROS2BASELCNODE_HPP_
