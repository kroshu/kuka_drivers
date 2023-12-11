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

#include "kuka_drivers_core/ParameterHandler.hpp"

#include <string>
#include <vector>
#include <memory>

namespace kuka_drivers_core
{
ParameterHandler::ParameterHandler(rclcpp_lifecycle::LifecycleNode * node)
: node_(node)
{
}

rcl_interfaces::msg::SetParametersResult ParameterHandler::onParamChange(
  const std::vector<rclcpp::Parameter> & parameters) const
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = false;
  for (const rclcpp::Parameter & param : parameters) {
    auto found_param_it = std::find_if(
      params_.begin(), params_.end(),
      [&param](auto param_ptr) {
        return param_ptr->getName() == param.get_name();
      });
    // When used properly, we should not reach this
    // but better to keep additional check to filter improper use
    if (found_param_it == params_.end()) {
      printf("Invalid parameter name\n");
    } else if (canSetParameter(*(*found_param_it))) {
      result.successful = (*found_param_it)->callCallback(param);
    }
  }
  return result;
}

bool ParameterHandler::canSetParameter(const ParameterBase & param) const
{
  if (node_ == nullptr) {
    // Node is not lifecycle node, paramater can always be set
    return true;
  }
  try {
    if (!param.getRights().isSetAllowed(node_->get_current_state().id())) {
      RCLCPP_ERROR(
        node_->get_logger(),
        "Parameter %s cannot be changed while in state %s",
        param.getName().c_str(), node_->get_current_state().label().c_str());
      return false;
    }
  } catch (const std::out_of_range &) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "Parameter set access rights for parameter %s couldn't be determined",
      param.getName().c_str());
    return false;
  }
  return true;
}

void ParameterHandler::registerParameter(
  std::shared_ptr<ParameterBase> param_shared_ptr,
  bool block)
{
  params_.emplace_back(param_shared_ptr);
  param_shared_ptr->getParameterInterface()->declare_parameter(
    param_shared_ptr->getName(), param_shared_ptr->getDefaultValue());
  if (block) {
    param_shared_ptr->blockParameter();
  }
}

}  // namespace kuka_drivers_core
