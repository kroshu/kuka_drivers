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

#ifndef KUKA_DRIVERS_CORE__PARAMETER_HANDLER_HPP_
#define KUKA_DRIVERS_CORE__PARAMETER_HANDLER_HPP_

#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/node_interfaces/node_parameters_interface.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace kuka_drivers_core
{
struct ParameterSetAccessRights
{
  bool inactive;
  bool active;
  bool isSetAllowed(std::uint8_t current_state) const
  {
    switch (current_state)
    {
      case lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED:
        return true;
      case lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE:
        return inactive;
      case lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE:
        return active;
      default:
        return false;
    }
  }
};

class ParameterHandler
{
  class ParameterBase
  {
  public:
    ParameterBase(
      const std::string & name, const ParameterSetAccessRights & rights,
      rclcpp::node_interfaces::NodeParametersInterface::SharedPtr param_IF)
    : name_(name), rights_(rights), paramIF_(param_IF)
    {
    }

    virtual ~ParameterBase() = default;

    const std::string & getName() const { return name_; }

    const ParameterSetAccessRights & getRights() const { return rights_; }

    const rclcpp::ParameterValue & getDefaultValue() const { return default_value_; }

    rclcpp::node_interfaces::NodeParametersInterface::SharedPtr getParameterInterface() const
    {
      return paramIF_;
    }

    virtual void blockParameter() = 0;

    virtual bool callCallback(const rclcpp::Parameter &) const { return false; }

  protected:
    void setDefaultValue(rclcpp::ParameterValue && value) { default_value_ = value; }
    const std::string name_;

  private:
    const ParameterSetAccessRights rights_;
    rclcpp::node_interfaces::NodeParametersInterface::SharedPtr paramIF_;
    rclcpp::ParameterValue default_value_;
  };

  template <typename T>
  class Parameter : public ParameterBase
  {
  public:
    Parameter(
      const std::string & name, const T & value, const ParameterSetAccessRights & rights,
      std::function<bool(const T &)> on_change_callback,
      std::shared_ptr<rclcpp::node_interfaces::NodeParametersInterface> paramIF)
    : ParameterBase(name, rights, paramIF), on_change_callback_(on_change_callback)
    {
      setDefaultValue(rclcpp::ParameterValue(value));
    }

    ~Parameter() override = default;

    bool callCallback(const rclcpp::Parameter & new_param) const override
    {
      try
      {
        return on_change_callback_(new_param.get_value<T>());
      }
      catch (const rclcpp::exceptions::InvalidParameterTypeException & e)
      {
        printf("%s", e.what());
        return false;
      }
    }

    void blockParameter() override
    {
      on_change_callback_ = [this](const T &) -> bool
      {
        printf("Parameter %s can be set only at startup\n", name_.c_str());
        return false;
      };
    }

  private:
    std::function<bool(const T &)> on_change_callback_;
  };

public:
  explicit ParameterHandler(rclcpp_lifecycle::LifecycleNode * node = nullptr);

  rcl_interfaces::msg::SetParametersResult onParamChange(
    const std::vector<rclcpp::Parameter> & parameters) const;
  bool canSetParameter(const ParameterBase & param) const;

  template <typename T>
  void registerParameter(
    const std::string & name, const T & value, const ParameterSetAccessRights & rights,
    std::function<bool(const T &)> on_change_callback,
    rclcpp::node_interfaces::NodeParametersInterface::SharedPtr param_IF, bool block = false)
  {
    auto param_shared_ptr = std::make_shared<ParameterHandler::Parameter<T>>(
      name, value, rights, on_change_callback, param_IF);
    registerParameter(param_shared_ptr, block);
  }
  template <typename T>
  void registerParameter(
    const std::string & name, const T & value, std::function<bool(const T &)> on_change_callback,
    rclcpp::node_interfaces::NodeParametersInterface::SharedPtr param_IF, bool block = false)
  {
    auto param_shared_ptr = std::make_shared<ParameterHandler::Parameter<T>>(
      name, value, ParameterSetAccessRights(), on_change_callback, param_IF);
    registerParameter(param_shared_ptr, block);
  }

private:
  std::vector<std::shared_ptr<ParameterBase>> params_;
  rclcpp_lifecycle::LifecycleNode * node_;
  void registerParameter(std::shared_ptr<ParameterBase> param_shared_ptr, bool block);
};
}  // namespace kuka_drivers_core

#endif  // KUKA_DRIVERS_CORE__PARAMETER_HANDLER_HPP_
