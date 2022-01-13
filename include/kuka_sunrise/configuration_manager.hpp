// Copyright 2020 Zoltán Rési
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

#ifndef KUKA_SUNRISE__CONFIGURATION_MANAGER_HPP_
#define KUKA_SUNRISE__CONFIGURATION_MANAGER_HPP_

#include <map>
#include <vector>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "kuka_sunrise_interfaces/srv/set_int.hpp"

namespace kuka_sunrise
{

class RobotManager;

struct ParameterSetAccessRights
{
  bool unconfigured;
  bool inactive;
  bool active;
  bool finalized;
  bool configuring;
  bool isSetAllowed(std::uint8_t current_state)
  {
    switch (current_state) {
      case lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED:
        return unconfigured;
      case lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE:
        return inactive;
      case lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE:
        return active;
      case lifecycle_msgs::msg::State::PRIMARY_STATE_FINALIZED:
        return finalized;
      case lifecycle_msgs::msg::State::TRANSITION_STATE_CONFIGURING:
        return configuring;
      default:
        return false;
    }
  }
};

class ConfigurationManager
{
public:
  ConfigurationManager(
    rclcpp_lifecycle::LifecycleNode::SharedPtr robot_manager_node,
    std::shared_ptr<RobotManager> robot_manager);

private:
  rclcpp_lifecycle::LifecycleNode::SharedPtr robot_manager_node_;
  std::shared_ptr<RobotManager> robot_manager_;
  rclcpp::callback_group::CallbackGroup::SharedPtr cbg_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr command_mode_client_;
  rclcpp::Client<kuka_sunrise_interfaces::srv::SetInt>::SharedPtr receive_multiplier_client_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr set_parameter_service_;
  std::map<std::string, struct ParameterSetAccessRights> parameter_set_access_rights_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_;

  std::vector<double> joint_stiffness_temp_;
  std::vector<double> joint_damping_temp_;

  rcl_interfaces::msg::SetParametersResult onParamChange(
    const std::vector<rclcpp::Parameter> & parameters);
  bool canSetParameter(const rclcpp::Parameter & param);
  bool onCommandModeChangeRequest(const rclcpp::Parameter & param);
  bool onControlModeChangeRequest(const rclcpp::Parameter & param);
  bool onJointStiffnessChangeRequest(const rclcpp::Parameter & param);
  bool onJointDampingChangeRequest(const rclcpp::Parameter & param);
  bool onSendPeriodChangeRequest(const rclcpp::Parameter & param);
  bool onReceiveMultiplierChangeRequest(const rclcpp::Parameter & param);
  bool onControllerIpChangeRequest(const rclcpp::Parameter & param);
  bool setCommandMode(const std::string & control_mode);
  bool setReceiveMultiplier(int receive_multiplier);
};

}  // namespace kuka_sunrise

#endif  // KUKA_SUNRISE__CONFIGURATION_MANAGER_HPP_
