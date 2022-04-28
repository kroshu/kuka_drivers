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

#ifndef ROBOT_CONTROL__INTERPOLATING_CONTROLLER_HPP_
#define ROBOT_CONTROL__INTERPOLATING_CONTROLLER_HPP_

#include <string>
#include <vector>
#include <map>

#include "rclcpp/rclcpp.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "kuka_sunrise_interfaces/srv/set_double.hpp"
#include "kuka_sunrise_interfaces/srv/set_int.hpp"

#include "robot_control/joint_controller_base.hpp"


namespace robot_control
{

struct ParameterSetAccessRights
{
  bool unconfigured;
  bool inactive;
  bool active;
  bool finalized;
  bool isSetAllowed(std::uint8_t current_state) const
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
      default:
        return false;
    }
  }
};

class InterpolatingController : public JointControllerBase
{
public:
  InterpolatingController(
    const std::string & node_name,
    const rclcpp::NodeOptions & options);
  ~InterpolatingController() override = default;

protected:
  virtual void controlLoopCallback(sensor_msgs::msg::JointState::SharedPtr measured_joint_state);
  virtual void referenceUpdateCallback(
    sensor_msgs::msg::JointState::SharedPtr reference_joint_state);

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr reference_joint_state_listener_;

  rclcpp::CallbackGroup::SharedPtr cbg_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_;

  rcl_interfaces::msg::SetParametersResult onParamChange(
    const std::vector<rclcpp::Parameter> & parameters);
  bool canSetParameter(const rclcpp::Parameter & param);
  bool onMaxVelocitiesChangeRequest(const rclcpp::Parameter & param);
  bool onLowerLimitsChangeRequest(const rclcpp::Parameter & param);
  bool onUpperLimitsChangeRequest(const rclcpp::Parameter & param);
  virtual void setJointCommandPosition(const std::vector<double> & measured_joint_position);
  virtual void enforceSpeedLimits(const std::vector<double> & measured_joint_position);

  std::map<std::string, struct ParameterSetAccessRights> parameter_set_access_rights_;
};
}  // namespace robot_control


#endif  // ROBOT_CONTROL__INTERPOLATING_CONTROLLER_HPP_
