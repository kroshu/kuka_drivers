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

#ifndef ROBOT_CONTROL__JOINT_CONTROLLER_HPP_
#define ROBOT_CONTROL__JOINT_CONTROLLER_HPP_

#include <string>
#include <vector>
#include <map>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "kuka_sunrise_interfaces/srv/set_double.hpp"
#include "kuka_sunrise_interfaces/srv/set_int.hpp"

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

class JointController : public rclcpp_lifecycle::LifecycleNode
{
public:
  JointController(
    const std::string & node_name,
    const rclcpp::NodeOptions & options);
  ~JointController() override = default;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State &);

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State &);

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_shutdown(const rclcpp_lifecycle::State &);

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &);

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &);

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_error(const rclcpp_lifecycle::State &);

protected:
  void jointStateMeasurementsCallback(sensor_msgs::msg::JointState::SharedPtr measured_joint_state);
  virtual void controlLoopCallback(sensor_msgs::msg::JointState::SharedPtr measured_joint_state);
  virtual void referenceUpdateCallback(
    sensor_msgs::msg::JointState::SharedPtr reference_joint_state);
  sensor_msgs::msg::JointState::SharedPtr referenceJointState() const;
  const std::vector<double> & maxVelocitiesRadPs() const;
  const std::vector<double> & lowerLimitsRad() const;
  const std::vector<double> & upperLimitsRad() const;

private:
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr reference_joint_state_listener_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr measured_joint_state_listener_;
  rclcpp::Service<kuka_sunrise_interfaces::srv::SetDouble>::SharedPtr set_rate_service_;
  rclcpp::Service<kuka_sunrise_interfaces::srv::SetInt>::SharedPtr sync_send_period_service_;
  rclcpp::Service<kuka_sunrise_interfaces::srv::SetInt>::SharedPtr sync_receive_multiplier_service_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::JointState>::SharedPtr
    joint_command_publisher_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Bool>::SharedPtr
    joint_controller_is_active_publisher_;

  sensor_msgs::msg::JointState::SharedPtr reference_joint_state_;
  sensor_msgs::msg::JointState::SharedPtr joint_command_;
  std_msgs::msg::Bool::SharedPtr joint_controller_is_active_;
  rclcpp::CallbackGroup::SharedPtr cbg_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_;

  rcl_interfaces::msg::SetParametersResult onParamChange(
    const std::vector<rclcpp::Parameter> & parameters);
  bool canSetParameter(const rclcpp::Parameter & param);
  bool onMaxVelocitiesChangeRequest(const rclcpp::Parameter & param);
  bool onLowerLimitsChangeRequest(const rclcpp::Parameter & param);
  bool onUpperLimitsChangeRequest(const rclcpp::Parameter & param);
  bool onVelocityScalingChangeRequest(const rclcpp::Parameter & param);
  void updateMaxPositionDifference();
  void setJointCommandPositionWithVelocity(const std::vector<double> & measured_joint_position);
  void enforceSpeedLimits(const std::vector<double> & measured_joint_position);

  std::map<std::string, struct ParameterSetAccessRights> parameter_set_access_rights_;
  std::vector<double> max_velocities_radPs_ = std::vector<double>(7);
  std::vector<double> max_position_difference_ = std::vector<double>(7);
  std::vector<double> lower_limits_rad_ = std::vector<double>(7);
  std::vector<double> upper_limits_rad_ = std::vector<double>(7);
  std::vector<double> prev_ref_joint_pos_ = std::vector<double>(7);
  std::vector<bool> slow_start_ = std::vector<bool>(7, true);

  int send_period_ms_ = 10;
  int receive_multiplier_ = 1;
  int loop_period_ms_ = 10;
  int loop_count_ = 0;
  int cmd_count_ = 0;
  int cmd_per_frame_temp_ = 0;  // for syncing changing with commands
  int cmd_per_frame_ = 13;  // default for 8Hz frequency of camera
  bool start_flag_ = true;
  bool velocity_scaling_ = true;
  static constexpr int ms_in_sec_ = 1000;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn SUCCESS =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn ERROR =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn FAILURE =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
};
}  // namespace robot_control


#endif  // ROBOT_CONTROL__JOINT_CONTROLLER_HPP_
