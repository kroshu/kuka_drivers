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

#ifndef ROBOT_CONTROL__RATE_SCALED_CONTROLLER_HPP_
#define ROBOT_CONTROL__RATE_SCALED_CONTROLLER_HPP_

#include <string>
#include <vector>
#include <map>

#include "rclcpp/rclcpp.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "kuka_driver_interfaces/srv/set_double.hpp"
#include "kuka_driver_interfaces/srv/set_int.hpp"

#include "robot_control/interpolating_controller.hpp"


namespace robot_control
{

class RateScaledJointController : public InterpolatingController
{
public:
  RateScaledJointController(
    const std::string & node_name,
    const rclcpp::NodeOptions & options);
  ~RateScaledJointController() override = default;

private:
  void setJointCommandPosition(const std::vector<double> & measured_joint_position) final;
  void enforceSpeedLimits(const std::vector<double> & measured_joint_position) final;
  void setSlowStart();
  bool OnReferenceRateChangeRequest(const double & reference_rate);

  rclcpp::Service<kuka_driver_interfaces::srv::SetDouble>::SharedPtr set_rate_service_;

  std::vector<double> prev_ref_joint_pos_ = std::vector<double>(n_dof_);
  std::vector<double> pprev_ref_joint_pos_ = std::vector<double>(n_dof_);
  std::vector<bool> slow_start_ = std::vector<bool>(n_dof_, true);

  int cmd_count_ = 0;
  int cmd_per_frame_temp_ = 0;  // for syncing changing with commands
  int cmd_per_frame_;
  bool start_flag_ = true;
};
}  // namespace robot_control


#endif  // ROBOT_CONTROL__RATE_SCALED_CONTROLLER_HPP_
