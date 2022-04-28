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

#include "rclcpp/rclcpp.hpp"
#include "robot_control/joint_controller_base.hpp"

namespace robot_control
{
class JointController : public JointControllerBase
{
public:
  JointController(
    const std::string & node_name,
    const rclcpp::NodeOptions & options);
    
private:
  void controlLoopCallback(sensor_msgs::msg::JointState::SharedPtr measured_joint_state);

};
}  // namespace robot_control


#endif  // ROBOT_CONTROL__JOINT_CONTROLLER_HPP_
