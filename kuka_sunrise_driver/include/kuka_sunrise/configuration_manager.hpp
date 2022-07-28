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
#include "controller_manager_msgs/srv/list_controllers.hpp"

#include "kroshu_ros2_core/ROS2BaseLCNode.hpp"

namespace kuka_sunrise
{

class RobotManager;

class ConfigurationManager
{
public:
  ConfigurationManager(
    std::shared_ptr<kroshu_ros2_core::ROS2BaseLCNode> robot_manager_node,
    std::shared_ptr<RobotManager> robot_manager);

private:
  bool configured_ = false;
  bool position_controller_available_ = false;
  bool torque_controller_available_ = false;
  std::shared_ptr<kroshu_ros2_core::ROS2BaseLCNode> robot_manager_node_;
  std::shared_ptr<RobotManager> robot_manager_;
  rclcpp::CallbackGroup::SharedPtr cbg_;
  rclcpp::CallbackGroup::SharedPtr param_cbg_;
  rclcpp::Client<kuka_sunrise_interfaces::srv::SetInt>::SharedPtr receive_multiplier_client_;
  rclcpp::Client<controller_manager_msgs::srv::ListControllers>::SharedPtr get_controllers_client_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr set_parameter_service_;

  std::vector<double> joint_stiffness_ = std::vector<double>(7, 1000.0);
  std::vector<double> joint_damping_ = std::vector<double>(7, 0.7);

  const std::string POSITION_COMMAND = "position";
  const std::string TORQUE_COMMAND = "torque";
  const std::string POSITION_CONTROL = "position";
  const std::string IMPEDANCE_CONTROL = "joint_impedance";

  bool onCommandModeChangeRequest(const std::string & command_mode) const;
  bool onControlModeChangeRequest(const std::string & control_mode) const;
  bool onJointStiffnessChangeRequest(const std::vector<double> & joint_stiffness);
  bool onJointDampingChangeRequest(const std::vector<double> & joint_damping);
  bool onSendPeriodChangeRequest(const int & send_period) const;
  bool onReceiveMultiplierChangeRequest(const int & receive_multiplier) const;
  bool onControllerIpChangeRequest(const std::string & controller_ip) const;
  bool onControllerNameChangeRequest(const std::string & controller_name, bool position);
  bool setCommandMode(const std::string & control_mode) const;
  bool setReceiveMultiplier(int receive_multiplier) const;
  void setParameters(std_srvs::srv::Trigger::Response::SharedPtr response);
};
}  // namespace kuka_sunrise

#endif  // KUKA_SUNRISE__CONFIGURATION_MANAGER_HPP_
