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

#ifndef KUKA_SUNRISE_FRI_DRIVER__CONFIGURATION_MANAGER_HPP_
#define KUKA_SUNRISE_FRI_DRIVER__CONFIGURATION_MANAGER_HPP_

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "controller_manager_msgs/srv/list_controllers.hpp"
#include "kuka_driver_interfaces/srv/set_fri_configuration.hpp"
#include "kuka_sunrise_fri_driver/fri_connection.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "std_msgs/msg/u_int32.hpp"
#include "std_srvs/srv/set_bool.hpp"

#include "kuka_drivers_core/control_mode.hpp"
#include "kuka_drivers_core/ros2_base_lc_node.hpp"

namespace kuka_sunrise_fri_driver
{

class RobotManager;

class ConfigurationManager
{
public:
  explicit ConfigurationManager(
    std::shared_ptr<kuka_drivers_core::ROS2BaseLCNode> robot_manager_node);

  std::string getRobotModel() { return robot_model_; }
  std::string GetControllerName();

private:
  std::shared_ptr<kuka_drivers_core::ROS2BaseLCNode> robot_manager_node_;
  rclcpp::CallbackGroup::SharedPtr cbg_;
  rclcpp::Client<kuka_driver_interfaces::srv::SetFriConfiguration>::SharedPtr fri_config_client_;
  rclcpp::Client<controller_manager_msgs::srv::ListControllers>::SharedPtr get_controllers_client_;
  rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr control_mode_pub_;
  std_msgs::msg::UInt32 control_mode_msg_;

  int receive_multiplier_;
  int send_period_ms_;
  std::string robot_model_;
  std::string joint_pos_controller_name_;
  std::string joint_torque_controller_name_;

  bool onControlModeChangeRequest(int control_mode);
  bool onRobotModelChangeRequest(const std::string & robot_model);
  bool onSendPeriodChangeRequest(int send_period);
  bool setReceiveMultiplier(int receive_multiplier);
  bool onReceiveMultiplierChangeRequest(const int & receive_multiplier);
  bool onControllerIpChangeRequest(const std::string & controller_ip) const;
  bool onControllerNameChangeRequest(
    const std::string & controller_name, kuka_drivers_core::ControllerType controller_type);
  bool setFriConfiguration(int send_period_ms, int receive_multiplier);
};
}  // namespace kuka_sunrise_fri_driver

#endif  // KUKA_SUNRISE_FRI_DRIVER__CONFIGURATION_MANAGER_HPP_
