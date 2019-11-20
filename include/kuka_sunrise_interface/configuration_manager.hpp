/*
 * configuration_manager.hpp
 *
 *  Created on: Nov 19, 2019
 *      Author: rosdeveloper
 */

#ifndef INCLUDE_KUKA_SUNRISE_INTERFACE_CONFIGURATION_MANAGER_HPP_
#define INCLUDE_KUKA_SUNRISE_INTERFACE_CONFIGURATION_MANAGER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "std_srvs/srv/set_bool.hpp"

#include "map"

namespace kuka_sunrise_interface{

class RobotManager;

struct ParameterSetAccessRights{
  bool unconfigured;
  bool inactive;
  bool active;
  bool finalized;
  bool isSetAllowed(std::uint8_t current_state){
    switch(current_state){
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

class ConfigurationManager{
public:
  ConfigurationManager(rclcpp_lifecycle::LifecycleNode::SharedPtr robot_manager_node, std::shared_ptr<RobotManager> robot_manager);
  rcl_interfaces::msg::SetParametersResult onParamChange(const std::vector<rclcpp::Parameter>& parameters);

private:
  rclcpp_lifecycle::LifecycleNode::SharedPtr robot_manager_node_;
  std::shared_ptr<RobotManager> robot_manager_;
  rclcpp::callback_group::CallbackGroup::SharedPtr cbg_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr robot_control_client_;
  std::map<std::string, struct ParameterSetAccessRights> parameter_set_access_rights_;

  bool canSetParameter(const rclcpp::Parameter& param);
  bool onCommandModeChangeRequest(const rclcpp::Parameter& param);
  bool onControlModeChangeRequest(const rclcpp::Parameter& param);
  bool onJointStiffnessChangeRequest(const rclcpp::Parameter& param);
  bool onJointDampingChangeRequest(const rclcpp::Parameter& param);
  bool setCommandMode(const std::string& control_mode);
};

}



#endif /* INCLUDE_KUKA_SUNRISE_INTERFACE_CONFIGURATION_MANAGER_HPP_ */
