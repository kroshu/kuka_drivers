/*
 * keyboard_control.hpp
 *
 *  Created on: Dec 2, 2019
 *      Author: rosdeveloper
 */

#ifndef INCLUDE_CONTROL_LOGIC_KEYBOARD_CONTROL_HPP_
#define INCLUDE_CONTROL_LOGIC_KEYBOARD_CONTROL_HPP_

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/twist.hpp"

namespace teleop_guided_robot{

struct ParameterSetAccessRights
{
  bool unconfigured;
  bool inactive;
  bool active;
  bool finalized;
  bool isSetAllowed(std::uint8_t current_state) const
  {
    switch (current_state)
    {
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

class KeyboardControl : public rclcpp_lifecycle::LifecycleNode
{
public:
  KeyboardControl(const std::string& node_name, const rclcpp::NodeOptions& options);

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State&);

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State&);

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_shutdown(const rclcpp_lifecycle::State&);

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State&);

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State&);

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_error(const rclcpp_lifecycle::State&);


private:
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::JointState>::SharedPtr reference_joint_state_publisher_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr key_teleop_subscription_;
  void messageReceivedCallback(geometry_msgs::msg::Twist::SharedPtr msg);

  rcl_interfaces::msg::SetParametersResult onParamChange(const std::vector<rclcpp::Parameter> &parameters);
  bool canSetParameter(const rclcpp::Parameter &param);
  bool onLowerLimitsChangeRequest(const rclcpp::Parameter& param);
  bool onUpperLimitsChangeRequest(const rclcpp::Parameter& param);
  std::map<std::string, struct ParameterSetAccessRights> parameter_set_access_rights_;
  std::vector<double> lower_limits_rad_;
  std::vector<double> upper_limits_rad_;
  sensor_msgs::msg::JointState::SharedPtr reference_joint_state_;
  int active_joint_;
  bool changing_joint_;

  const struct{
    double X_POS = 1.25;
    double X_NEG = 2;
    double Z = 1;
  } WEIGHTS;

  const double turning_velocity_increment_;
  const rclcpp::Duration elapsed_time_treshold_;
  rclcpp::Time last_time_;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn SUCCESS =
      rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn ERROR =
      rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn FAILURE =
      rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
};


}



#endif /* INCLUDE_CONTROL_LOGIC_KEYBOARD_CONTROL_HPP_ */
