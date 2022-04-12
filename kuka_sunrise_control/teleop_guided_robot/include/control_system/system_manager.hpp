/*
 * system_manager.hpp
 *
 *  Created on: Dec 3, 2019
 *      Author: rosdeveloper
 */

#ifndef INCLUDE_CONTROL_SYSTEM_KEYBOARD_CONTROL_HPP_
#define INCLUDE_CONTROL_SYSTEM_KEYBOARD_CONTROL_HPP_

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_srvs/srv/set_bool.hpp"

namespace teleop_guided_robot
{

class SystemManager : public rclcpp_lifecycle::LifecycleNode
{
public:
  SystemManager(const std::string& node_name, const rclcpp::NodeOptions& options);

  void activateControl();
  void deactivateControl();

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

  bool changeState(const std::string& node_name, std::uint8_t transition);
  bool changeRobotCommandingState(bool is_active);
  void robotCommandingStateChanged(bool is_active);
  std::vector<rclcpp::Subscription<lifecycle_msgs::msg::Transition>::SharedPtr> lifecycle_subscriptions_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr robot_commanding_state_subscription_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr change_robot_commanding_state_client_;
  bool robot_control_active_;

  rclcpp::QoS qos_;
  rclcpp::callback_group::CallbackGroup::SharedPtr cbg_;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn SUCCESS =
      rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn ERROR =
      rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn FAILURE =
      rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;

  const std::string JOINT_CONTROLLER = "joint_controller";
  const std::string CONTROL_LOGIC = "keyboard_control";
  const std::string ROBOT_INTERFACE = "robot_manager";
};




}

#endif /* INCLUDE_CONTROL_SYSTEM_KEYBOARD_CONTROL_HPP_ */

