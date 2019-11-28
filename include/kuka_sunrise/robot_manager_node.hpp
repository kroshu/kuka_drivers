/*
 * robot_manager_node.hpp
 *
 *  Created on: Nov 12, 2019
 *      Author: rosdeveloper
 */

#ifndef INCLUDE_KUKA_SUNRISE_ROBOT_MANAGER_NODE_HPP_
#define INCLUDE_KUKA_SUNRISE_ROBOT_MANAGER_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/client.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/msg/state.hpp"

#include "kuka_sunrise/robot_manager.hpp"
#include "kuka_sunrise/configuration_manager.hpp"

#include "atomic"

namespace kuka_sunrise
{

class RobotManagerNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  RobotManagerNode();

  virtual rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State&);

  virtual rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State&);

  virtual rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_shutdown(const rclcpp_lifecycle::State&);

  virtual rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State&);

  virtual rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State&);

  virtual rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_error(const rclcpp_lifecycle::State&);

private:
  std::shared_ptr<RobotManager> robot_manager_;
  std::unique_ptr<ConfigurationManager> configuration_manager_;
  rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr change_robot_control_state_client_;
  rclcpp::callback_group::CallbackGroup::SharedPtr cbg_;
  bool requestRobotControlNodeStateTransition(std::uint8_t transition);
  void handleControlEndedError();
  void handleFRIEndedError();

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn SUCCESS =
      rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn ERROR =
      rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn FAILURE =
      rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
};

}

#endif /* INCLUDE_KUKA_SUNRISE_ROBOT_MANAGER_NODE_HPP_ */
