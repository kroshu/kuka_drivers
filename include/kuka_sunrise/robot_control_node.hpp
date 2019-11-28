/*
 * robot_control_node.hpp
 *
 *  Created on: Nov 11, 2019
 *      Author: rosdeveloper
 */

#ifndef INCLUDE_KUKA_SUNRISE_ROBOT_CONTROL_NODE_HPP_
#define INCLUDE_KUKA_SUNRISE_ROBOT_CONTROL_NODE_HPP_

#include "fri_client/friClientApplication.h"
#include "fri_client/friUdpConnection.h"
#include "kuka_sunrise/robot_control_client.hpp"
#include "kuka_sunrise/internal/activatable_interface.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "std_srvs/srv/set_bool.hpp"

#include "pthread.h"
#include "limits.h"
#include "sched.h"
#include "sys/mman.h"

namespace kuka_sunrise
{

class RobotControlNode : public rclcpp_lifecycle::LifecycleNode, public ActivatableInterface
{
public:
  RobotControlNode();
  ~RobotControlNode();
  void runClientApplication();

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

  virtual bool activate();
  virtual bool deactivate();

private:
  KUKA::FRI::UdpConnection udp_connection_;
  std::unique_ptr<RobotControlClient> client_;
  std::unique_ptr<KUKA::FRI::ClientApplication> client_application_;

  std::unique_ptr<pthread_t> client_application_thread_;
  std::atomic_bool close_requested_;
  rclcpp::callback_group::CallbackGroup::SharedPtr cbg_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr set_command_state_service_;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn SUCCESS =
      rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn ERROR =
      rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn FAILURE =
      rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;

};

}
#endif /* INCLUDE_KUKA_SUNRISE_ROBOT_CONTROL_NODE_HPP_ */
