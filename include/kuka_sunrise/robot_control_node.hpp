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

#ifndef KUKA_SUNRISE__ROBOT_CONTROL_NODE_HPP_
#define KUKA_SUNRISE__ROBOT_CONTROL_NODE_HPP_

#include <pthread.h>
#include <limits.h>
#include <sched.h>
#include <sys/mman.h>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "std_srvs/srv/set_bool.hpp"

#include "fri_client/friClientApplication.h"
#include "fri_client/friUdpConnection.h"
#include "kuka_sunrise/robot_control_client.hpp"
#include "kuka_sunrise/internal/activatable_interface.hpp"

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

}  // namespace kuka_sunrise
#endif  // KUKA_SUNRISE__ROBOT_CONTROL_NODE_HPP_
