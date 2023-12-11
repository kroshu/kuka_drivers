// Copyright 2022 Komáromi Sándor
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

#ifndef KUKA_IIQKA_EAC_DRIVER__ROBOT_MANAGER_NODE_HPP_
#define KUKA_IIQKA_EAC_DRIVER__ROBOT_MANAGER_NODE_HPP_

#include <string>
#include <memory>
#include <vector>
#include <map>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/client.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "controller_manager_msgs/srv/set_hardware_component_state.hpp"
#include "controller_manager_msgs/srv/switch_controller.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/u_int32.hpp"

#include "communication_helpers/service_tools.hpp"
#include "kuka_drivers_core/ROS2BaseLCNode.hpp"
#include "kuka_drivers_core/ControllerHandler.hpp"

#include "kuka/ecs/v1/motion_services_ecs.grpc.pb.h"

namespace kuka_rox
{
class RobotManagerNode : public kuka_drivers_core::ROS2BaseLCNode
{
public:
  RobotManagerNode();
  ~RobotManagerNode();


  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State &) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State &) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &) override;

private:
  void ObserveControl();
  bool onControlModeChangeRequest(int control_mode);
  bool onRobotModelChangeRequest(const std::string & robot_model);

  rclcpp::Client<controller_manager_msgs::srv::SetHardwareComponentState>::SharedPtr
    change_hardware_state_client_;
  rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr
    change_controller_state_client_;
  rclcpp::CallbackGroup::SharedPtr cbg_;
  std::string robot_model_;

  kuka_drivers_core::ControllerHandler controller_handler_;

  std::thread observe_thread_;
  std::atomic<bool> terminate_{false};
  bool param_declared_ = false;
#ifdef NON_MOCK_SETUP
  std::unique_ptr<kuka::ecs::v1::ExternalControlService::Stub> stub_;
  std::unique_ptr<grpc::ClientContext> context_;

  std::condition_variable control_mode_cv_;
  std::mutex control_mode_cv_m_;
  bool control_mode_change_finished_;
#endif

  rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr control_mode_pub_;

  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Bool>> is_configured_pub_;
  std_msgs::msg::Bool is_configured_msg_;

  // There are two kinds of control modes with different number of necessary interfaces to be set:
  //  - in standard modes (position, torque), only the control signal to the used interface (1)
  //  - in impedance modes, the setpoint and the parameters describing the behaviour (2)
  static constexpr int STANDARD_MODE_IF_SIZE = 1;
  static constexpr int IMPEDANCE_MODE_IF_SIZE = 2;
};

}   // namespace kuka_rox


#endif  // KUKA_IIQKA_EAC_DRIVER__ROBOT_MANAGER_NODE_HPP_
