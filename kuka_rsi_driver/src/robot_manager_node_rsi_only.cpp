// Copyright 2023 KUKA Hungaria Kft.
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

#include "communication_helpers/ros2_control_tools.hpp"
#include "communication_helpers/service_tools.hpp"

#include "kuka_drivers_core/controller_names.hpp"
#include "kuka_drivers_core/hardware_event.hpp"
#include "kuka_rsi_driver/robot_manager_node_rsi_only.hpp"

using namespace controller_manager_msgs::srv;  // NOLINT
using namespace lifecycle_msgs::msg;           // NOLINT

namespace kuka_rsi_driver
{
<<<<<<< HEAD
RobotManagerNodeRsi::RobotManagerNodeRsi() : kuka_drivers_core::ROS2BaseLCNode("robot_manager")
{
  auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
  qos.reliable();
  cbg_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  change_hardware_state_client_ = this->create_client<SetHardwareComponentState>(
    "controller_manager/set_hardware_component_state", qos.get_rmw_qos_profile(), cbg_);
  change_controller_state_client_ = this->create_client<SwitchController>(
    "controller_manager/switch_controller", qos.get_rmw_qos_profile(), cbg_);

  auto is_configured_qos = rclcpp::QoS(rclcpp::KeepLast(1));
  is_configured_qos.best_effort();

  is_configured_pub_ =
    this->create_publisher<std_msgs::msg::Bool>("robot_manager/is_configured", is_configured_qos);

  // Subscribe to event_broadcaster/hardware_event
  rclcpp::SubscriptionOptions sub_options;
  event_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  sub_options.callback_group = event_callback_group_;
  event_subscriber_ = create_subscription<std_msgs::msg::UInt8>(
    "event_broadcaster/hardware_event", rclcpp::SystemDefaultsQoS(),
    [this](const std_msgs::msg::UInt8::SharedPtr message) { EventSubscriptionCallback(message); },
    sub_options);

  this->registerStaticParameter<std::string>(
    "robot_model", "kr6_r700_sixx", kuka_drivers_core::ParameterSetAccessRights{false, false},
    [this](const std::string & robot_model)
    { return this->onRobotModelChangeRequest(robot_model); });

  this->registerStaticParameter<bool>(
    "use_gpio", false, kuka_drivers_core::ParameterSetAccessRights{false, false},
    [this](const bool use_gpio)
    {
      use_gpio_ = use_gpio;
      return true;
    });

  this->registerParameter<std::string>(
    "position_controller_name", kuka_drivers_core::JOINT_TRAJECTORY_CONTROLLER,
    kuka_drivers_core::ParameterSetAccessRights{true, false},
    [this](const std::string & controller_name)
    {
      this->position_controller_name_ = controller_name;
      return true;
    });
}

=======
>>>>>>> acb5556 (Add mxAutomation support and refactor (#284))
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
RobotManagerNodeRsi::on_configure(const rclcpp_lifecycle::State &)
{
  std::vector<std::string> controllers_to_activate{kuka_drivers_core::EVENT_BROADCASTER};

  return RobotManagerBase::configure_driver(controllers_to_activate);
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
RobotManagerNodeRsi::on_cleanup(const rclcpp_lifecycle::State &)
{
  // Deactivate event broadcaster
  std::vector<std::string> controllers_to_deactivate{kuka_drivers_core::EVENT_BROADCASTER};

  return RobotManagerBase::cleanup_driver(controllers_to_deactivate);
}

}  // namespace kuka_rsi_driver

int main(int argc, char * argv[])
{
  setvbuf(stdout, nullptr, _IONBF, BUFSIZ);

  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor;
  auto node = std::make_shared<kuka_rsi_driver::RobotManagerNodeRsi>();
  executor.add_node(node->get_node_base_interface());
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
