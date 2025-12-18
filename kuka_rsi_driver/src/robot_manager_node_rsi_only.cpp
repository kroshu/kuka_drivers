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
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
RobotManagerNodeRsi::on_configure(const rclcpp_lifecycle::State &)
{
  std::vector<std::string> controllers_to_activate{kuka_drivers_core::EVENT_BROADCASTER};

  return RobotManagerBase::configure(controllers_to_activate);
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
RobotManagerNodeRsi::on_cleanup(const rclcpp_lifecycle::State &)
{
  // Deactivate event broadcaster
  std::vector<std::string> controllers_to_deactivate{kuka_drivers_core::EVENT_BROADCASTER};

  return RobotManagerBase::cleanup(controllers_to_deactivate);
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
