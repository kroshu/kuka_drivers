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

#include "robot_control/joint_controller.hpp"

#include <sys/mman.h>
#include <string>
#include <memory>
#include <vector>

namespace robot_control
{

double d2r(double degrees)
{
  return degrees / 180 * M_PI;
}

JointController::JointController(
  const std::string & node_name,
  const rclcpp::NodeOptions & options)
: JointControllerBase(node_name, options)
{

}

void JointController::controlLoopCallback(
  sensor_msgs::msg::JointState::SharedPtr measured_joint_state)
{
  joint_command_->header = measured_joint_state->header;
  joint_command_publisher_->publish(*joint_command_);
}

}  // namespace robot_control

int main(int argc, char * argv[])
{
  setvbuf(stdout, nullptr, _IONBF, BUFSIZ);

  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor executor;
  auto node = std::make_shared<robot_control::JointController>(
    "joint_controller", rclcpp::NodeOptions());
  executor.add_node(node->get_node_base_interface());
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
