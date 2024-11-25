// Copyright 2022 Áron Svastits
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

#include "moveit_examples/moveit_collision_avoidance_example.hpp"

int main(int argc, char * argv[])
{
  // Setup
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const example_node = std::make_shared<MoveitExample>();
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(example_node);
  std::thread([&executor]() { executor.spin(); }).detach();

  moveItCollisionAvoidanceExample(example_node,
      std::vector<double>{0.2111, -0.9106, 1.2075, 0.0, -1.8677,  -0.2111},
      geometry_msgs::build<geometry_msgs::msg::Vector3>().x(0.1).y(0).z(1.2),
      geometry_msgs::build<geometry_msgs::msg::Vector3>().x(0.3).y(-0.075).z(1),
      geometry_msgs::build<geometry_msgs::msg::Vector3>().x(0.1).y(1.0).z(0.1));

  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}
