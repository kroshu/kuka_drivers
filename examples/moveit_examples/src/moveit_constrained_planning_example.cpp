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

#include <math.h>
#include <memory>

#include "moveit_examples/moveit_example.hpp"

int main(int argc, char * argv[])
{
  // Setup
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const example_node = std::make_shared<MoveitExample>();
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(example_node);
  std::thread([&executor]() { executor.spin(); }).detach();

  example_node->initialize();

  // Add robot platform
  example_node->addRobotPlatform();

  // Go to correct position for the example
  auto init_trajectory = example_node->planToPosition(
    std::vector<double>{0.0017, -2.096, 1.514, 0.0012, -0.9888, -0.0029});
  if (init_trajectory != nullptr)
  {
    example_node->moveGroupInterface()->execute(*init_trajectory);
  }

  // Add collision object
  example_node->addCollisionBox(
    geometry_msgs::build<geometry_msgs::msg::Vector3>().x(0.125).y(0.15).z(0.5),
    geometry_msgs::build<geometry_msgs::msg::Vector3>().x(0.1).y(1.0).z(0.1));
  example_node->addBreakPoint();

  auto cart_goal =
    Eigen::Isometry3d(Eigen::Translation3d(0.4, -0.15, 0.55) * Eigen::Quaterniond::Identity());

  geometry_msgs::msg::Quaternion q;
  q.x = 0;
  q.y = 0;
  q.z = 0;
  q.w = 1;

  example_node->moveGroupInterface()->setPlanningTime(30.0);

  example_node->setOrientationConstraint(q);
  // Plan with collision avoidance
  auto planned_trajectory =
    example_node->planToPointUntilSuccess(cart_goal, "ompl", "RRTkConfigDefault");
  if (planned_trajectory != nullptr)
  {
    example_node->drawTrajectory(*planned_trajectory);
    example_node->addBreakPoint();
    example_node->moveGroupInterface()->execute(*planned_trajectory);
  }

  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}
