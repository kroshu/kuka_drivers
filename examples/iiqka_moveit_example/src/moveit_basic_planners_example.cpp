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

#include "iiqka_moveit_example/moveit_example.hpp"

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
  example_node->addBreakPoint();

  // Add robot platform
  example_node->addRobotPlatform();

  // Pilz PTP planner
  auto standing_pose =
    Eigen::Isometry3d(Eigen::Translation3d(0.1, 0, 0.8) * Eigen::Quaterniond::Identity());

  auto planned_trajectory =
    example_node->planToPoint(standing_pose, "pilz_industrial_motion_planner", "PTP");
  if (planned_trajectory != nullptr)
  {
    example_node->drawTrajectory(*planned_trajectory);
    example_node->addBreakPoint();
    example_node->moveGroupInterface()->execute(*planned_trajectory);
  }
  example_node->addBreakPoint();

  // Pilz LIN planner
  auto cart_goal =
    Eigen::Isometry3d(Eigen::Translation3d(0.4, -0.15, 0.55) * Eigen::Quaterniond::Identity());
  planned_trajectory =
    example_node->planToPoint(cart_goal, "pilz_industrial_motion_planner", "LIN");
  if (planned_trajectory != nullptr)
  {
    example_node->drawTrajectory(*planned_trajectory);
    example_node->addBreakPoint();
    example_node->moveGroupInterface()->execute(*planned_trajectory);
  }

  // Add collision object
  example_node->addCollisionBox(
    geometry_msgs::build<geometry_msgs::msg::Vector3>().x(0.25).y(-0.075).z(0.675),
    geometry_msgs::build<geometry_msgs::msg::Vector3>().x(0.1).y(0.4).z(0.1));
  example_node->addBreakPoint();

  // Try moving back with Pilz LIN
  planned_trajectory =
    example_node->planToPoint(standing_pose, "pilz_industrial_motion_planner", "LIN");
  if (planned_trajectory != nullptr)
  {
    example_node->drawTrajectory(*planned_trajectory);
  }
  else
  {
    example_node->drawTitle("Failed planning with Pilz LIN");
  }
  example_node->addBreakPoint();

  // Try moving back with Pilz PTP
  planned_trajectory =
    example_node->planToPoint(standing_pose, "pilz_industrial_motion_planner", "PTP");
  if (planned_trajectory != nullptr)
  {
    example_node->drawTrajectory(*planned_trajectory);
  }
  else
  {
    example_node->drawTitle("Failed planning with Pilz PTP");
  }
  example_node->addBreakPoint();

  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}
