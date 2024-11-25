// Copyright 2024 Gergely Kovács
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

#ifndef MOVEIT_EXAMPLES__MOVEIT_CONSTRAINED_PLANNING_EXAMPLE_HPP_
#define MOVEIT_EXAMPLES__MOVEIT_CONSTRAINED_PLANNING_EXAMPLE_HPP_

#include <math.h>
#include <memory>

#include "moveit_examples/moveit_example.hpp"

void moveItConstrainedPlanningExample(std::shared_ptr<MoveitExample> example_node,
    const geometry_msgs::msg::Vector3 & start_coords,
    const geometry_msgs::msg::Vector3 & end_goal,
    const geometry_msgs::msg::Vector3 & collision_box_pos,
    const geometry_msgs::msg::Vector3 & collision_box_size,
    double constraint_tolerance)
{
  example_node->initialize();

  // Add robot platform
  example_node->addRobotPlatform();

  // Go to correct position for the example
  auto start_pos =
    Eigen::Isometry3d(Eigen::Translation3d(start_coords.x, start_coords.y, start_coords.z) * Eigen::Quaterniond::Identity());
  auto init_trajectory = example_node->planToPoint(start_pos);
  if (init_trajectory != nullptr)
  {
    example_node->moveGroupInterface()->execute(*init_trajectory);
  }

  // Add collision object
  example_node->addCollisionBox(collision_box_pos, collision_box_size);
  example_node->addBreakPoint();

  auto cart_goal =
    Eigen::Isometry3d(Eigen::Translation3d(end_goal.x, end_goal.y, end_goal.z) * Eigen::Quaterniond::Identity());

  geometry_msgs::msg::Quaternion q;
  q.x = 0;
  q.y = 0;
  q.z = 0;
  q.w = 1;

  example_node->moveGroupInterface()->setPlanningTime(30.0);

  example_node->setOrientationConstraint(q, constraint_tolerance);
  // Plan with collision avoidance
  auto planned_trajectory =
    example_node->planToPointUntilSuccess(cart_goal, "ompl", "RRTConnectkConfigDefault");
  if (planned_trajectory != nullptr)
  {
    example_node->drawTrajectory(*planned_trajectory);
    example_node->addBreakPoint();
    example_node->moveGroupInterface()->execute(*planned_trajectory);
  }
}

#endif  // MOVEIT_EXAMPLES__MOVEIT_CONSTRAINED_PLANNING_EXAMPLE_HPP_