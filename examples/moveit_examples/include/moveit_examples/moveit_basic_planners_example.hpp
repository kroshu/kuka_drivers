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

#ifndef MOVEIT_EXAMPLES__MOVEIT_BASIC_PLANNERS_EXAMPLE_HPP_
#define MOVEIT_EXAMPLES__MOVEIT_BASIC_PLANNERS_EXAMPLE_HPP_

#include <math.h>

#include <memory>

#include "moveit_examples/moveit_example.hpp"

void moveItBasicPlannersExample(std::shared_ptr<MoveitExample> example_node,
    const geometry_msgs::msg::Vector3 & standing_pose_coords,
    const geometry_msgs::msg::Vector3 & lin_goal_coords,
    const geometry_msgs::msg::Vector3 & collision_box_pos,
    const geometry_msgs::msg::Vector3 & collision_box_size)
{
  example_node->initialize();
  example_node->addBreakPoint();
  
  // Add robot platform
  example_node->addRobotPlatform();

  // Pilz PTP planner
  auto standing_pose =
    Eigen::Isometry3d(Eigen::Translation3d(standing_pose_coords.x, standing_pose_coords.y, standing_pose_coords.z) * Eigen::Quaterniond::Identity());

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
    Eigen::Isometry3d(Eigen::Translation3d(lin_goal_coords.x, lin_goal_coords.y, lin_goal_coords.z) * Eigen::Quaterniond::Identity());
  planned_trajectory =
    example_node->planToPoint(cart_goal, "pilz_industrial_motion_planner", "LIN");
  if (planned_trajectory != nullptr)
  {
    example_node->drawTrajectory(*planned_trajectory);
    example_node->addBreakPoint();
    example_node->moveGroupInterface()->execute(*planned_trajectory);
  }
  example_node->addBreakPoint();

  // Add collision object
  example_node->addCollisionBox(collision_box_pos, collision_box_size);
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
}

#endif  // MOVEIT_EXAMPLES__MOVEIT_BASIC_PLANNERS_EXAMPLE_HPP_
