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

#ifndef MOVEIT_EXAMPLES__MOVEIT_COLLISION_AVOIDANCE_EXAMPLE_HPP_
#define MOVEIT_EXAMPLES__MOVEIT_COLLISION_AVOIDANCE_EXAMPLE_HPP_

#include <math.h>
#include <memory>

#include "moveit_examples/moveit_example.hpp"

void moveItCollisionAvoidanceExample(std::shared_ptr<MoveitExample> example_node,
    const geometry_msgs::msg::Vector3 & start_coords,
    const geometry_msgs::msg::Vector3 & standing_pose_coords,
    const geometry_msgs::msg::Vector3 & collision_box_pos,
    const geometry_msgs::msg::Vector3 & collision_box_size)
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

  auto standing_pose =
    Eigen::Isometry3d(Eigen::Translation3d(standing_pose_coords.x, standing_pose_coords.y, standing_pose_coords.z) * Eigen::Quaterniond::Identity());

  // Plan with collision avoidance
  auto planned_trajectory =
    example_node->planToPoint(standing_pose, "ompl", "RRTConnectkConfigDefault");
  if (planned_trajectory != nullptr)
  {
    example_node->drawTrajectory(*planned_trajectory);
    example_node->addBreakPoint();
    example_node->moveGroupInterface()->execute(*planned_trajectory);
  }
}

#endif  // MOVEIT_EXAMPLES__MOVEIT_COLLISION_AVOIDANCE_EXAMPLE_HPP_