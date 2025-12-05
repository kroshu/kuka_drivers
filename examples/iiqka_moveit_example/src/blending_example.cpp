// Copyright 2024 Áron Svastits
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


// Workaround for trajectories not strictly increasing timstamps
void shiftRobotTrajectory(moveit_msgs::msg::RobotTrajectory& trajectory) {
    int time_shift_ns = 0;

    for (size_t index = 0; index < trajectory.joint_trajectory.points.size(); ++index) {
        if (index == 0) {
            continue;
        }

        trajectory.joint_trajectory.points[index].time_from_start.nanosec += time_shift_ns;

        if (trajectory.joint_trajectory.points[index-1].time_from_start == trajectory.joint_trajectory.points[index].time_from_start) {
            trajectory.joint_trajectory.points[index].time_from_start.nanosec += static_cast<int>(1e8);
            time_shift_ns += static_cast<int>(1e8);
        }
    }
}


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

  // Define goals
  auto standing_pose =
    Eigen::Isometry3d(Eigen::Translation3d(0.1, 0, 0.8) * Eigen::Quaterniond::Identity());
  auto cart_goal =
    Eigen::Isometry3d(Eigen::Translation3d(0.2, -0.15, 0.6) * Eigen::Quaterniond::Identity());
  auto cart_goal2 =
    Eigen::Isometry3d(Eigen::Translation3d(0.4, -0.15, 0.55) * Eigen::Quaterniond::Identity());

  std::vector<MotionSegment> motion_sequence;
  motion_sequence.emplace_back(standing_pose, "PTP", 0.01);
  motion_sequence.emplace_back(cart_goal, "LIN", 0.05);
  motion_sequence.emplace_back(cart_goal2, "LIN", 0.0);

  auto planned_trajectory = example_node->blend(motion_sequence);

  if (planned_trajectory != nullptr)
  {
    shiftRobotTrajectory(*planned_trajectory);
    example_node->drawTrajectory(*planned_trajectory);
    example_node->addBreakPoint();
    example_node->moveGroupInterface()->execute(*planned_trajectory);
  }

  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}
