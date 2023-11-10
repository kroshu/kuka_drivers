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

class Depalletizer : public MoveitExample
{
public:
  void Depalletize()
  {
    for (int k = 0; k < 3; k++) {
      for (int j = 0; j < 3; j++) {
        for (int i = 0; i < 3; i++) {
          std::string object_name = "pallet_" + std::to_string(9 * k + 3 * j + i);
          RCLCPP_INFO(LOGGER, "Going for object %s", object_name.c_str());

          // Go to pickup
          Eigen::Isometry3d pose = Eigen::Isometry3d(
            Eigen::Translation3d(
              0.3 + i * 0.1, j * 0.1 - 0.1,
              0.35 - 0.1 * k) *
            Eigen::Quaterniond(0, 1, 0, 0));
          auto planned_trajectory =
            planToPointUntilSuccess(pose, "ompl", "RRTConnectkConfigDefault");
          if (planned_trajectory != nullptr) {
            move_group_interface_->execute(*planned_trajectory);
          } else {
            RCLCPP_ERROR(LOGGER, "Planning failed");
          }

          // Attach object
          AttachObject(object_name);

          // Drop off to -0.3, 0.0, 0.35 pointing down
          Eigen::Isometry3d dropoff_pose = Eigen::Isometry3d(
            Eigen::Translation3d(-0.3, 0.0, 0.35) * Eigen::Quaterniond(0, 1, 0, 0));
          auto drop_trajectory = planToPointUntilSuccess(
            dropoff_pose, "ompl",
            "RRTConnectkConfigDefault");
          if (drop_trajectory != nullptr) {
            move_group_interface_->execute(*drop_trajectory);
          } else {
            RCLCPP_ERROR(LOGGER, "Planning failed");
          }

          // Detach
          DetachAndRemoveObject(object_name);
        }
      }
    }
  }
};

int main(int argc, char * argv[])
{
  // Setup
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<Depalletizer>();
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread(
    [&executor]()
    {executor.spin();})
  .detach();

  node->initialize();

  node->moveGroupInterface()->setMaxVelocityScalingFactor(1.0);
  node->moveGroupInterface()->setMaxAccelerationScalingFactor(1.0);
  // Add robot platform
  node->addRobotPlatform();
  node->addBreakPoint();

  // Add pallets
  node->addPalletObjects();
  node->addBreakPoint();

  node->Depalletize();

  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}
