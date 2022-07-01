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

#include <thread>
#include <memory>

#include "controller_manager/controller_manager.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  auto controller_manager = std::make_shared<controller_manager::ControllerManager>(executor, "controller_manager");

  std::thread control_loop([controller_manager]() {
    const rclcpp::Duration dt = rclcpp::Duration::from_seconds(1.0 / controller_manager->get_update_rate());

    // TODO(Svastits): wait for FRI state changed
    while (rclcpp::ok()) {
      controller_manager->read(controller_manager->now(), dt);  // Blocking until state received
      printf("AFTER READ\n");
      controller_manager->update(controller_manager->now(), dt);
      controller_manager->write(controller_manager->now(), dt);
    }
  });

  executor->add_node(controller_manager);
  
  executor->spin();
  control_loop.join();

  // shutdown
  rclcpp::shutdown();

  return 0;
}
