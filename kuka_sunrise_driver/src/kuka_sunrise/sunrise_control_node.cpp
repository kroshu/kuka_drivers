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
#include "std_msgs/msg/int32.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  int fri_state_ = 0;
  auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  auto controller_manager = std::make_shared<controller_manager::ControllerManager>(
    executor,
    "controller_manager");
  std::thread control_loop([controller_manager, &fri_state_]() {
      const rclcpp::Duration dt =
      rclcpp::Duration::from_seconds(1.0 / controller_manager->get_update_rate());

      auto callback = [&fri_state_, controller_manager](std_msgs::msg::Int32::SharedPtr state) {
        RCLCPP_INFO(controller_manager->get_logger(), "State received: %i", state->data);
        fri_state_ = state->data;
      };
      auto sub =
      controller_manager->create_subscription<std_msgs::msg::Int32>("fri_state", 1, callback);
      // TODO(Svastits): sync controller start state with robot pose at startup
      //  currently if robot is not in candle, speed limit is exceeded at startup
      while (rclcpp::ok()) {
        controller_manager->read(controller_manager->now(), dt);
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
