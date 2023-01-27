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
#include "std_msgs/msg/bool.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  auto controller_manager = std::make_shared<controller_manager::ControllerManager>(
    executor,
    "controller_manager");

  auto qos = rclcpp::QoS(rclcpp::KeepLast(1));
  qos.best_effort();

  std::atomic_bool is_configured = true;
  // auto is_configured_sub = controller_manager->create_subscription<std_msgs::msg::Bool>(
  //   "robot_manager/is_configured", qos,
  //   [&is_configured](std_msgs::msg::Bool::SharedPtr msg) {
  //     is_configured = msg->data;
  //   });


  std::thread control_loop([controller_manager, &is_configured]() {
      const rclcpp::Duration dt =
      rclcpp::Duration::from_seconds(1.0 / controller_manager->get_update_rate());
      std::chrono::milliseconds dt_ms {1000 / controller_manager->get_update_rate()};

      while (rclcpp::ok()) {
        if (is_configured) {
          std::this_thread::sleep_for(std::chrono::milliseconds(1));  // TODO(Svastits): temporary workaround
          controller_manager->read(controller_manager->now(), dt);
          controller_manager->update(controller_manager->now(), dt);
          controller_manager->write(controller_manager->now(), dt);
        } else {
          std::this_thread::sleep_for(dt_ms);
        }
      }
    });

  executor->add_node(controller_manager);

  executor->spin();
  control_loop.join();

  // shutdown
  rclcpp::shutdown();

  return 0;
}
