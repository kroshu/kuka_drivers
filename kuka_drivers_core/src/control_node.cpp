// Copyright 2022 KUKA Hungaria Kft.
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

#include <sys/mman.h>

#include <memory>
#include <thread>

#include "controller_manager/controller_manager.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  auto controller_manager =
    std::make_shared<controller_manager::ControllerManager>(executor, "controller_manager");

  auto qos = rclcpp::QoS(rclcpp::KeepLast(1));
  qos.best_effort();

  std::atomic_bool is_configured = false;
  auto is_configured_sub = controller_manager->create_subscription<std_msgs::msg::Bool>(
    "robot_manager/is_configured", qos,
    [&is_configured](std_msgs::msg::Bool::SharedPtr msg) { is_configured = msg->data; });

  std::thread control_loop(
    [controller_manager, &is_configured]()
    {
      rclcpp::Parameter cpu_affinity_param;
      if (controller_manager->get_parameter("cpu_affinity", cpu_affinity_param))
      {
        int cpu = -1;
        if (cpu_affinity_param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER)
        {
          cpu = static_cast<int>(cpu_affinity_param.as_int());
        }
        else
        {
          RCLCPP_ERROR(
            controller_manager->get_logger(),
            "Invalid parameter type for 'cpu_affinity', it has to be an integer");
        }

        if (cpu >= 0)
        {
          cpu_set_t cpuset;
          CPU_ZERO(&cpuset);
          CPU_SET(cpu, &cpuset);

          pthread_t thread = pthread_self();
          if (pthread_setaffinity_np(thread, sizeof(cpu_set_t), &cpuset) != 0)
          {
            RCLCPP_ERROR(controller_manager->get_logger(), "Failed to set CPU affinity");
            RCLCPP_ERROR(controller_manager->get_logger(), strerror(errno));
          }
          else
          {
            RCLCPP_INFO(controller_manager->get_logger(), "CPU affinity set to core %d", cpu);
          }
        }
      }

      bool lock_memory = controller_manager->get_parameter_or<bool>("lock_memory", true);
      if (lock_memory)
      {
        if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1)
        {
          RCLCPP_ERROR(controller_manager->get_logger(), "mlockall error: %s", strerror(errno));
        }
        else
        {
          RCLCPP_INFO(
            controller_manager->get_logger(),
            "Memory of control loop locked successfully to disable paging");
        }
      }
      else
      {
        RCLCPP_WARN(
          controller_manager->get_logger(),
          "Memory locking disabled, consider enabling it for better real-time performance");
      }

      struct sched_param param;
      param.sched_priority = controller_manager->get_parameter_or<int>("thread_priority", 70);
      if (sched_setscheduler(0, SCHED_FIFO, &param) == -1)
      {
        RCLCPP_ERROR(controller_manager->get_logger(), "setscheduler error");
        RCLCPP_ERROR(controller_manager->get_logger(), strerror(errno));
        RCLCPP_WARN(
          controller_manager->get_logger(),
          "You can use the driver but scheduler priority was not set");
      }
      else
      {
        RCLCPP_INFO(
          controller_manager->get_logger(), "Control loop priority was set to %d",
          param.sched_priority);
      }

      try
      {
        rclcpp::Time current_time = controller_manager->now();
        bool using_fixed_time = false;
        while (rclcpp::ok())
        {
          // Use a fixed period for interpolation, as the interpolation cycle is also fixed on the
          // controller side. Calculating the period from the actual time could cause jitter in the
          // interpolated values
          // TODO: adjust dt for non-integer update rates, (e.g. 12 ms cycle time for RSI IPO mode)
          const rclcpp::Duration dt =
            rclcpp::Duration::from_seconds(1.0 / controller_manager->get_update_rate());

          if (is_configured)
          {
            if (!using_fixed_time)
            {
              using_fixed_time = true;
              current_time = controller_manager->now();
            }

            controller_manager->read(controller_manager->now(), dt);
            controller_manager->update(current_time, dt);
            controller_manager->write(controller_manager->now(), dt);

            // Advance the simulated control time by the fixed dt period
            current_time = rclcpp::Time(current_time.nanoseconds() + dt.nanoseconds());
          }
          else
          {
            using_fixed_time = false;
            controller_manager->update(controller_manager->now(), dt);
            std::this_thread::sleep_for(dt.to_chrono<std::chrono::nanoseconds>());
          }
        }

        // Unlock memory after control loop finishes
        int rc = munlockall();
        if (rc != 0)
        {
          RCLCPP_ERROR(controller_manager->get_logger(), "munlockall error: %s", strerror(errno));
        }
      }
      catch (std::exception & e)
      {
        RCLCPP_ERROR(
          controller_manager->get_logger(), "Quitting control loop due to: %s", e.what());
      }
    });

  executor->add_node(controller_manager);

  executor->spin();
  control_loop.join();

  // shutdown
  rclcpp::shutdown();

  return 0;
}
