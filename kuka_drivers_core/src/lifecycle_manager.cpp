// Copyright 2026 KUKA Hungaria Kft.
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

#include <chrono>
#include <string>
#include <thread>

#include "communication_helpers/service_tools.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "rclcpp/rclcpp.hpp"

/**
 * @brief A node that drives a lifecycle node through configure and activate transitions.
 *
 * Uses the /<managed_node>/change_state service directly (with wait-for-service retry)
 * to avoid the "Node not found" flakiness seen with CLI tools in CI environments.
 *
 * Parameters:
 *   - managed_node   (string, default "robot_manager"): name of the target lifecycle node
 *   - configure_delay (double, default 20.0):  seconds from startup to send configure
 *   - activate_delay  (double, default 25.0):  seconds from startup to send activate
 */
class LifecycleManager : public rclcpp::Node
{
public:
  explicit LifecycleManager(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("lifecycle_manager", options)
  {
    this->declare_parameter<std::string>("managed_node", "robot_manager");
    this->declare_parameter<double>("configure_delay", 20.0);
    this->declare_parameter<double>("activate_delay", 25.0);

    const auto managed_node = this->get_parameter("managed_node").as_string();
    configure_delay_ = this->get_parameter("configure_delay").as_double();
    activate_delay_ = this->get_parameter("activate_delay").as_double();

    change_state_client_ =
      this->create_client<lifecycle_msgs::srv::ChangeState>(managed_node + "/change_state");

    // Run delayed transitions in a worker thread so the executor can keep spinning
    // and respond quickly to shutdown while timing logic runs independently.
    worker_thread_ = std::thread([this]() { this->run(); });
  }

  ~LifecycleManager()
  {
    if (worker_thread_.joinable())
    {
      worker_thread_.join();
    }
  }

private:
  bool sleepInterruptible(double delay_s, const char * phase)
  {
    if (delay_s <= 0.0)
    {
      return rclcpp::ok();
    }

    const auto delay = std::chrono::duration<double>(delay_s);
    const auto start = std::chrono::steady_clock::now();

    while (rclcpp::ok() && (std::chrono::steady_clock::now() - start) < delay)
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    if (!rclcpp::ok())
    {
      const auto elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - start);
      RCLCPP_WARN(
        this->get_logger(), "Interrupted while waiting for %s: held %.3f s out of %.3f s", phase,
        elapsed.count(), delay_s);
      return false;
    }

    return true;
  }

  bool changeState(uint8_t transition_id)
  {
    auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
    request->transition.id = transition_id;

    auto response = kuka_drivers_core::sendRequest<lifecycle_msgs::srv::ChangeState::Response>(
      change_state_client_, request, 1000, 5000);

    if (!response)
    {
      RCLCPP_ERROR(this->get_logger(), "Change state service call failed");
      return false;
    }

    if (!response->success)
    {
      RCLCPP_ERROR(
        this->get_logger(), "Change state transition %u rejected by managed node", transition_id);
      return false;
    }

    return true;
  }

  void run()
  {
    // Wait for configure_delay seconds, then trigger configure
    if (!sleepInterruptible(configure_delay_, "configure delay"))
    {
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Sending 'configure' transition");
    if (!changeState(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE))
    {
      RCLCPP_ERROR(this->get_logger(), "'configure' transition failed");
      return;
    }

    // Wait for the remaining time before activate
    const double remaining_delay = activate_delay_ - configure_delay_;
    if (remaining_delay > 0.0)
    {
      if (!sleepInterruptible(remaining_delay, "activate delay"))
      {
        return;
      }
    }

    RCLCPP_INFO(this->get_logger(), "Sending 'activate' transition");
    if (!changeState(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE))
    {
      RCLCPP_ERROR(this->get_logger(), "'activate' transition failed");
    }
  }

  rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr change_state_client_;
  std::thread worker_thread_;
  double configure_delay_{20.0};
  double activate_delay_{25.0};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LifecycleManager>());
  rclcpp::shutdown();
  return 0;
}
