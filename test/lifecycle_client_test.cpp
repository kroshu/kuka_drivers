// Copyright 2020 Zoltán Rési
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

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executor.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "rclcpp/message_memory_strategy.hpp"

using rclcpp::message_memory_strategy::MessageMemoryStrategy;

template<typename FutureT, typename WaitTimeT>
std::future_status wait_for_result(FutureT & future, WaitTimeT time_to_wait)
{
  auto end = std::chrono::steady_clock::now() + time_to_wait;
  std::chrono::milliseconds wait_period(100);
  std::future_status status = std::future_status::timeout;
  do {
    auto now = std::chrono::steady_clock::now();
    auto time_left = end - now;
    if (time_left <= std::chrono::seconds(0)) {
      break;
    }
    status = future.wait_for((time_left < wait_period) ? time_left : wait_period);
  } while (rclcpp::ok() && status != std::future_status::ready);
  return status;
}

class LifecycleClientTester : public rclcpp::Node
{
public:
  LifecycleClientTester()
  : Node("LifecycleClientTest")
  {
    auto qos = rclcpp::QoS(rclcpp::KeepAll());
    qos.reliable();
    service_cbg_ = this->create_callback_group(
      rclcpp::callback_group::CallbackGroupType::MutuallyExclusive);
    topic_cbg_ = this->create_callback_group(
      rclcpp::callback_group::CallbackGroupType::MutuallyExclusive);
    auto sub_opt = rclcpp::SubscriptionOptions();
    sub_opt.callback_group = topic_cbg_;
    client_ = this->create_client<lifecycle_msgs::srv::ChangeState>("/lc_talker/change_state",
        qos.get_rmw_qos_profile(),
        service_cbg_);
    getter_client_ = this->create_client<lifecycle_msgs::srv::GetState>("/lc_talker/get_state");
    trigger_ = this->create_subscription<std_msgs::msg::Bool>(
      "change_state", qos, [this](std_msgs::msg::Bool::SharedPtr) {
        auto result = this->configure();
        RCLCPP_INFO(get_logger(), "%u", result);
      } /*,  sub_opt*/);
  }

  virtual rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn configure()
  {
    auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
    request->transition.id = lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE;

    if (!client_->wait_for_service(std::chrono::milliseconds(2000))) {
      RCLCPP_ERROR(get_logger(), "Wait for service failed");
      return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
    }
    auto future_result =
      client_->async_send_request(
      request,
      [this](std::shared_future<lifecycle_msgs::srv::ChangeState::Response::SharedPtr>)
      {RCLCPP_INFO(get_logger(), "callback called");});
    auto future_status2 = wait_for_result(future_result, std::chrono::milliseconds(10000));
    if (future_status2 != std::future_status::ready) {
      RCLCPP_ERROR(get_logger(), "Future status not ready");
      return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
    }
    if (future_result.get()->success) {
      return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    } else {
      RCLCPP_ERROR(get_logger(), "Future result not success");
      return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
    }
  }

  unsigned int get_state(std::chrono::seconds time_out)
  {
    auto request = std::make_shared<lifecycle_msgs::srv::GetState::Request>();

    if (!getter_client_->wait_for_service(time_out)) {
      RCLCPP_ERROR(get_logger(), "Service %s is not available.",
        getter_client_->get_service_name());
      return lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
    }

    // We send the service request for asking the current
    // state of the lc_talker node.
    auto future_result = getter_client_->async_send_request(request);

    // Let's wait until we have the answer from the node.
    // If the request times out, we return an unknown state.
    auto future_status = wait_for_result(future_result, time_out);

    if (future_status != std::future_status::ready) {
      RCLCPP_ERROR(get_logger(), "Server time out while getting current state");
      return lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
    }

    // We have an succesful answer. So let's print the current state.
    if (future_result.get()) {
      RCLCPP_INFO(get_logger(), "current state %s.",
        future_result.get()->current_state.label.c_str());
      return future_result.get()->current_state.id;
    } else {
      RCLCPP_ERROR(get_logger(), "Failed to get current state");
      return lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
    }
  }

private:
  rclcpp::callback_group::CallbackGroup::SharedPtr topic_cbg_;
  rclcpp::callback_group::CallbackGroup::SharedPtr service_cbg_;
  rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr client_;
  rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr getter_client_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr trigger_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor(
    rclcpp::executor::create_default_executor_arguments(), 2);
  auto node = std::make_shared<LifecycleClientTester>();
  executor.add_node(node->get_node_base_interface());
  executor.spin();
  // rclcpp::spin_until_future_complete(node->get_node_base_interface());
  // rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
