// Copyright 2025 KUKA Hungaria Kft.
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

#ifndef KUKA_RSI_DRIVER__ROBOT_MANAGER_NODE_EXTENDED_HPP_
#define KUKA_RSI_DRIVER__ROBOT_MANAGER_NODE_EXTENDED_HPP_

#include "std_msgs/msg/u_int32.hpp"
#include "std_msgs/msg/u_int8.hpp"

#include "robot_manager_base.hpp"

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace kuka_rsi_driver
{
class RobotManagerNodeEkiRsi : public RobotManagerBase
{
public:
  RobotManagerNodeEkiRsi();
  ~RobotManagerNodeEkiRsi() = default;

  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;

  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override;

private:
  void EventSubscriptionCallback(const std_msgs::msg::UInt8::SharedPtr message) override;

  bool OnControlModeChangeRequest(const int control_mode) override;

  std::condition_variable control_mode_cv_;
  std::mutex control_mode_cv_m_;
  bool control_mode_change_finished_ = false;
  rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr control_mode_pub_;
};
}  // namespace kuka_rsi_driver

#endif  // KUKA_RSI_DRIVER__ROBOT_MANAGER_NODE_EXTENDED_HPP_
