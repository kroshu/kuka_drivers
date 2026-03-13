// Copyright 2023 KUKA Hungaria Kft.
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

#ifndef KUKA_RSI_DRIVER__ROBOT_MANAGER_BASE_HPP_
#define KUKA_RSI_DRIVER__ROBOT_MANAGER_BASE_HPP_

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "controller_manager_msgs/srv/set_hardware_component_state.hpp"
#include "controller_manager_msgs/srv/switch_controller.hpp"
#include "rclcpp/client.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/u_int8.hpp"

#include "kuka_drivers_core/controller_handler.hpp"
#include "kuka_drivers_core/ros2_base_lc_node.hpp"

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace kuka_rsi_driver
{
class RobotManagerBase : public kuka_drivers_core::ROS2BaseLCNode
{
public:
  RobotManagerBase();
  virtual ~RobotManagerBase() = default;

  CallbackReturn configure_driver(const std::vector<std::string> & controllers_to_activate);

  CallbackReturn cleanup_driver(const std::vector<std::string> & controllers_to_deactivate);

  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;

protected:
  bool onRobotModelChangeRequest(const std::string & robot_model);
  virtual void EventSubscriptionCallback(const std_msgs::msg::UInt8::SharedPtr message);
  virtual bool OnControlModeChangeRequest(const int control_mode);
  virtual bool OnControlModeChangeRequestAdditionalTasks([[maybe_unused]] const int control_mode)
  {
    return true;
  }

  enum class CycleTime
  {
    UNDEFINED = -1,
    RSI_4MS = 1,
    RSI_12MS = 2
  };

  bool ChangeCycleTime(CycleTime cycle_time);
  bool ValidateCycleTime(CycleTime cycle_time);

  // Convert CycleTime enum to human-readable string
  inline const char * CycleTimeToString(CycleTime cycle_time)
  {
    switch (cycle_time)
    {
      case CycleTime::RSI_4MS:
        return "1 (4ms)";
      case CycleTime::RSI_12MS:
        return "2 (12ms)";
      case CycleTime::UNDEFINED:
        return "undefined";
      default:
        return std::to_string(static_cast<int>(cycle_time)).c_str();
    }
  }

  inline int CycleTimeToInt(CycleTime cycle_time)
  {
    switch (cycle_time)
    {
      case CycleTime::RSI_4MS:
        return 4;
      case CycleTime::RSI_12MS:
        return 12;
      case CycleTime::UNDEFINED:
        return -1;
      default:
        return -1;
    }
  }

  rclcpp::Client<controller_manager_msgs::srv::SetHardwareComponentState>::SharedPtr
    change_hardware_state_client_;
  rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr
    change_controller_state_client_;
  rclcpp::CallbackGroup::SharedPtr cbg_;
  rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedPtr set_param_client_;
  std::string robot_model_;
  bool use_gpio_ = false;
  std::string position_controller_name_;

  kuka_drivers_core::ControllerHandler controller_handler_;
  kuka_drivers_core::ControlMode control_mode_ =
    kuka_drivers_core::ControlMode::CONTROL_MODE_UNSPECIFIED;

  std::atomic<bool> terminate_{false};

  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Bool>> is_configured_pub_;
  std_msgs::msg::Bool is_configured_msg_;

  rclcpp::CallbackGroup::SharedPtr event_callback_group_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr event_subscriber_;

  // publisher and backing field for cycle time
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr cycle_time_pub_;
  CycleTime cycle_time_{CycleTime::UNDEFINED};  // 1 => 4 ms (RSI_4MS), 2 => 12 ms (RSI_12MS)

  static constexpr int HARDWARE_ACTIVATION_TIMEOUT_MS = 15'000;
  static constexpr int HARDWARE_DEACTIVATION_TIMEOUT_MS = 15'000;
};
}  // namespace kuka_rsi_driver

#endif  // KUKA_RSI_DRIVER__ROBOT_MANAGER_BASE_HPP_
