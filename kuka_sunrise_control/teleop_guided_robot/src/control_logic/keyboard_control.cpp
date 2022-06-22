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

#include <vector>
#include <memory>
#include <string>
#include <cmath>

#include "control_logic/keyboard_control.hpp"

namespace teleop_guided_robot
{

double d2r(double degrees)
{
  return degrees / 180 * M_PI;
}

KeyboardControl::KeyboardControl(const std::string & node_name, const rclcpp::NodeOptions & options)
: kroshu_ros2_core::ROS2BaseLCNode(node_name, options)
{
  auto qos = rclcpp::QoS(rclcpp::KeepAll());

  key_teleop_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "key_vel", qos, [this](geometry_msgs::msg::Twist::SharedPtr msg) {
      this->messageReceivedCallback(msg);
    });
  reference_joint_state_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>(
    "reference_joint_state", qos);
  reference_joint_state_.position.resize(7);

  registerParameter<std::vector<double>>(
    "lower_limits_deg", std::vector<double>(
      {-170, -120, -170, -120, -170, -120,
        -175}), kroshu_ros2_core::ParameterSetAccessRights {true, true,
      true, false}, [this](const std::vector<double> & lower_lim) {
      return this->onLowerLimitsChangeRequest(lower_lim);
    });

  registerParameter<std::vector<double>>(
    "upper_limits_deg", std::vector<double>(
      {170, 120, 170, 120, 170, 120, 175}), kroshu_ros2_core::ParameterSetAccessRights {true, true,
      true, false}, [this](const std::vector<double> & upper_lim) {
      return this->onUpperLimitsChangeRequest(upper_lim);
    });
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn KeyboardControl::
on_cleanup(
  const rclcpp_lifecycle::State &)
{
  reference_joint_state_.position.assign(7, 0);
  active_joint_ = 0;
  return SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn KeyboardControl::
on_activate(
  const rclcpp_lifecycle::State &)
{
  reference_joint_state_publisher_->on_activate();
  last_time_ = this->now();
  reference_joint_state_publisher_->publish(reference_joint_state_);
  return SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn KeyboardControl::
on_deactivate(
  const rclcpp_lifecycle::State &)
{
  reference_joint_state_publisher_->on_deactivate();
  return SUCCESS;
}

void KeyboardControl::messageReceivedCallback(geometry_msgs::msg::Twist::SharedPtr msg)
{
  if (this->get_current_state().label() != "active") {
    return;
  }

  rclcpp::Time current_time = this->now();
  rclcpp::Duration elapsed_time = current_time - last_time_;


  // Update reference only with a max rate of 10 Hz
  if (elapsed_time < elapsed_time_treshold_) {
    return;
  }

  double z = -msg->angular.z;
  if (z > 0) {
    if (!changing_joint_) {
      changing_joint_ = true;
      if (++active_joint_ > 6) {
        active_joint_ = 0;
      }
      RCLCPP_INFO(get_logger(), "Active joint: %i", active_joint_ + 1);
    }
  } else if (z < 0) {
    if (!changing_joint_) {
      changing_joint_ = true;
      if (--active_joint_ < 0) {
        active_joint_ = 6;
      }
      RCLCPP_INFO(get_logger(), "Active joint: %i", active_joint_ + 1);
    }
  } else if (changing_joint_) {
    changing_joint_ = false;
  }

  double x = msg->linear.x;
  double x_dir = x > 0 ? x * WEIGHTS.X_POS : x * WEIGHTS.X_NEG;
  double new_reference_joint_state = reference_joint_state_.position[active_joint_] + x_dir *
    turning_velocity_increment_;
  if (lower_limits_rad_[active_joint_] < new_reference_joint_state &&
    new_reference_joint_state < upper_limits_rad_[active_joint_])
  {
    reference_joint_state_.position[active_joint_] = new_reference_joint_state;
    last_time_ = current_time;
    reference_joint_state_publisher_->publish(reference_joint_state_);
  } else {
    RCLCPP_WARN(get_logger(), "Joint limit reached!");
  }
}

bool KeyboardControl::onLowerLimitsChangeRequest(
  const std::vector<double> & lower_limits)
{
  if (lower_limits.size() != 7) {
    RCLCPP_ERROR(
      this->get_logger(),
      "Invalid parameter array length for lower limits");
    return false;
  }
  std::transform(
    lower_limits.begin(),
    lower_limits.end(), lower_limits_rad_.begin(), [](double v) {
      return d2r(v * 0.9);
    });
  return true;
}

bool KeyboardControl::onUpperLimitsChangeRequest(const std::vector<double> & upper_limits)
{
  if (upper_limits.size() != 7) {
    RCLCPP_ERROR(
      this->get_logger(),
      "Invalid parameter array length for upper limits");
    return false;
  }
  std::transform(
    upper_limits.begin(),
    upper_limits.end(), upper_limits_rad_.begin(), [](double v) {
      return d2r(v * 0.9);
    });
  return true;
}
}  // namespace teleop_guided_robot

int main(int argc, char * argv[])
{
  setvbuf(stdout, nullptr, _IONBF, BUFSIZ);

  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor executor;
  auto node = std::make_shared<teleop_guided_robot::KeyboardControl>(
    "keyboard_control",
    rclcpp::NodeOptions());
  executor.add_node(node->get_node_base_interface());
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
