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
#include "rclcpp/exceptions.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "rclcpp/message_memory_strategy.hpp"

using rclcpp::message_memory_strategy::MessageMemoryStrategy;

class PositionController : public rclcpp::Node
{
private:
  sensor_msgs::msg::JointState command_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscription_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_command_publisher_;
  rclcpp::Clock ros_clock_;
  int joint_mask_;
  int receive_multiplier_;
  int receive_counter_;
  double offset_, ampl_rad_, phi_, freq_hz_, step_width_;

public:
  PositionController()
  : Node("torque_controller", rclcpp::NodeOptions().allow_undeclared_parameters(true)),
    ros_clock_(RCL_ROS_TIME), joint_mask_(0x8), receive_multiplier_(0), receive_counter_(0),
    offset_(0), ampl_rad_(3), phi_(0), freq_hz_(0.25), step_width_(0)
  {
    step_width_ = 2 * M_PI * freq_hz_ * 0.01;
    command_.position.reserve(7);
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1));
    qos.best_effort();
    auto callback = [this](sensor_msgs::msg::JointState::ConstSharedPtr msg) -> void {
        this->loopCallback(msg);
      };
    // TODO(resizoltan) use TLSFAllocator? implement static strategy for jointstatemsg?
    auto msg_strategy = std::make_shared<MessageMemoryStrategy<sensor_msgs::msg::JointState>>();
    joint_state_subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "lbr_joint_state", qos, callback, rclcpp::SubscriptionOptions(), msg_strategy);
    joint_command_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>(
      "lbr_joint_command", qos);
  }
  void loopCallback(sensor_msgs::msg::JointState::ConstSharedPtr msg)
  {
    // RCLCPP_INFO(get_logger(), "joint state received");
    if (receive_multiplier_ == 0) {
      try {
        receive_multiplier_ = this->get_parameter("receive_multiplier").as_int();
      } catch (const rclcpp::exceptions::ParameterNotDeclaredException & e) {
        RCLCPP_INFO(get_logger(), "Parameter receive_multiplier not set, using default: 1");
        receive_multiplier_ = 1;
      } catch (...) {
        RCLCPP_INFO(get_logger(), "error");
        receive_multiplier_ = 1;
      }
      receive_counter_ = receive_multiplier_;
    }
    if (--receive_counter_ == 0) {
      receive_counter_ = receive_multiplier_;
      offset_ = ampl_rad_ * std::sin(phi_);
      phi_ += step_width_;
      if (phi_ >= 2 * M_PI) {
        phi_ -= 2 * M_PI;
      }
      double jointTorque[7];
      for (int i = 0; i < 7; i++) {
        if (joint_mask_ & (1 << i)) {
          jointTorque[i] = offset_;
        }
      }
      command_.header.frame_id = "world";
      command_.header.stamp = msg->header.stamp;
      command_.effort.assign(jointTorque, jointTorque + 7);
      // RCLCPP_INFO(get_logger(), "command calculated");
      joint_command_publisher_->publish(command_);
    }
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  rclcpp::spin(std::make_shared<PositionController>()->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
