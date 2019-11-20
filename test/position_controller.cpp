/*
 * position_controller.cpp
 *
 *  Created on: Nov 13, 2019
 *      Author: rosdeveloper
 */

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include <rclcpp/message_memory_strategy.hpp>
#include <math.h>

using rclcpp::message_memory_strategy::MessageMemoryStrategy;

class PositionController : public rclcpp::Node{
private:
  sensor_msgs::msg::JointState command_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscription_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_command_publisher_;
  rclcpp::Clock ros_clock_;
  int joint_mask_;
  double offset_, ampl_rad_, phi_, freq_hz_, filter_coeff_, step_width_;
public:
  PositionController():
    Node("position_controller"),
    ros_clock_(RCL_ROS_TIME),
    joint_mask_(0x8),
    offset_(0),
    ampl_rad_(1),
    phi_(0),
    freq_hz_(1),
    filter_coeff_(0.99),
    step_width_(0)
  {
    step_width_ = 2*M_PI*freq_hz_*0.01;
    command_.position.reserve(7);
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1));
    qos.best_effort();
    auto callback = [this](sensor_msgs::msg::JointState::ConstSharedPtr msg)->void{this->loopCallback(msg);};
    auto msg_strategy = std::make_shared<MessageMemoryStrategy<sensor_msgs::msg::JointState>>();//TODO use TLSFAllocator? implement static strategy for jointstatemsg?
    joint_state_subscription_ =
          this->create_subscription<sensor_msgs::msg::JointState>("lbr_joint_state", qos,
                                                                  callback,
                                                                  rclcpp::SubscriptionOptions(), msg_strategy);
    joint_command_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("lbr_joint_command", qos);
  }
  void loopCallback(sensor_msgs::msg::JointState::ConstSharedPtr msg){
    //RCLCPP_INFO(get_logger(), "joint state received");
    double newOffset = ampl_rad_ * std::sin(phi_);
    offset_ = offset_ * filter_coeff_ + newOffset * (1.0 - filter_coeff_);
    phi_ += step_width_;
    if(phi_ >= 2* M_PI){
      phi_ -= 2*M_PI;
    }
    double jointPos[7];
    if(msg->position.empty()){
      //RCLCPP_WARN(get_logger(), "joint position data is empty");
    }
    if(msg->position.size() != 7){
      //RCLCPP_WARN(get_logger(), "joint position count is not 7");
    }
    memcpy(jointPos, msg->position.data(), 7*sizeof(double));
    for(int i = 0; i < 7; i++){
      if(joint_mask_ & (1<<i)){
        jointPos[i] += offset_;
      }
    }
    command_.header.frame_id = "world";
    command_.header.stamp = msg->header.stamp;
    command_.position.assign(jointPos, jointPos + 7);
    //RCLCPP_INFO(get_logger(), "command calculated");
    joint_command_publisher_->publish(command_);
  }


};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PositionController>()->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
