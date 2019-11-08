/*
 * robot_commander.cpp
 *
 *  Created on: Nov 5, 2019
 *      Author: rosdeveloper
 */

#include "kuka_sunrise_interface/robot_commander.hpp"
#include "std_msgs/msg/bool.hpp"

namespace kuka_sunrise_interface{

RobotCommander::RobotCommander(KUKA::FRI::LBRCommand& robot_command, const KUKA::FRI::LBRState& robot_state_, std::function<void(rclcpp::Time)> command_ready_callback, rclcpp::Node::SharedPtr robot_control_node):
    robot_command_(robot_command),
    robot_state_(robot_state_),
    torque_command_mode_(false),
    robot_control_node_(robot_control_node),
    ros_clock_(RCL_ROS_TIME),
    commandReadyCallback_(command_ready_callback)
{
  auto qos = rclcpp::QoS(rclcpp::KeepLast(1));
  qos.best_effort();
  auto callback = [this](sensor_msgs::msg::JointState::ConstSharedPtr msg)->void{this->commandReceivedCallback(msg);};
  auto msg_strategy = std::make_shared<MessageMemoryStrategy<sensor_msgs::msg::JointState>>();//TODO use TLSFAllocator? implement static strategy for jointstatemsg?
  joint_command_subscription_ =
        robot_control_node->create_subscription<sensor_msgs::msg::JointState>("lbr_joint_command", qos,
                                                                              callback,
                                                                              rclcpp::SubscriptionOptions(), msg_strategy);
}

void RobotCommander::commandReceivedCallback(sensor_msgs::msg::JointState::ConstSharedPtr msg){
  if(!isActive()){
    return;
  }
  if(torque_command_mode_){
    if(msg->effort.empty()){
      //raise some error/warning
      return;
    }
    const double* joint_torques_ = msg->position.data();
    robot_command_.setJointPosition(robot_state_.getIpoJointPosition());
    robot_command_.setTorque(joint_torques_);
  } else {
    if(msg->position.empty()){
      //raise some error/warning
      return;
    }
    const double* joint_positions_ = msg->position.data();
    robot_command_.setJointPosition(joint_positions_);
  }
  commandReadyCallback_(msg->header.stamp);

}









}


