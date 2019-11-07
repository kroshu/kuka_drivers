/*
 * robot_observer.cpp
 *
 *  Created on: Nov 5, 2019
 *      Author: rosdeveloper
 */

#include "kuka_sunrise_interface/robot_observer.hpp"

#include "rclcpp/time.hpp"

namespace kuka_sunrise_interface{

InputPublisherBase::InputPublisherBase(std::string name, const KUKA::FRI::LBRState& robot_state, rclcpp::Node::SharedPtr robot_control_node):
    name_(name),
    robot_state_(robot_state),
    robot_control_node_(robot_control_node)
{
}

void BooleanInputPublisher::createPublisher(){
  auto qos = rclcpp::QoS(rclcpp::KeepLast(1));
  qos.best_effort();
  publisher_ = robot_control_node_->create_publisher<std_msgs::msg::Bool>(name_, qos);
}

void BooleanInputPublisher::publishInputValue(){
  std_msgs::msg::Bool msg;
  msg.data = robot_state_.getBooleanIOValue(name_.c_str()); //TODO: catch exception
  publisher_->publish(msg);
}

void DigitalInputPublisher::createPublisher(){
  auto qos = rclcpp::QoS(rclcpp::KeepLast(1));
  qos.best_effort();
  publisher_ = robot_control_node_->create_publisher<std_msgs::msg::Int64>(name_, qos);
}

void DigitalInputPublisher::publishInputValue(){
  std_msgs::msg::Int64 msg;
  msg.data = robot_state_.getDigitalIOValue(name_.c_str()); //TODO: catch exception
  publisher_->publish(msg);
}

void AnalogInputPublisher::createPublisher(){
  auto qos = rclcpp::QoS(rclcpp::KeepLast(1));
  qos.best_effort();
  publisher_ = robot_control_node_->create_publisher<std_msgs::msg::Float64>(name_, qos);
}

void AnalogInputPublisher::publishInputValue(){
  std_msgs::msg::Float64 msg;
  msg.data = robot_state_.getAnalogIOValue(name_.c_str()); //TODO: catch exception
  publisher_->publish(msg);
}

RobotObserver::RobotObserver(const KUKA::FRI::LBRState& robot_state, rclcpp::Node::SharedPtr robot_control_node):
    robot_state_(robot_state),
    robot_control_node_(robot_control_node),
    ros_clock_(RCL_ROS_TIME)
{
  joint_state_msg_.position.reserve(robot_state_.NUMBER_OF_JOINTS);
  joint_state_msg_.velocity.reserve(robot_state_.NUMBER_OF_JOINTS);
  joint_state_msg_.effort.reserve(robot_state_.NUMBER_OF_JOINTS);
  auto qos = rclcpp::QoS(rclcpp::KeepLast(1));
  qos.best_effort();
  robot_control_node->create_publisher<sensor_msgs::msg::JointState>("lbr_joint_state", qos);
}

void RobotObserver::publishRobotState(){
  const double* joint_positions = robot_state_.getMeasuredJointPosition();
  const double* joint_torques = robot_state_.getExternalTorque(); //TODO: external vs measured?

  joint_state_msg_.header.frame_id = "world";
  joint_state_msg_.header.stamp = ros_clock_.now();//TODO catch exceptions

  joint_state_msg_.velocity.clear();
  joint_state_msg_.position.assign(joint_positions, joint_positions + robot_state_.NUMBER_OF_JOINTS);
  joint_state_msg_.effort.assign(joint_positions, joint_torques + robot_state_.NUMBER_OF_JOINTS);
  joint_state_publisher_->publish(joint_state_msg_);
  //TODO double check this:
  for(auto i = std::next(input_publishers_.begin()); i != input_publishers_.end(); i++){
    (*i)->publishInputValue(robot_state_);
  }
}




}
