/*
 * robot_commander.cpp
 *
 *  Created on: Nov 5, 2019
 *      Author: rosdeveloper
 */

#include "kuka_sunrise_interface/robot_commander.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/u_int64.hpp"
#include "std_msgs/msg/float64.hpp"


namespace kuka_sunrise_interface{

RobotCommander::RobotCommander(KUKA::FRI::LBRCommand& robot_command, const KUKA::FRI::LBRState& robot_state_, rclcpp_lifecycle::LifecycleNode::SharedPtr robot_control_node):
    robot_command_(robot_command),
    robot_state_(robot_state_),
    torque_command_mode_(false),
    robot_control_node_(robot_control_node),
    ros_clock_(RCL_ROS_TIME)
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

void RobotCommander::addBooleanOutputCommander(const std::string& name){
  if(robot_control_node_->get_current_state().label() != "unconfigured"){
    return; //TODO handle other states
  }
  auto output_setter_func = [this](std::string name, bool value)->void{
    return this->robot_command_.setBooleanIOValue(name.c_str(), value);
  };
  output_subsciptions_.emplace_back(std::make_unique<OutputSubscription<bool, std_msgs::msg::Bool>>
                                    (name, output_setter_func, is_active_, robot_control_node_));
}

void RobotCommander::addDigitalOutputCommander(const std::string& name){
  if(robot_control_node_->get_current_state().label() != "unconfigured"){
    return; //TODO handle other states
  }
  auto output_setter_func = [this](std::string name, unsigned long long value)->void{
    return this->robot_command_.setDigitalIOValue(name.c_str(), value);
  };
  output_subsciptions_.emplace_back(std::make_unique<OutputSubscription<unsigned long long, std_msgs::msg::UInt64>>
                                    (name, output_setter_func, is_active_, robot_control_node_));
}

void RobotCommander::addAnalogOutputCommander(const std::string& name){
  if(robot_control_node_->get_current_state().label() != "unconfigured"){
    return; //TODO handle other states
  }
  auto output_setter_func = [this](std::string name, double value)->void{
    return this->robot_command_.setAnalogIOValue(name.c_str(), value);
  };
  output_subsciptions_.emplace_back(std::make_unique<OutputSubscription<double, std_msgs::msg::Float64>>
                                    (name, output_setter_func, is_active_, robot_control_node_));
}

void RobotCommander::setTorqeCommanding(bool is_torque_mode_active){
  if(is_active_){
    return; //TODO handle
  }
  torque_command_mode_ = is_torque_mode_active;
}

void RobotCommander::updateCommand(const rclcpp::Time& stamp){
  std::unique_lock<std::mutex> lk(m_);
  lk.lock();
  if(joint_command_msg_->header.stamp != stamp){
    lk.unlock();
    cv_.wait(lk);
    lk.lock();
  }
  //check if wait has been interrupted by the robot manager
  if(!is_active_){
    lk.unlock();
    return;
  }

  if(torque_command_mode_){
    if(joint_command_msg_->effort.empty()){
      //raise some error/warning
      return;
    }
    const double* joint_torques_ = joint_command_msg_->position.data();
    robot_command_.setJointPosition(robot_state_.getIpoJointPosition());
    robot_command_.setTorque(joint_torques_);
  } else {
    if(joint_command_msg_->position.empty()){
      //raise some error/warning
      return;
    }
    const double* joint_positions_ = joint_command_msg_->position.data();
    robot_command_.setJointPosition(joint_positions_);
  }

  for(auto& output_subscription : output_subsciptions_){
    output_subscription->updateOutput();
  }

  lk.unlock();
}

void RobotCommander::commandReceivedCallback(sensor_msgs::msg::JointState::ConstSharedPtr msg){
  std::unique_lock<std::mutex> lk(m_);
  lk.lock();
  if(!is_active_){
    lk.unlock();
    return;
  }
  joint_command_msg_ = msg;
  lk.unlock();
  cv_.notify_one();
}

bool RobotCommander::deactivate(){
  std::unique_lock<std::mutex> lk(m_);
  lk.lock();
  is_active_ = false;
  lk.unlock();
  cv_.notify_one();//interrupt updateCommand()
  return true;
}







}


