/*
 * robot_observer.cpp
 *
 *  Created on: Nov 5, 2019
 *      Author: rosdeveloper
 */

#include "kuka_sunrise/robot_observer.hpp"

#include "rclcpp/time.hpp"

namespace kuka_sunrise
{

void RobotObserver::addBooleanInputObserver(std::string name)
{
  if (robot_control_node_->get_current_state().label() != "unconfigured")
  {
    return; //TODO handle other states
  }
  auto input_getter_func = [this](std::string name) -> bool
  {
    return this->robot_state_.getBooleanIOValue(name.c_str());
  };
  input_publishers_.emplace_back(
      std::make_unique<InputPublisher<bool, std_msgs::msg::Bool>>(name, input_getter_func, robot_control_node_));
}

void RobotObserver::addDigitalInputObserver(std::string name)
{
  if (robot_control_node_->get_current_state().label() != "unconfigured")
  {
    return; //TODO handle other states
  }
  auto input_getter_func = [this](std::string name) -> unsigned long long
  {
    return this->robot_state_.getDigitalIOValue(name.c_str());
  };
  input_publishers_.emplace_back(
      std::make_unique<InputPublisher<unsigned long long, std_msgs::msg::UInt64>>(name, input_getter_func,
                                                                                  robot_control_node_));
}

void RobotObserver::addAnalogInputObserver(std::string name)
{
  if (robot_control_node_->get_current_state().label() != "unconfigured")
  {
    return; //TODO handle other states
  }
  auto input_getter_func = [this](std::string name) -> double
  {
    return this->robot_state_.getDigitalIOValue(name.c_str());
  };
  input_publishers_.emplace_back(
      std::make_unique<InputPublisher<double, std_msgs::msg::Float64>>(name, input_getter_func, robot_control_node_));
}

bool RobotObserver::activate()
{
  this->ActivatableInterface::activate();
  joint_state_publisher_->on_activate();
  tracking_performance_publisher_->on_activate();
  return true;
}

bool RobotObserver::deactivate()
{
  this->ActivatableInterface::deactivate();
  joint_state_publisher_->on_deactivate();
  tracking_performance_publisher_->on_deactivate();
  return true;
}

RobotObserver::RobotObserver(const KUKA::FRI::LBRState &robot_state,
                             rclcpp_lifecycle::LifecycleNode::SharedPtr robot_control_node) :
    robot_state_(robot_state), robot_control_node_(robot_control_node)
{
  joint_state_msg_.position.reserve(robot_state_.NUMBER_OF_JOINTS);
  joint_state_msg_.velocity.reserve(robot_state_.NUMBER_OF_JOINTS);
  joint_state_msg_.effort.reserve(robot_state_.NUMBER_OF_JOINTS);
  auto qos = rclcpp::QoS(rclcpp::KeepLast(1));
  qos.best_effort();
  joint_state_publisher_ = robot_control_node->create_publisher<sensor_msgs::msg::JointState>("lbr_joint_state", qos);
  joint_state_publisher2_ = robot_control_node->create_publisher<sensor_msgs::msg::JointState>("lbr_joint_state2", qos);
  tracking_performance_publisher_ = robot_control_node->create_publisher<std_msgs::msg::Float64>("tracking_performance", qos);
  //joint_state_publisher_ = robot_control_node->create_publisher<sensor_msgs::msg::JointState>("lbr_joint_state", qos);
}

void RobotObserver::publishRobotState(const rclcpp::Time &stamp)
{
  if (robot_control_node_->get_current_state().label() != "inactive"
      && robot_control_node_->get_current_state().label() != "active")
  {
    return; //TODO handle other states
  }

  joint_state_msg_.header.frame_id = "world";
  joint_state_msg_.header.stamp = stamp; //TODO catch exceptions

  /*const double *joint_positions_measured = robot_state_.getMeasuredJointPosition();
  const double *joint_torques_measured = robot_state_.getMeasuredTorque();

  joint_state_msg_.velocity.clear();
  joint_state_msg_.position.assign(joint_positions_measured, joint_positions_measured + robot_state_.NUMBER_OF_JOINTS);
  joint_state_msg_.effort.assign(joint_torques_measured, joint_torques_measured + robot_state_.NUMBER_OF_JOINTS);
  joint_state_publisher2_->publish(joint_state_msg_);*/
  //RCLCPP_INFO(robot_control_node_->get_logger(), "%u", robot_state_.getSessionState());
  if (robot_state_.getSessionState() == KUKA::FRI::COMMANDING_WAIT
      || robot_state_.getSessionState() == KUKA::FRI::COMMANDING_ACTIVE)
  {
    const double *joint_positions_ipo = robot_state_.getMeasuredJointPosition();
    const double *joint_torques_external = robot_state_.getExternalTorque(); //TODO: external vs measured?
    joint_state_msg_.velocity.clear();
    joint_state_msg_.position.assign(joint_positions_ipo, joint_positions_ipo + robot_state_.NUMBER_OF_JOINTS);
    joint_state_msg_.effort.assign(joint_torques_external, joint_torques_external + robot_state_.NUMBER_OF_JOINTS);
    //RCLCPP_INFO(robot_control_node_->get_logger(), "joint msg updated");
    joint_state_publisher_->publish(joint_state_msg_);
    //RCLCPP_INFO(robot_control_node_->get_logger(), "joint msg sent");
    const double &tracking_performance = robot_state_.getTrackingPerformance();
    std_msgs::msg::Float64 tracking_perf_msg;
    tracking_perf_msg.data = tracking_performance;
    tracking_performance_publisher_->publish(tracking_perf_msg);
  }
  //TODO double check this:
  for (auto i = std::next(input_publishers_.begin()); i != input_publishers_.end(); i++)
  {
    (*i)->publishInputValue();
  }
}

}
