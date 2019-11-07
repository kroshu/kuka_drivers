/*
 * robot_observer.hpp
 *
 *  Created on: Nov 5, 2019
 *      Author: rosdeveloper
 */

#ifndef INCLUDE_KUKA_SUNRISE_INTERFACE_ROBOT_OBSERVER_HPP_
#define INCLUDE_KUKA_SUNRISE_INTERFACE_ROBOT_OBSERVER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int64.hpp"
#include "std_msgs/msg/float64.hpp"

#include "fri_client/friLBRClient.h"
#include <list>

namespace kuka_sunrise_interface{

//TODO: use message types with headers instead?
class InputPublisherBase{
public:
  InputPublisherBase(std::string name, const KUKA::FRI::LBRState& robot_state, rclcpp::Node::SharedPtr robot_control_node);
  virtual void publishInputValue(const KUKA::FRI::LBRState& robot_state) = 0;
  virtual ~InputPublisherBase();
protected:
  virtual void createPublisher() = 0;
  std::string name_;
  const KUKA::FRI::LBRState& robot_state_;
  rclcpp::Node::SharedPtr robot_control_node_;
};

class BooleanInputPublisher: public InputPublisherBase{
public:
  using InputPublisherBase::InputPublisherBase;
  virtual void publishInputValue();
  virtual void createPublisher();
  //virtual ~BooleanInputPublisher();
private:
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisher_;
};

class DigitalInputPublisher: public InputPublisherBase{
public:
  using InputPublisherBase::InputPublisherBase;
  virtual void publishInputValue();
  virtual void createPublisher();
  //virtual ~DigitalInputPublisher();
private:
  rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr publisher_;
};

class AnalogInputPublisher: public InputPublisherBase{
public:
  using InputPublisherBase::InputPublisherBase;
  virtual void publishInputValue();
  virtual void createPublisher();
  //virtual ~AnalogInputPublisher();
private:
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_;
};

class RobotObserver{
  RobotObserver(const KUKA::FRI::LBRState& robot_state, rclcpp::Node::SharedPtr robot_control_node);
  void publishRobotState();

private:
  const KUKA::FRI::LBRState& robot_state_;
  sensor_msgs::msg::JointState joint_state_msg_;

  rclcpp::Node::SharedPtr robot_control_node_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;
  std::list<std::unique_ptr<InputPublisherBase>> input_publishers_;

  rclcpp::Clock ros_clock_;
};




}



#endif /* INCLUDE_KUKA_SUNRISE_INTERFACE_ROBOT_OBSERVER_HPP_ */
