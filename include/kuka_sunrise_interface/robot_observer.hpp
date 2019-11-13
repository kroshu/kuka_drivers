/*
 * robot_observer.hpp
 *
 *  Created on: Nov 5, 2019
 *      Author: rosdeveloper
 */

#ifndef INCLUDE_KUKA_SUNRISE_INTERFACE_ROBOT_OBSERVER_HPP_
#define INCLUDE_KUKA_SUNRISE_INTERFACE_ROBOT_OBSERVER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node_impl.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/u_int64.hpp"
#include "std_msgs/msg/float64.hpp"

#include "fri_client/friLBRClient.h"
#include <list>

namespace kuka_sunrise_interface{

//TODO: use message types with headers instead?
class InputPublisherBase{
public:
  //InputPublisherBase(std::string name);
  virtual void publishInputValue() = 0;
  virtual ~InputPublisherBase(){}
protected:
  //virtual void createPublisher() = 0;
  //std::string name_;
  //const KUKA::FRI::LBRState& robot_state_;
  //rclcpp::Node::SharedPtr robot_control_node_;
};

template <typename FRIType, typename ROSType>
class InputPublisher : public InputPublisherBase{
public:
  InputPublisher(std::string name, std::function<FRIType(std::string)> input_getter_func, rclcpp_lifecycle::LifecycleNode::SharedPtr robot_control_node):
    name_(name),
    getInput_(input_getter_func)
  {
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1));
    qos.best_effort();
    publisher_ = robot_control_node->create_publisher<ROSType>(name_, qos);
  }
  virtual void publishInputValue(){
    ROSType msg;
    msg.data = getInput_(name_);
    publisher_->publish(msg);
  }

private:
  std::string name_;
  std::function<FRIType(std::string)> getInput_;
  typename rclcpp::Publisher<ROSType>::SharedPtr publisher_;
};
/*
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
*/
class RobotObserver{
public:
  RobotObserver(const KUKA::FRI::LBRState& robot_state, rclcpp_lifecycle::LifecycleNode::SharedPtr robot_control_node);
  void addBooleanInputObserver(std::string name);
  void addDigitalInputObserver(std::string name);
  void addAnalogInputObserver(std::string name);
  void publishRobotState(const rclcpp::Time& stamp, bool ipo);

private:
  const KUKA::FRI::LBRState& robot_state_;
  sensor_msgs::msg::JointState joint_state_msg_;

  rclcpp_lifecycle::LifecycleNode::SharedPtr robot_control_node_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;
  std::list<std::unique_ptr<InputPublisherBase>> input_publishers_;
};




}



#endif /* INCLUDE_KUKA_SUNRISE_INTERFACE_ROBOT_OBSERVER_HPP_ */
