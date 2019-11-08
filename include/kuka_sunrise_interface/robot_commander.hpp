/*
 * robot_commander.hpp
 *
 *  Created on: Nov 5, 2019
 *      Author: rosdeveloper
 */

#ifndef INCLUDE_KUKA_SUNRISE_INTERFACE_ROBOT_COMMANDER_HPP_
#define INCLUDE_KUKA_SUNRISE_INTERFACE_ROBOT_COMMANDER_HPP_

#include "fri_client/friLBRClient.h"
#include "kuka_sunrise_interface/internal/activatable_interface.hpp"

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include <rclcpp/message_memory_strategy.hpp>
#include <rclcpp/strategies/message_pool_memory_strategy.hpp>
#include <rclcpp/strategies/allocator_memory_strategy.hpp>

#include <functional>

namespace kuka_sunrise_interface{

using rclcpp::message_memory_strategy::MessageMemoryStrategy;
using rclcpp::strategies::message_pool_memory_strategy::MessagePoolMemoryStrategy;
using rclcpp::memory_strategies::allocator_memory_strategy::AllocatorMemoryStrategy;

class OutputSubscriptionBase{
public:
  virtual ~OutputSubscriptionBase();
};

template <typename FRIType, typename ROSType>
class OutputSubscription : public OutputSubscriptionBase{
public:
  OutputSubscription(std::string name, std::function<void(std::string, FRIType)> output_setter_func, const bool& is_commanding_active_flag, rclcpp::Node::SharedPtr robot_control_node):
    name_(name),
    setOutput_(output_setter_func),
    is_commanding_active_(is_commanding_active_flag)
  {
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1));
    qos.best_effort();
    auto msg_strategy = std::make_shared<MessagePoolMemoryStrategy<ROSType, 1>>();
    subscription_ = robot_control_node->create_subscription(name_, qos,
                                                            [this](ROSType msg){this->commandReceivedCallback(msg);},
                                                            rclcpp::SubscriptionOptions(), msg_strategy);
  }
private:
  std::string name_;
  std::function<void(std::string name, FRIType value)> setOutput_;
  const bool& is_commanding_active_;
  typename rclcpp::Subscription<ROSType>::SharedPtr subscription_;

  void commandReceivedCallback(ROSType msg){
    if(!is_commanding_active_){
      return;
    }
    FRIType value = msg.data;
    setOutput_(name_, value);
  }
};

class RobotCommander: public ActivatableInterface{
public:
  RobotCommander(KUKA::FRI::LBRCommand& robot_command, const KUKA::FRI::LBRState& robot_state_, std::function<void(rclcpp::Time)> command_ready_callback, rclcpp::Node::SharedPtr robot_control_node);
  void addBooleanOutputCommander(std::string name);
  void addDigitalOutputCommander(std::string name);
  void addAnalogOutputCommander(std::string name);

private:
  KUKA::FRI::LBRCommand& robot_command_;
  const KUKA::FRI::LBRState& robot_state_;
  bool torque_command_mode_;//TODO use atomic instead?
  sensor_msgs::msg::JointState joint_command_msg_;

  rclcpp::Node::SharedPtr robot_control_node_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_command_subscription_;
  std::list<std::unique_ptr<OutputSubscriptionBase>> output_subsciptions_;

  rclcpp::Clock ros_clock_;

  std::function<void(rclcpp::Time)> commandReadyCallback_;
  void commandReceivedCallback(sensor_msgs::msg::JointState::ConstSharedPtr msg);



};


}




#endif /* INCLUDE_KUKA_SUNRISE_INTERFACE_ROBOT_COMMANDER_HPP_ */