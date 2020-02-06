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

#ifndef INCLUDE_KUKA_SUNRISE_ROBOT_OBSERVER_HPP_
#define INCLUDE_KUKA_SUNRISE_ROBOT_OBSERVER_HPP_

#include <fri_client/friLBRClient.h>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node_impl.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/u_int64.hpp"
#include "std_msgs/msg/float64.hpp"

#include <list>

#include "kuka_sunrise/internal/activatable_interface.hpp"

namespace kuka_sunrise
{

//TODO: use message types with headers instead?
class InputPublisherBase
{
public:
  virtual void publishInputValue() = 0;
  virtual ~InputPublisherBase()
  {
  }

};

template<typename FRIType, typename ROSType>
class InputPublisher : public InputPublisherBase, public ActivatableInterface
{
public:
  InputPublisher(std::string name, std::function<FRIType(std::string)> input_getter_func,
                 rclcpp_lifecycle::LifecycleNode::SharedPtr robot_control_node) :
      name_(name), getInput_(input_getter_func)
  {
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1));
    qos.best_effort();
    publisher_ = robot_control_node->create_publisher<ROSType>(name_, qos);
  }
  virtual void publishInputValue()
  {
    ROSType msg;
    msg.data = getInput_(name_);
    publisher_->publish(msg);
  }

private:
  std::string name_;
  std::function<FRIType(std::string)> getInput_;
  typename rclcpp::Publisher<ROSType>::SharedPtr publisher_;
};

class RobotObserver : public ActivatableInterface
{
public:
  RobotObserver(const KUKA::FRI::LBRState &robot_state, rclcpp_lifecycle::LifecycleNode::SharedPtr robot_control_node);
  void addBooleanInputObserver(std::string name);
  void addDigitalInputObserver(std::string name);
  void addAnalogInputObserver(std::string name);
  void publishRobotState(const rclcpp::Time &stamp);
  virtual bool activate();
  virtual bool deactivate();

private:
  const KUKA::FRI::LBRState &robot_state_;
  sensor_msgs::msg::JointState joint_state_msg_;

  rclcpp_lifecycle::LifecycleNode::SharedPtr robot_control_node_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher2_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64>::SharedPtr tracking_performance_publisher_;
  std::list<std::unique_ptr<InputPublisherBase>> input_publishers_;

  int i = 0;
};

}

#endif /* INCLUDE_KUKA_SUNRISE_ROBOT_OBSERVER_HPP_ */
