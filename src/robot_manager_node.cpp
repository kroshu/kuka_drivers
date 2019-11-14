/*
 * robot_manager_node.cpp
 *
 *  Created on: Nov 12, 2019
 *      Author: rosdeveloper
 */


#include "kuka_sunrise_interface/robot_manager_node.hpp"

namespace kuka_sunrise_interface{

RobotManagerNode::RobotManagerNode():
    LifecycleNode("robot_manager"),
    robot_manager_([this]{this->handleControlEndedError();}, [this]{this->handleFRIEndedError();})
{
  change_robot_control_state_client_ = this->create_client<lifecycle_msgs::srv::ChangeState>("robot_control_node/change_state");
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
RobotManagerNode::on_configure(const rclcpp_lifecycle::State& state){
  (void)state;
  if(!robot_manager_.isConnected()){
    if(!robot_manager_.connect("192.168.37.29", 30000)){ //TODO use ros params
      RCLCPP_ERROR(get_logger(), "could not connect");
      return ERROR;
    }
  }
  //TODO get IO configuration

  if(!requestRobotControlNodeStateTransition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE)){
    RCLCPP_ERROR(get_logger(), "could not configure robot control node");
    return ERROR;
  }

  if(!robot_manager_.startFRI()){
    RCLCPP_ERROR(get_logger(), "could not start fri");
    return ERROR;
  }

  return SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
RobotManagerNode::on_cleanup(const rclcpp_lifecycle::State& state){
  (void)state;
  if(!robot_manager_.isConnected()){
    RCLCPP_ERROR(get_logger(), "not connected");
    return ERROR;
  }

  if(!robot_manager_.endFRI()){
    RCLCPP_ERROR(get_logger(), "could not end fri");
    return ERROR;
  }

  if(!robot_manager_.disconnect()){ //TODO use ros params
    RCLCPP_ERROR(get_logger(), "could not disconnect");
    return ERROR;
  }

  if(!requestRobotControlNodeStateTransition(lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP)){
    RCLCPP_ERROR(get_logger(), "could not clean up control");
    return ERROR;
  }

  return SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
RobotManagerNode::on_shutdown(const rclcpp_lifecycle::State& state){
  lifecycle_msgs::msg::Transition::_id_type transition;
  switch(state.id()){
    case lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE:
      this->on_deactivate(state);
      this->on_cleanup(state);
      transition = lifecycle_msgs::msg::Transition::TRANSITION_ACTIVE_SHUTDOWN;
      break;
    case lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE:
      this->on_cleanup(state);
      transition = lifecycle_msgs::msg::Transition::TRANSITION_INACTIVE_SHUTDOWN;
      break;
    case lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED:
      transition = lifecycle_msgs::msg::Transition::TRANSITION_UNCONFIGURED_SHUTDOWN;
      break;
    default:
      return ERROR;
  }

  if(!requestRobotControlNodeStateTransition(transition)){
    RCLCPP_ERROR(get_logger(), "could not shut down control");
    return ERROR;
  }

  return SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
RobotManagerNode::on_activate(const rclcpp_lifecycle::State& state){
  (void)state;
  if(!robot_manager_.isConnected()){
    RCLCPP_ERROR(get_logger(), "not connected");
    return ERROR;
  }


  if(!requestRobotControlNodeStateTransition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE)){
    RCLCPP_ERROR(get_logger(), "could not activate control node");
    return ERROR;
  }

  if(!robot_manager_.activateControl()){
    //deactivate robot control node
    RCLCPP_ERROR(get_logger(), "could not deactivate control");
    return ERROR;
  }

  return SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
RobotManagerNode::on_deactivate(const rclcpp_lifecycle::State& state){
  (void)state;
  if(!robot_manager_.isConnected()){
    RCLCPP_ERROR(get_logger(), "not connected");
    return ERROR;
  }

  if(!robot_manager_.deactivateControl()){
    RCLCPP_ERROR(get_logger(), "could not deactivate control");
    return ERROR;
  }

  if(!requestRobotControlNodeStateTransition(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE)){
    RCLCPP_ERROR(get_logger(), "could not deactivate control node");
    return ERROR;
  }

  return SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
RobotManagerNode::on_error(const rclcpp_lifecycle::State& state){
  (void)state;
  return this->on_cleanup(state);
}

template<typename FutureT, typename WaitTimeT>
std::future_status
wait_for_result(
  FutureT & future,
  WaitTimeT time_to_wait)
{
  auto end = std::chrono::steady_clock::now() + time_to_wait;
  std::chrono::milliseconds wait_period(100);
  std::future_status status = std::future_status::timeout;
  do {
    auto now = std::chrono::steady_clock::now();
    auto time_left = end - now;
    if (time_left <= std::chrono::seconds(0)) {break;}
    status = future.wait_for((time_left < wait_period) ? time_left : wait_period);
  } while (rclcpp::ok() && status != std::future_status::ready);
  return status;
}

bool RobotManagerNode::requestRobotControlNodeStateTransition(std::uint8_t transition){
  return true;
  auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
  request->transition.id = transition;
  if(!change_robot_control_state_client_->wait_for_service(std::chrono::seconds(1))){
    return false;
  }
  auto future_result = change_robot_control_state_client_->async_send_request(request);
  auto future_status = wait_for_result(future_result, std::chrono::seconds(1));
  if(future_status != std::future_status::ready){
    return false;
  }
  if(future_result.get()->success){
    return true;
  } else {
    return false;
  }

}

void RobotManagerNode::handleControlEndedError(){
  RCLCPP_INFO(get_logger(), "control ended");
  deactivate();
}

void RobotManagerNode::handleFRIEndedError(){
  RCLCPP_INFO(get_logger(), "FRI ended");
  if(get_current_state().label() == "active"){
    deactivate();
  }
  cleanup();
}


}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<kuka_sunrise_interface::RobotManagerNode>()->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}

