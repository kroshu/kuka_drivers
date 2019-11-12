/*
 * robot_control_node.cpp
 *
 *  Created on: Nov 11, 2019
 *      Author: rosdeveloper
 */


#include "kuka_sunrise_interface/robot_control_node.hpp"

namespace kuka_sunrise_interface{

RobotControlNode::RobotControlNode():
    LifecycleNode("robot_control"),
    client_(rclcpp_lifecycle::LifecycleNode::shared_from_this()),
    client_application_(udp_connection_, client_),
    client_application_thread_(0)
{

}

void RobotControlNode::runClientApplication(){
  client_application_.connect(30200, NULL);
  bool success = true;
  while(success){
    success = client_application_.step();

    if(client_.robotState().getSessionState() == KUKA::FRI::IDLE){
      break;
    }
  }
  if(!success){
    //TODO handle
  }
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
RobotControlNode::on_configure(const rclcpp_lifecycle::State& state){
  (void)state;
  auto SUCCESS =  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  auto ERROR = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  //TODO change stack size with setrlimit rlimit_stack?
  if(mlockall(MCL_CURRENT|MCL_FUTURE) == -1){
    return ERROR;
  }

  struct sched_param param;
  param.sched_priority = 90;
  if(sched_setscheduler(0, SCHED_FIFO, &param) == -1){
    return ERROR;
  }

  void*(*run_app)(void*) = [](void* robot_control_node)->void*{
    static_cast<RobotControlNode*>(robot_control_node)->runClientApplication();
    return NULL;
  };

  if(pthread_create(&client_application_thread_, NULL, run_app, static_cast<void*>(this))){
    return ERROR;
  }

  return SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
RobotControlNode::on_cleanup(const rclcpp_lifecycle::State& state){
  (void)state;
  auto SUCCESS =  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  //auto ERROR = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  pthread_join(client_application_thread_, NULL);//TODO close client separately here?
  return SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
RobotControlNode::on_shutdown(const rclcpp_lifecycle::State& state){
  (void)state;
  auto SUCCESS =  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  //auto ERROR = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  pthread_join(client_application_thread_, NULL);//TODO close client separately here?
  return SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
RobotControlNode::on_activate(const rclcpp_lifecycle::State& state){
  (void)state;
  auto SUCCESS =  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  auto ERROR = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;

  if(client_.activateControl()){
    return SUCCESS;
  } else {
    return ERROR;
  }
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
RobotControlNode::on_deactivate(const rclcpp_lifecycle::State& state){
  (void)state;
  auto SUCCESS =  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  auto ERROR = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;

  if(client_.deactivateControl()){
    return SUCCESS;
  } else {
    return ERROR;
  }
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
RobotControlNode::on_error(const rclcpp_lifecycle::State& state){
  return this->on_cleanup(state);
}

}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<kuka_sunrise_interface::RobotControlNode>()->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
