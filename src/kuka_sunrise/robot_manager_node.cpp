/*
 * robot_manager_node.cpp
 *
 *  Created on: Nov 12, 2019
 *      Author: rosdeveloper
 */

#include "kuka_sunrise/robot_manager_node.hpp"
#include "kuka_sunrise/internal/service_tools.hpp"

#include "std_msgs/msg/bool.hpp"

namespace kuka_sunrise
{

RobotManagerNode::RobotManagerNode() :
    LifecycleNode("robot_manager")
{
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  robot_manager_ = std::make_shared<RobotManager>([this]
  { this->handleControlEndedError();}, [this]
  { this->handleFRIEndedError();});
  auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
  qos.reliable();
  cbg_ = this->create_callback_group(rclcpp::callback_group::CallbackGroupType::MutuallyExclusive);
  change_robot_control_state_client_ = this->create_client<lifecycle_msgs::srv::ChangeState>(
      "robot_control/change_state", qos.get_rmw_qos_profile(), cbg_);
  set_command_state_client_ = this->create_client<std_srvs::srv::SetBool>("set_command_state_robot_control_",
                                                                          qos.get_rmw_qos_profile(), cbg_);
  auto command_srv_callback = [this](const std::shared_ptr<rmw_request_id_t> request_header,
                                     std_srvs::srv::SetBool::Request::SharedPtr request,
                                     std_srvs::srv::SetBool::Response::SharedPtr response)
                                     {
                                       (void)request_header;
                                       if(request->data == true)
                                       {
                                         response->success = this->activate();
                                       }
                                       else
                                       {
                                         response->success = this->deactivate();
                                       }
                                     };
  set_command_state_service_ = this->create_service<std_srvs::srv::SetBool>("set_commanding_state",
                                                                            command_srv_callback);
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn RobotManagerNode::on_configure(
    const rclcpp_lifecycle::State &state)
{
  (void)state;
  configuration_manager_ = std::make_unique<ConfigurationManager>(this->shared_from_this(), robot_manager_);
  if (!robot_manager_->isConnected())
  {
    if (!robot_manager_->connect("192.168.37.29", 30000))
    { //TODO use ros params
      RCLCPP_ERROR(get_logger(), "could not connect");
      return FAILURE;
    }
  }
  //TODO get IO configuration

  if (!requestRobotControlNodeStateTransition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE))
  {
    RCLCPP_ERROR(get_logger(), "could not configure robot control node");
    return FAILURE;
  }

  return SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn RobotManagerNode::on_cleanup(
    const rclcpp_lifecycle::State &state)
{
  (void)state;


  if (!robot_manager_->disconnect())
  { //TODO use ros params
    RCLCPP_ERROR(get_logger(), "could not disconnect");
    return FAILURE;
  }


  return SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn RobotManagerNode::on_shutdown(
    const rclcpp_lifecycle::State &state)
{
  switch (state.id())
  {
    case lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE:
      if (this->on_deactivate(get_current_state()) != SUCCESS)
      {
        break;
      }
      this->on_cleanup(get_current_state());
      break;
    case lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE:
      this->on_cleanup(get_current_state());
      break;
    case lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED:
      break;
    default:
      return SUCCESS;
  }

  if (!requestRobotControlNodeStateTransition(lifecycle_msgs::msg::Transition::TRANSITION_UNCONFIGURED_SHUTDOWN))
  {
    RCLCPP_ERROR(get_logger(), "could not shut down control");
    return ERROR;
  }

  return SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn RobotManagerNode::on_activate(
    const rclcpp_lifecycle::State &state)
{
  (void)state;

  if (!robot_manager_->isConnected())
  {
    RCLCPP_ERROR(get_logger(), "not connected");
    return ERROR;
  }

  if (!this->has_parameter("send_period_ms") || !this->has_parameter("receive_multiplier"))
  {
    RCLCPP_ERROR(get_logger(), "Parameter send_period_ms or receive_multiplier not available");
    return FAILURE;
  }
  rclcpp::Parameter send_period_ms = this->get_parameter("send_period_ms");
  rclcpp::Parameter receive_multiplier = this->get_parameter("receive_multiplier");

  if (!robot_manager_->setFRIConfig(30200, send_period_ms.as_int(), receive_multiplier.as_int()))
  {
    RCLCPP_ERROR(get_logger(), "could not set fri config");
    return FAILURE;
  }


  if (!requestRobotControlNodeStateTransition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE))
  {
    RCLCPP_ERROR(get_logger(), "could not activate");
    return FAILURE;
  }

  if (!robot_manager_->startFRI())
  {
    RCLCPP_ERROR(get_logger(), "could not start fri");
    return FAILURE;
  }

  return SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn RobotManagerNode::on_deactivate(
    const rclcpp_lifecycle::State &state)
{
  (void)state;
  if (!robot_manager_->isConnected())
  {
    RCLCPP_ERROR(get_logger(), "not connected");
    return ERROR;
  }

  if(this->isActive() && !this->deactivate()){
    RCLCPP_ERROR(get_logger(), "could not deactivate control");
    return FAILURE;
  }

  if (!robot_manager_->endFRI())
  {
    RCLCPP_ERROR(get_logger(), "could not end fri");
    return FAILURE;
  }

  if (!requestRobotControlNodeStateTransition(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE))
  {
    //TODO out of sync
    RCLCPP_ERROR(get_logger(), "could not deactivate");
    return ERROR;
  }

  return SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn RobotManagerNode::on_error(
    const rclcpp_lifecycle::State &state)
{
  (void)state;
  RCLCPP_ERROR(get_logger(), "An error occured");
  return SUCCESS;
}

bool RobotManagerNode::activate()
{
  if (!robot_manager_->isConnected())
  {
    RCLCPP_ERROR(get_logger(), "not connected");
    return false;
  }

  if(!setRobotControlNodeCommandState(true)){
    RCLCPP_ERROR(get_logger(), "could not set command state");
    return false;
  }

  if (!robot_manager_->activateControl())
  {
    //TODO check robot control node state first
    setRobotControlNodeCommandState(false);
    RCLCPP_ERROR(get_logger(), "could not activate control");
    return false;
  }

  return true;
}

bool RobotManagerNode::deactivate()
{
  if (!robot_manager_->isConnected())
  {
    RCLCPP_ERROR(get_logger(), "not connected");
    return false;
  }

  if (!robot_manager_->deactivateControl())
  {
    RCLCPP_ERROR(get_logger(), "could not deactivate control");
    return false;
  }

  if(!setRobotControlNodeCommandState(false)){
    //TODO out of sync
    RCLCPP_ERROR(get_logger(), "could not set command state");
    return false;
  }

  return true;
}

bool RobotManagerNode::requestRobotControlNodeStateTransition(std::uint8_t transition)
{
  auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
  request->transition.id = transition;
  if (!change_robot_control_state_client_->wait_for_service(std::chrono::milliseconds(2000)))
  {
    RCLCPP_ERROR(get_logger(), "Wait for service failed");
    return false;
  }
  auto future_result = change_robot_control_state_client_->async_send_request(request);
  auto future_status = wait_for_result(future_result, std::chrono::milliseconds(3000));
  if (future_status != std::future_status::ready)
  {
    RCLCPP_ERROR(get_logger(), "Future status not ready");
    return false;
  }
  if (future_result.get()->success)
  {
    return true;
  }
  else
  {
    RCLCPP_ERROR(get_logger(), "Future result not success");
    return false;
  }

}

bool RobotManagerNode::setRobotControlNodeCommandState(bool active)
{
  auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
  request->data = active;

  auto future_result = set_command_state_client_->async_send_request(request);
  auto future_status = wait_for_result(future_result, std::chrono::milliseconds(3000));
  if (future_status != std::future_status::ready)
  {
    RCLCPP_ERROR(get_logger(), "Future status not ready");
    return false;
  }

  if (!future_result.get()->success)
  {
    RCLCPP_ERROR(get_logger(), "Future result not success");
    return false;
  }

  return true;
}

void RobotManagerNode::handleControlEndedError()
{
  RCLCPP_INFO(get_logger(), "control ended");
  deactivate();
}

void RobotManagerNode::handleFRIEndedError()
{
  RCLCPP_INFO(get_logger(), "FRI ended");
  if (get_current_state().label() == "active")
  {
    deactivate();
  }
  cleanup();
}

}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor;
  auto node = std::make_shared<kuka_sunrise::RobotManagerNode>();
  executor.add_node(node->get_node_base_interface());
  executor.spin();
  rclcpp::shutdown();
  return 0;
}

