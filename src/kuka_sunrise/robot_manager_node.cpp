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

#include <memory>

#include "std_msgs/msg/bool.hpp"

#include "kuka_sunrise/robot_manager_node.hpp"
#include "kuka_sunrise/internal/service_tools.hpp"


namespace kuka_sunrise
{

RobotManagerNode::RobotManagerNode()
: LifecycleNode("robot_manager")
{
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  robot_manager_ = std::make_shared<RobotManager>(
    [this]
    {this->handleControlEndedError();}, [this]
    {this->handleFRIEndedError();});
  auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
  qos.reliable();
  cbg_ = this->create_callback_group(rclcpp::callback_group::CallbackGroupType::MutuallyExclusive);
  change_robot_control_state_client_ = this->create_client<lifecycle_msgs::srv::ChangeState>(
    "robot_control/change_state", qos.get_rmw_qos_profile(), cbg_);
  set_command_state_client_ = this->create_client<std_srvs::srv::SetBool>(
    "robot_control/set_commanding_state", qos.get_rmw_qos_profile(), cbg_);
  auto command_srv_callback = [this](
    std_srvs::srv::SetBool::Request::SharedPtr request,
    std_srvs::srv::SetBool::Response::SharedPtr response) {
      if (request->data == true) {
        response->success = this->activate();
      } else {
        response->success = this->deactivate();
      }
    };
  change_robot_manager_state_service_ = this->create_service<std_srvs::srv::SetBool>(
    "robot_manager/set_commanding_state", command_srv_callback);
  command_state_changed_publisher_ = this->create_publisher<std_msgs::msg::Bool>(
    "robot_manager/commanding_state_changed", qos);
  set_parameter_client_ = this->create_client<std_srvs::srv::Trigger>(
    "configuration_manager/set_params", ::rmw_qos_profile_default, cbg_);
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
RobotManagerNode::on_configure(const rclcpp_lifecycle::State &)
{
  if (!requestRobotControlNodeStateTransition(
      lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE))
  {
    RCLCPP_ERROR(get_logger(), "could not configure robot control node");
    return FAILURE;
  }

  configuration_manager_ = std::make_unique<ConfigurationManager>(
    this->shared_from_this(),
    robot_manager_);

  if (!this->has_parameter("controller_ip")) {
    RCLCPP_ERROR(get_logger(), "Parameter controller_ip not available");
    if (!requestRobotControlNodeStateTransition(
        lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP))
    {
      RCLCPP_ERROR(get_logger(), "Could not solve differing states, restart needed");
    }
    return FAILURE;
  }

  const char * controller_ip = this->get_parameter("controller_ip").as_string().c_str();
  if (!robot_manager_->isConnected()) {
    if (!robot_manager_->connect(controller_ip, 30000)) {
      RCLCPP_ERROR(get_logger(), "could not connect");
      if (!requestRobotControlNodeStateTransition(
          lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP))
      {
        RCLCPP_ERROR(get_logger(), "Could not solve differing states, restart needed");
      }
      return FAILURE;
    }
  }
  // TODO(resizoltan) get IO configuration

  auto trigger_request =
    std::make_shared<std_srvs::srv::Trigger::Request>();
  auto future_result = set_parameter_client_->async_send_request(trigger_request);
  auto future_status = kuka_sunrise::wait_for_result(
    future_result,
    std::chrono::milliseconds(3000));
  if (future_status != std::future_status::ready) {
    RCLCPP_ERROR(get_logger(), "Future status not ready, could not set parameters");
    return FAILURE;
  }
  if (!future_result.get()->success) {
    RCLCPP_ERROR(get_logger(), "Future result not success, could not set parameters");
    return FAILURE;
  }

  return SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
RobotManagerNode::on_cleanup(const rclcpp_lifecycle::State &)
{
  if (!requestRobotControlNodeStateTransition(
      lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP))
  {
    RCLCPP_ERROR(get_logger(), "could not clean up robot control node");
    return FAILURE;
  }

  if (!robot_manager_->disconnect()) {
    RCLCPP_ERROR(get_logger(), "could not disconnect");
    return FAILURE;
  }

  return SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
RobotManagerNode::on_shutdown(const rclcpp_lifecycle::State & state)
{
  switch (state.id()) {
    case lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE:
      if (this->on_deactivate(get_current_state()) != SUCCESS) {
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

  if (!requestRobotControlNodeStateTransition(
      lifecycle_msgs::msg::Transition::TRANSITION_UNCONFIGURED_SHUTDOWN))
  {
    RCLCPP_ERROR(get_logger(), "could not shut down control");
    return ERROR;
  }

  return SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
RobotManagerNode::on_activate(const rclcpp_lifecycle::State &)
{
  if (!robot_manager_->isConnected()) {
    RCLCPP_ERROR(get_logger(), "not connected");
    return ERROR;
  }

  if (!this->has_parameter("send_period_ms") || !this->has_parameter("receive_multiplier")) {
    RCLCPP_ERROR(get_logger(), "Parameter send_period_ms or receive_multiplier not available");
    return FAILURE;
  }
  rclcpp::Parameter send_period_ms = this->get_parameter("send_period_ms");
  rclcpp::Parameter receive_multiplier = this->get_parameter("receive_multiplier");

  if (!robot_manager_->setFRIConfig(30200, send_period_ms.as_int(), receive_multiplier.as_int())) {
    RCLCPP_ERROR(get_logger(), "could not set fri config");
    return FAILURE;
  }

  if (!requestRobotControlNodeStateTransition(
      lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE))
  {
    RCLCPP_ERROR(get_logger(), "could not activate");
    return FAILURE;
  }

  if (!robot_manager_->startFRI()) {
    RCLCPP_ERROR(get_logger(), "could not start fri");
    if (!requestRobotControlNodeStateTransition(
        lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE))
    {
      RCLCPP_ERROR(
        get_logger(),
        "Could not solve differing states, restart needed");
    }
    return FAILURE;
  }

  command_state_changed_publisher_->on_activate();

  return SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
RobotManagerNode::on_deactivate(const rclcpp_lifecycle::State &)
{
  if (!robot_manager_->isConnected()) {
    RCLCPP_ERROR(get_logger(), "not connected");
    return ERROR;
  }

  if (this->isActive() && !this->deactivate()) {
    RCLCPP_ERROR(get_logger(), "could not deactivate control");
    return FAILURE;
  }

  if (!robot_manager_->endFRI()) {
    RCLCPP_ERROR(get_logger(), "could not end fri");
    return FAILURE;
  }

  if (!requestRobotControlNodeStateTransition(
      lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE))
  {
    // TODO(resizoltan) out of sync
    RCLCPP_ERROR(get_logger(), "could not deactivate");
    return ERROR;
  }

  command_state_changed_publisher_->on_deactivate();

  return SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
RobotManagerNode::on_error(const rclcpp_lifecycle::State &)
{
  RCLCPP_ERROR(get_logger(), "An error occured");
  return SUCCESS;
}

bool RobotManagerNode::activate()
{
  this->ActivatableInterface::activate();
  if (!robot_manager_->isConnected()) {
    RCLCPP_ERROR(get_logger(), "not connected");
    return false;
  }

  if (!setRobotControlNodeCommandState(true)) {
    RCLCPP_ERROR(get_logger(), "could not set command state");
    return false;
  }

  if (!robot_manager_->activateControl()) {
    // TODO(resizoltan) check robot control node state first
    this->ActivatableInterface::deactivate();
    RCLCPP_ERROR(get_logger(), "could not activate control");
    return false;
  }
  std_msgs::msg::Bool command_state;
  command_state.data = true;
  command_state_changed_publisher_->publish(command_state);
  return true;
}

bool RobotManagerNode::deactivate()
{
  if (!robot_manager_->isConnected()) {
    RCLCPP_ERROR(get_logger(), "not connected");
    return false;
  }

  if (this->isActive() && !robot_manager_->deactivateControl()) {
    RCLCPP_ERROR(get_logger(), "could not deactivate control");
    return false;
  }
  this->ActivatableInterface::deactivate();

  if (!setRobotControlNodeCommandState(false)) {
    // TODO(resizoltan) out of sync
    RCLCPP_ERROR(get_logger(), "could not set command state");
    return false;
  }
  std_msgs::msg::Bool command_state;
  command_state.data = false;
  command_state_changed_publisher_->publish(command_state);
  return true;
}

bool RobotManagerNode::requestRobotControlNodeStateTransition(std::uint8_t transition)
{
  auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
  request->transition.id = transition;
  if (!change_robot_control_state_client_->wait_for_service(std::chrono::milliseconds(2000))) {
    RCLCPP_ERROR(get_logger(), "Wait for service failed");
    return false;
  }
  auto future_result = change_robot_control_state_client_->async_send_request(request);
  auto future_status = wait_for_result(future_result, std::chrono::milliseconds(3000));
  if (future_status != std::future_status::ready) {
    RCLCPP_ERROR(get_logger(), "Future status not ready, could not change robot control state");
    return false;
  }
  if (future_result.get()->success) {
    return true;
  } else {
    RCLCPP_ERROR(get_logger(), "Future result not success, could not change robot control state");
    return false;
  }
}

bool RobotManagerNode::setRobotControlNodeCommandState(bool active)
{
  auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
  request->data = active;

  auto future_result = set_command_state_client_->async_send_request(request);
  auto future_status = wait_for_result(future_result, std::chrono::milliseconds(3000));
  if (future_status != std::future_status::ready) {
    RCLCPP_ERROR(get_logger(), "Future status not ready, could not set robot command state");
    return false;
  }

  if (!future_result.get()->success) {
    RCLCPP_ERROR(get_logger(), "Future result not success, could not set robot command state");
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
  if (get_current_state().label() == "active") {
    deactivate();
  }
  this->LifecycleNode::deactivate();
}

}  // namespace kuka_sunrise

int main(int argc, char * argv[])
{
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor;
  auto node = std::make_shared<kuka_sunrise::RobotManagerNode>();
  executor.add_node(node->get_node_base_interface());
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
