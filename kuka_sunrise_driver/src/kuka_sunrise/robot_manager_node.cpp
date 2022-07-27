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
: kroshu_ros2_core::ROS2BaseLCNode("robot_manager")
{
  // Controllers do not support the cleanup transition (as of now)
  // Therefore controllers are loaded and configured at startup, only activation
  //   and deactivation is managed by this node
  // There are two kind of controllers used:
  //  - RT: joint state broadcaster and joint state/effort commander
  //  - non-RT: configuration (workaround until runtime parameters are enabled)
  //            and robot state broadcaster
  // RT controllers are started after interface activation
  // non-RT controllers are started after interface configuration

  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  robot_manager_ = std::make_shared<RobotManager>(
    [this]
    {this->handleControlEndedError();}, [this]
    {this->handleFRIEndedError();});
  auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
  qos.reliable();
  cbg_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  change_robot_control_state_client_ = this->create_client<lifecycle_msgs::srv::ChangeState>(
    "robot_control/change_state", qos.get_rmw_qos_profile(), cbg_);
  set_commanding_state_client_ = this->create_client<std_srvs::srv::SetBool>(
    "robot_control/set_commanding_state", qos.get_rmw_qos_profile(), cbg_);
  change_hardware_state_client_ =
    this->create_client<controller_manager_msgs::srv::SetHardwareComponentState>(
    "controller_manager/set_hardware_component_state", qos.get_rmw_qos_profile(), cbg_);
  change_controller_state_client_ =
    this->create_client<controller_manager_msgs::srv::SwitchController>(
    "controller_manager/switch_controller", qos.get_rmw_qos_profile(), cbg_);
  auto command_srv_callback = [this](
    std_srvs::srv::SetBool::Request::SharedPtr request,
    std_srvs::srv::SetBool::Response::SharedPtr response) {
      if (request->data == true) {
        response->success = this->activate();
      } else {
        response->success = this->deactivate();
      }
    };
  change_robot_commanding_state_service_ = this->create_service<std_srvs::srv::SetBool>(
    "robot_manager/set_commanding_state", command_srv_callback);
  command_state_changed_publisher_ = this->create_publisher<std_msgs::msg::Bool>(
    "robot_manager/commanding_state_changed", qos);
  set_parameter_client_ = this->create_client<std_srvs::srv::Trigger>(
    "configuration_manager/set_params", ::rmw_qos_profile_default, cbg_);
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
RobotManagerNode::on_configure(const rclcpp_lifecycle::State &)
{
  // Configure hardware interface
  auto hw_request =
    std::make_shared<controller_manager_msgs::srv::SetHardwareComponentState::Request>();
  hw_request->name = "iiwa_hardware";
  hw_request->target_state.label = "inactive";
  auto hw_response =
    kuka_sunrise::sendRequest<controller_manager_msgs::srv::SetHardwareComponentState::Response>(
    change_hardware_state_client_, hw_request, 0, 2000);
  if (!hw_response || !hw_response->ok) {
    RCLCPP_ERROR(get_logger(), "Could not configure hardware interface");
    return FAILURE;
  }

  auto result = SUCCESS;

  // If this fails, the node should be restarted, with different parameter values
  // Therefore exceptions are not caught
  if (!configuration_manager_) {
    configuration_manager_ = std::make_unique<ConfigurationManager>(
      std::dynamic_pointer_cast<kroshu_ros2_core::ROS2BaseLCNode>(
        this->shared_from_this()), robot_manager_);
  }
  RCLCPP_INFO(get_logger(), "Successfully set 'controller_ip' parameter");

  // Start non-RT controllers
  auto controller_request =
    std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
  controller_request->strictness = controller_manager_msgs::srv::SwitchController::Request::STRICT;
  controller_request->activate_controllers =
    std::vector<std::string>{"timing_controller", "robot_state_broadcaster"};
  auto controller_response =
    kuka_sunrise::sendRequest<controller_manager_msgs::srv::SwitchController::Response>(
    change_controller_state_client_, controller_request, 0, 2000);
  if (!controller_response || !controller_response->ok) {
    RCLCPP_ERROR(get_logger(), "Could not start controllers");
    result = FAILURE;
  }

  const char * controller_ip = this->get_parameter("controller_ip").as_string().c_str();
  if (!robot_manager_->isConnected()) {
    if (!robot_manager_->connect(controller_ip, 30000)) {
      RCLCPP_ERROR(get_logger(), "could not connect");
      result = FAILURE;
    }
  } else {
    RCLCPP_ERROR(get_logger(), "Robot manager is connected in inactive state");
    return ERROR;
  }
  RCLCPP_INFO(get_logger(), "Successfully connected to FRI");
  // TODO(resizoltan) get IO configuration

  if (result == SUCCESS) {
    auto trigger_request =
      std::make_shared<std_srvs::srv::Trigger::Request>();
    auto response = kuka_sunrise::sendRequest<std_srvs::srv::Trigger::Response>(
      set_parameter_client_, trigger_request, 0, 1000);

    if (!response || !response->success) {
      RCLCPP_ERROR(get_logger(), "Could not set parameters");
      result = FAILURE;
    }
  }
  if (result != SUCCESS) {
    this->on_cleanup(get_current_state());
  }
  return result;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
RobotManagerNode::on_cleanup(const rclcpp_lifecycle::State &)
{
  if (robot_manager_->isConnected() && !robot_manager_->disconnect()) {
    RCLCPP_ERROR(get_logger(), "could not disconnect");
    return ERROR;
  }

  // Stop non-RT controllers
  auto controller_request =
    std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
  controller_request->strictness = controller_manager_msgs::srv::SwitchController::Request::STRICT;
  controller_request->deactivate_controllers =
    std::vector<std::string>{"timing_controller", "robot_state_broadcaster"};
  auto controller_response =
    kuka_sunrise::sendRequest<controller_manager_msgs::srv::SwitchController::Response>(
    change_controller_state_client_, controller_request, 0, 2000);
  if (!controller_response || !controller_response->ok) {
    RCLCPP_ERROR(get_logger(), "Could not stop controllers");
    return ERROR;
  }

  // Cleanup hardware interface
  auto hw_request =
    std::make_shared<controller_manager_msgs::srv::SetHardwareComponentState::Request>();
  hw_request->name = "iiwa_hardware";
  hw_request->target_state.label = "unconfigured";
  auto hw_response =
    kuka_sunrise::sendRequest<controller_manager_msgs::srv::SetHardwareComponentState::Response>(
    change_hardware_state_client_, hw_request, 0, 2000);
  if (!hw_response || !hw_response->ok) {
    RCLCPP_ERROR(get_logger(), "Could not cleanup hardware interface");
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

  // TODO(Svasits): unload controllers, destroy HWInterface
  return SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
RobotManagerNode::on_activate(const rclcpp_lifecycle::State &)
{
  // TODO(Svastits): implement rollback mechanism
  if (!robot_manager_->isConnected()) {
    RCLCPP_ERROR(get_logger(), "not connected");
    return ERROR;
  }

  auto send_period_ms = static_cast<int>(this->get_parameter("send_period_ms").as_int());
  auto receive_multiplier = static_cast<int>(this->get_parameter("receive_multiplier").as_int());
  if (!robot_manager_->setFRIConfig(30200, send_period_ms, receive_multiplier)) {
    RCLCPP_ERROR(get_logger(), "could not set fri config");
    return FAILURE;
  }
  RCLCPP_INFO(get_logger(), "Successfully set FRI config");

  // Activate hardware interface
  auto hw_request =
    std::make_shared<controller_manager_msgs::srv::SetHardwareComponentState::Request>();
  hw_request->name = "iiwa_hardware";
  hw_request->target_state.label = "active";
  auto hw_response =
    kuka_sunrise::sendRequest<controller_manager_msgs::srv::SetHardwareComponentState::Response>(
    change_hardware_state_client_, hw_request, 0, 2000);
  if (!hw_response || !hw_response->ok) {
    RCLCPP_ERROR(get_logger(), "Could not activate hardware interface");
    return FAILURE;
  }
  RCLCPP_INFO(get_logger(), "Activated LBR iiwa hardware interface");

  // Start FRI (in monitoring mode)
  if (!robot_manager_->startFRI()) {
    RCLCPP_ERROR(get_logger(), "Could not start FRI");
    return FAILURE;
  }
  RCLCPP_INFO(get_logger(), "Started FRI");

  // Activate joint state broadcaster
  // The other controller must be started later so that it can initialize internal state
  //   with broadcaster information
  auto controller_request =
    std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
  controller_request->strictness = controller_manager_msgs::srv::SwitchController::Request::STRICT;
  controller_request->activate_controllers = std::vector<std::string>{"joint_state_broadcaster"};
  auto controller_response =
    kuka_sunrise::sendRequest<controller_manager_msgs::srv::SwitchController::Response>(
    change_controller_state_client_, controller_request, 0, 2000);
  if (!controller_response || !controller_response->ok) {
    RCLCPP_ERROR(get_logger(), "Could not start controllers");
    return FAILURE;
  }

  // Activate RT commander
  controller_request->strictness = controller_manager_msgs::srv::SwitchController::Request::STRICT;
  controller_request->activate_controllers =
    std::vector<std::string>{(this->get_parameter("command_mode").as_string() ==
    "position") ? "position_controller" : "effort_controller"};
  controller_request->deactivate_controllers =
    std::vector<std::string>{(this->get_parameter("command_mode").as_string() ==
    "position") ? "effort_controller" : "position_controller"};
  controller_response =
    kuka_sunrise::sendRequest<controller_manager_msgs::srv::SwitchController::Response>(
    change_controller_state_client_, controller_request, 0, 2000);
  if (!controller_response || !controller_response->ok) {
    RCLCPP_ERROR(get_logger(), "Could not activate controller");
    return FAILURE;
  }
  command_state_changed_publisher_->on_activate();

  // Start commanding mode
  if (!activate()) {
    return FAILURE;
  }

  return SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
RobotManagerNode::on_deactivate(const rclcpp_lifecycle::State &)
{
  if (!robot_manager_->isConnected()) {
    RCLCPP_ERROR(get_logger(), "Not connected");
    return ERROR;
  }

  if (this->isActive() && !this->deactivate()) {
    RCLCPP_ERROR(get_logger(), "Could not deactivate control");
    return ERROR;
  }

  if (!robot_manager_->endFRI()) {
    RCLCPP_ERROR(get_logger(), "Could not end FRI");
    return ERROR;
  }

  // Deactivate hardware interface
  auto hw_request =
    std::make_shared<controller_manager_msgs::srv::SetHardwareComponentState::Request>();
  hw_request->name = "iiwa_hardware";
  hw_request->target_state.label = "inactive";
  auto hw_response =
    kuka_sunrise::sendRequest<controller_manager_msgs::srv::SetHardwareComponentState::Response>(
    change_hardware_state_client_, hw_request, 0, 2000);
  if (!hw_response || !hw_response->ok) {
    RCLCPP_ERROR(get_logger(), "Could not deactivate hardware interface");
    return ERROR;
  }
  RCLCPP_INFO(get_logger(), "Deactivated LBR iiwa hardware interface");


  // Stop RT controllers
  auto controller_request =
    std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
  controller_request->strictness = controller_manager_msgs::srv::SwitchController::Request::STRICT;
  controller_request->deactivate_controllers =
    std::vector<std::string>{"joint_state_broadcaster",
    this->get_parameter("controller_name").as_string()};
  auto controller_response =
    kuka_sunrise::sendRequest<controller_manager_msgs::srv::SwitchController::Response>(
    change_controller_state_client_, controller_request, 0, 2000);
  if (!controller_response || !controller_response->ok) {
    RCLCPP_ERROR(get_logger(), "Could not stop controllers");
    return ERROR;
  }

  command_state_changed_publisher_->on_deactivate();

  return SUCCESS;
}

bool RobotManagerNode::activate()
{
  this->ActivatableInterface::activate();
  if (!robot_manager_->isConnected()) {
    RCLCPP_ERROR(get_logger(), "Not connected");
    return false;
  }

  if (!robot_manager_->activateControl()) {
    this->ActivatableInterface::deactivate();
    RCLCPP_ERROR(get_logger(), "Could not activate control");
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
    RCLCPP_ERROR(get_logger(), "Not connected");
    return false;
  }

  if (this->isActive() && !robot_manager_->deactivateControl()) {
    RCLCPP_ERROR(get_logger(), "Could not deactivate control");
    return false;
  }
  this->ActivatableInterface::deactivate();

  std_msgs::msg::Bool command_state;
  command_state.data = false;
  command_state_changed_publisher_->publish(command_state);
  return true;
}

bool RobotManagerNode::requestRobotControlNodeStateTransition(std::uint8_t transition)
{
  auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
  request->transition.id = transition;

  auto response = kuka_sunrise::sendRequest<lifecycle_msgs::srv::ChangeState::Response>(
    change_robot_control_state_client_, request, 2000, 3000);

  if (!response || !response->success) {
    RCLCPP_ERROR(get_logger(), "Could not change robot control state");
    return false;
  }
  return true;
}

bool RobotManagerNode::setRobotControlNodeCommandState(bool active)
{
  auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
  request->data = active;

  auto response = kuka_sunrise::sendRequest<std_srvs::srv::SetBool::Response>(
    set_commanding_state_client_, request, 0, 1000);

  if (!response || !response->success) {
    RCLCPP_ERROR(get_logger(), "Could not set robot command state");
    return false;
  }
  return true;
}

void RobotManagerNode::handleControlEndedError()
{
  // TODO(Svastits): deactivate managers by internal control ended error
  //   currently the hardware interface deactivation fails with timeout
  //   (possible threading issue)
  RCLCPP_INFO(get_logger(), "Control ended");
  if (this->on_deactivate(get_current_state()) != SUCCESS) {
    RCLCPP_FATAL(get_logger(), "Could not deactivate managers");
  }
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
