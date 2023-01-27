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

#include "kuka_sunrise/robot_manager_node.hpp"

namespace kuka_sunrise
{

RobotManagerNode::RobotManagerNode()
: kroshu_ros2_core::ROS2BaseLCNode("robot_manager")
{
  // Controllers do not support the cleanup transition (as of now)
  // Therefore controllers are loaded and configured at startup, only activation
  //   and deactivation is managed by this node
  // There are two kind of controllers used:
  //  - RT: joint state broadcaster and joint position/effort commander
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
  change_hardware_state_client_ =
    this->create_client<controller_manager_msgs::srv::SetHardwareComponentState>(
    "controller_manager/set_hardware_component_state", qos.get_rmw_qos_profile(), cbg_);
  change_controller_state_client_ =
    this->create_client<controller_manager_msgs::srv::SwitchController>(
    "controller_manager/switch_controller", qos.get_rmw_qos_profile(), cbg_);
  command_state_changed_publisher_ = this->create_publisher<std_msgs::msg::Bool>(
    "robot_manager/commanding_state_changed", qos);
  set_parameter_client_ = this->create_client<std_srvs::srv::Trigger>(
    "configuration_manager/set_params", ::rmw_qos_profile_default, cbg_);

  auto is_configured_qos = rclcpp::QoS(rclcpp::KeepLast(1));
  is_configured_qos.best_effort();

  is_configured_pub_ = this->create_publisher<std_msgs::msg::Bool>(
    "robot_manager/is_configured",
    is_configured_qos);
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
    kroshu_ros2_core::sendRequest<controller_manager_msgs::srv::SetHardwareComponentState::Response>(
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
    kroshu_ros2_core::sendRequest<controller_manager_msgs::srv::SwitchController::Response>(
    change_controller_state_client_, controller_request, 0, 3000);
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

  if (result == SUCCESS) {
    auto trigger_request =
      std::make_shared<std_srvs::srv::Trigger::Request>();
    auto response = kroshu_ros2_core::sendRequest<std_srvs::srv::Trigger::Response>(
      set_parameter_client_, trigger_request, 0, 1000);

    if (!response || !response->success) {
      RCLCPP_ERROR(get_logger(), "Could not set parameters");
      result = FAILURE;
    }
  }
  if (result != SUCCESS) {
    this->on_cleanup(get_current_state());
  }

  is_configured_pub_->on_activate();
  is_configured_msg_.data = true;
  is_configured_pub_->publish(is_configured_msg_);

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
  // With best effort strictness, cleanup succeeds if specific controller is not active
  controller_request->strictness =
    controller_manager_msgs::srv::SwitchController::Request::BEST_EFFORT;
  controller_request->deactivate_controllers =
    std::vector<std::string>{"timing_controller", "robot_state_broadcaster"};
  auto controller_response =
    kroshu_ros2_core::sendRequest<controller_manager_msgs::srv::SwitchController::Response>(
    change_controller_state_client_, controller_request, 0, 2000);
  if (!controller_response || !controller_response->ok) {
    RCLCPP_ERROR(get_logger(), "Could not stop controllers");
    return ERROR;
  }

  // Cleanup hardware interface
  // If it is inactive, cleanup will also succeed
  auto hw_request =
    std::make_shared<controller_manager_msgs::srv::SetHardwareComponentState::Request>();
  hw_request->name = "iiwa_hardware";
  hw_request->target_state.label = "unconfigured";
  auto hw_response =
    kroshu_ros2_core::sendRequest<controller_manager_msgs::srv::SetHardwareComponentState::Response>(
    change_hardware_state_client_, hw_request, 0, 2000);
  if (!hw_response || !hw_response->ok) {
    RCLCPP_ERROR(get_logger(), "Could not cleanup hardware interface");
    return FAILURE;
  }

  if (is_configured_pub_->is_activated()) {
    is_configured_pub_->on_deactivate();
    is_configured_msg_.data = false;
    is_configured_pub_->publish(is_configured_msg_);
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

  // Publish message to notify other nodes about shutdown
  auto control_ended_pub = create_publisher<std_msgs::msg::Bool>("control_ended", rclcpp::QoS(1));
  std_msgs::msg::Bool end;
  end.data = true;
  control_ended_pub->publish(end);

  return SUCCESS;
}

// TODO(Svastits): activation fails after deactivation
// TODO(Svastits): check if we have to send unconfigured msg to control node
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
RobotManagerNode::on_activate(const rclcpp_lifecycle::State &)
{
  if (!robot_manager_->isConnected()) {
    RCLCPP_ERROR(get_logger(), "not connected");
    return ERROR;
  }

  auto send_period_ms = static_cast<int>(this->get_parameter("send_period_ms").as_int());
  auto receive_multiplier = static_cast<int>(this->get_parameter("receive_multiplier").as_int());
  if (!robot_manager_->setFRIConfig(30200, send_period_ms, receive_multiplier)) {
    RCLCPP_ERROR(get_logger(), "could not set FRI config");
    return FAILURE;
  }
  RCLCPP_INFO(get_logger(), "Successfully set FRI config");

  // Activate hardware interface
  auto hw_request =
    std::make_shared<controller_manager_msgs::srv::SetHardwareComponentState::Request>();
  hw_request->name = "iiwa_hardware";
  hw_request->target_state.label = "active";
  auto hw_response =
    kroshu_ros2_core::sendRequest<controller_manager_msgs::srv::SetHardwareComponentState::Response>(
    change_hardware_state_client_, hw_request, 0, 2000);
  if (!hw_response || !hw_response->ok) {
    RCLCPP_ERROR(get_logger(), "Could not activate hardware interface");
    // 'unset config' does not exist, safe to return
    return FAILURE;
  }
  RCLCPP_INFO(get_logger(), "Activated LBR iiwa hardware interface");

  // Start FRI (in monitoring mode)
  if (!robot_manager_->startFRI()) {
    RCLCPP_ERROR(get_logger(), "Could not start FRI");
    this->on_deactivate(get_current_state());
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
    kroshu_ros2_core::sendRequest<controller_manager_msgs::srv::SwitchController::Response>(
    change_controller_state_client_, controller_request, 0, 2000);
  if (!controller_response || !controller_response->ok) {
    RCLCPP_ERROR(get_logger(), "Could not start joint state broadcaster");
    this->on_deactivate(get_current_state());
    return FAILURE;
  }

  auto position_controller_name = this->get_parameter("position_controller_name").as_string();
  auto torque_controller_name = this->get_parameter("torque_controller_name").as_string();
  controller_name_ = (this->get_parameter("command_mode").as_string() ==
    "position") ? position_controller_name : torque_controller_name;
  // Activate RT commander
  controller_request->strictness = controller_manager_msgs::srv::SwitchController::Request::STRICT;
  controller_request->activate_controllers = std::vector<std::string>{controller_name_};
  controller_response =
    kroshu_ros2_core::sendRequest<controller_manager_msgs::srv::SwitchController::Response>(
    change_controller_state_client_, controller_request, 0, 2000);
  if (!controller_response || !controller_response->ok) {
    RCLCPP_ERROR(get_logger(), "Could not activate controller");
    this->on_deactivate(get_current_state());
    return FAILURE;
  }
  command_state_changed_publisher_->on_activate();

  // Start commanding mode
  if (!activateControl()) {
    this->on_deactivate(get_current_state());
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

  if (!this->deactivateControl()) {
    RCLCPP_ERROR(get_logger(), "Could not deactivate control");
    return ERROR;
  }

  if (!robot_manager_->endFRI()) {
    RCLCPP_ERROR(get_logger(), "Could not end FRI");
    return ERROR;
  }

  // Deactivate hardware interface
  // If it is inactive, deactivation will also succeed
  auto hw_request =
    std::make_shared<controller_manager_msgs::srv::SetHardwareComponentState::Request>();
  hw_request->name = "iiwa_hardware";
  hw_request->target_state.label = "inactive";
  auto hw_response =
    kroshu_ros2_core::sendRequest<controller_manager_msgs::srv::SetHardwareComponentState::Response>(
    change_hardware_state_client_, hw_request, 0, 2000);
  if (!hw_response || !hw_response->ok) {
    RCLCPP_ERROR(get_logger(), "Could not deactivate hardware interface");
    return ERROR;
  }
  RCLCPP_INFO(get_logger(), "Deactivated LBR iiwa hardware interface");

  // Stop RT controllers
  auto controller_request =
    std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
  // With best effort strictness, deactivation succeeds if specific controller is not active
  controller_request->strictness =
    controller_manager_msgs::srv::SwitchController::Request::BEST_EFFORT;
  controller_request->deactivate_controllers =
    std::vector<std::string>{controller_name_, "joint_state_broadcaster"};
  auto controller_response =
    kroshu_ros2_core::sendRequest<controller_manager_msgs::srv::SwitchController::Response>(
    change_controller_state_client_, controller_request, 0, 2000);
  if (!controller_response || !controller_response->ok) {
    RCLCPP_ERROR(get_logger(), "Could not stop controllers");
    return ERROR;
  }

  if (command_state_changed_publisher_->is_activated()) {
    command_state_changed_publisher_->on_deactivate();
  }

  return SUCCESS;
}

bool RobotManagerNode::activateControl()
{
  if (!robot_manager_->isConnected()) {
    RCLCPP_ERROR(get_logger(), "Not connected");
    return false;
  }

  if (!robot_manager_->activateControl()) {
    RCLCPP_ERROR(get_logger(), "Could not activate control");
    return false;
  }
  std_msgs::msg::Bool command_state;
  command_state.data = true;
  command_state_changed_publisher_->publish(command_state);
  return true;
}

bool RobotManagerNode::deactivateControl()
{
  if (!robot_manager_->isConnected()) {
    RCLCPP_ERROR(get_logger(), "Not connected");
    return false;
  }

  if (!robot_manager_->deactivateControl()) {
    RCLCPP_ERROR(get_logger(), "Could not deactivate control");
    return false;
  }

  std_msgs::msg::Bool command_state;
  command_state.data = false;
  command_state_changed_publisher_->publish(command_state);
  return true;
}

void RobotManagerNode::handleControlEndedError()
{
  RCLCPP_INFO(get_logger(), "Control ended");
  this->LifecycleNode::deactivate();
}

void RobotManagerNode::handleFRIEndedError()
{
  RCLCPP_INFO(get_logger(), "FRI ended");
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
