// Copyright 2022 Komáromi Sándor
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

#include "kuka_rox_hw_interface/robot_manager_node.hpp"

namespace kuka_rox
{

RobotManagerNode::RobotManagerNode()
: kroshu_ros2_core::ROS2BaseLCNode("robot_manager")
{
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
  qos.reliable();
  cbg_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  change_hardware_state_client_ =
    this->create_client<controller_manager_msgs::srv::SetHardwareComponentState>(
    "controller_manager/set_hardware_component_state", qos.get_rmw_qos_profile(), cbg_
    );
  change_controller_state_client_ =
    this->create_client<controller_manager_msgs::srv::SwitchController>(
    "controller_manager/switch_controller", qos.get_rmw_qos_profile(), cbg_
    );

  auto is_configured_qos = rclcpp::QoS(rclcpp::KeepLast(1));
  is_configured_qos.best_effort();

  is_configured_pub_ = this->create_publisher<std_msgs::msg::Bool>(
    "robot_manager/is_configured",
    is_configured_qos);

  this->registerParameter<std::string>(
    "control_mode", POSITION_CONTROL, kroshu_ros2_core::ParameterSetAccessRights {false, true, true, 
    false, true}, [this](const std::string & control_mode){
      return this->onControlModeChangeRequest(control_mode);
    });
  this->registerParameter<std::string>(
    POSITION_CONTROLLER_NAME, "", kroshu_ros2_core::ParameterSetAccessRights {false, true, false, 
    false, true}, [this](const std::string & controller_name){
      return true;
    });
  this->registerParameter<std::string>(
    IMPEDANCE_CONTROLLER_NAME, "", kroshu_ros2_core::ParameterSetAccessRights {false, true, false, 
    false, true}, [this](const std::string & controller_name){
      return true;
    });
  this->registerParameter<std::string>(
    TORQUE_CONTROLLER_NAME, "", kroshu_ros2_core::ParameterSetAccessRights {false, true, false, 
    false, true}, [this](const std::string & controller_name){
      return true;
    });
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
RobotManagerNode::on_configure(const rclcpp_lifecycle::State &)
{
  // Configure hardware interface
  auto hw_request =
    std::make_shared<controller_manager_msgs::srv::SetHardwareComponentState::Request>();
  hw_request->name = "iisy_hardware";
  hw_request->target_state.label = "inactive";
  auto hw_response =
    kuka_sunrise::sendRequest<controller_manager_msgs::srv::SetHardwareComponentState::Response>(
    change_hardware_state_client_, hw_request, 0, 2000
    );
  if (!hw_response || !hw_response->ok) {
    RCLCPP_ERROR(get_logger(), "Could not configure hardware interface");
    return FAILURE;
  }

  is_configured_pub_->on_activate();
  is_configured_msg_.data = true;
  is_configured_pub_->publish(is_configured_msg_);
  return SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
RobotManagerNode::on_cleanup(const rclcpp_lifecycle::State &)
{
  // Cleanup hardware interface
  auto hw_request =
    std::make_shared<controller_manager_msgs::srv::SetHardwareComponentState::Request>();
  hw_request->name = "iisy_hardware";
  hw_request->target_state.label = "inactive";
  auto hw_response =
    kuka_sunrise::sendRequest<controller_manager_msgs::srv::SetHardwareComponentState::Response>(
    change_hardware_state_client_, hw_request, 0, 2000
    );
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

  return SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
RobotManagerNode::on_activate(const rclcpp_lifecycle::State &)
{
  // Activate hardware interface
  auto hw_request =
    std::make_shared<controller_manager_msgs::srv::SetHardwareComponentState::Request>();
  hw_request->name = "iisy_hardware";
  hw_request->target_state.label = "active";
  auto hw_response =
    kuka_sunrise::sendRequest<controller_manager_msgs::srv::SetHardwareComponentState::Response>(
    change_hardware_state_client_, hw_request, 0, 2000
    );
  if (!hw_response || !hw_response->ok) {
    RCLCPP_ERROR(get_logger(), "Could not activate hardware interface");
    // 'unset config' does not exist, safe to return
    return FAILURE;
  }

  // Activate joint state broadcaster
  auto controller_request =
    std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
  controller_request->strictness = controller_manager_msgs::srv::SwitchController::Request::STRICT;
  controller_request->activate_controllers = std::vector<std::string>{"joint_state_broadcaster"};
  auto controller_response =
    kuka_sunrise::sendRequest<controller_manager_msgs::srv::SwitchController::Response>(
    change_controller_state_client_, controller_request, 0, 2000
    );
  if (!controller_response || !controller_response->ok) {
    RCLCPP_ERROR(get_logger(), "Could not start joint state broadcaster");
    this->on_deactivate(get_current_state());
    return FAILURE;
  }

  // Select control modes
  auto position_controller_name = this->get_parameter("position_controller_name").as_string();
  auto impedance_controller_name = this->get_parameter("impedance_controller_name").as_string();
  auto torque_controller_name = this->get_parameter("torque_controller_name").as_string();
  auto control_mode = this->get_parameter("control_mode").as_string();
  controller_names_.clear();
  if (control_mode == POSITION_CONTROL)
  {
    controller_names_.emplace_back("joint_trajectory_controller");
  }
  else if (control_mode == IMPEDANCE_CONTROL)
  {
    controller_names_.emplace_back("joint_trajectory_controller");
    controller_names_.emplace_back("joint_impedance_controller");
  }
  else if (control_mode == TORQUE_CONTROL)
  {

  }
  else
  {
    RCLCPP_ERROR(get_logger(), "Not valid control mode, control mode set to: %s", control_mode);
    return ERROR;
  }
  
  // Activate RT commander
  controller_request->strictness = controller_manager_msgs::srv::SwitchController::Request::STRICT;
  controller_request->activate_controllers = controller_names_;
  controller_response =
    kuka_sunrise::sendRequest<controller_manager_msgs::srv::SwitchController::Response>(
    change_controller_state_client_, controller_request, 0, 2000
    );
  if (!controller_response || !controller_response->ok) {
    RCLCPP_ERROR(get_logger(), "Could not  activate controller");
    this->on_deactivate(get_current_state());
    return FAILURE;
  }

  return SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
RobotManagerNode::on_deactivate(const rclcpp_lifecycle::State &)
{
  // Deactivate hardware interface
  auto hw_request =
    std::make_shared<controller_manager_msgs::srv::SetHardwareComponentState::Request>();
  hw_request->name = "iisy_hardware";
  hw_request->target_state.label = "inactive";
  auto hw_response =
    kuka_sunrise::sendRequest<controller_manager_msgs::srv::SetHardwareComponentState::Response>(
    change_hardware_state_client_, hw_request, 0, 2000
    );
  if (!hw_response || !hw_response->ok) {
    RCLCPP_ERROR(get_logger(), "Could not deactivate hardware interface");
    return ERROR;
  }

  RCLCPP_INFO(get_logger(), "Deactivated LBR iisy hardware interface");

  // Stop RT controllers
  controller_names_.emplace_back("joint_state_broadcaster");
  auto controller_request =
    std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
  // With best effort strictness, deactivation succeeds if specific controller is not active
  controller_request->strictness =
    controller_manager_msgs::srv::SwitchController::Request::BEST_EFFORT;
  controller_request->deactivate_controllers = controller_names_;
  auto controller_response =
    kuka_sunrise::sendRequest<controller_manager_msgs::srv::SwitchController::Response>(
    change_controller_state_client_, controller_request, 0, 2000
    );
  if (!controller_response || !controller_response->ok) {
    RCLCPP_ERROR(get_logger(), "Could not stop controllers");
    return ERROR;
  }
  return SUCCESS;
}

bool RobotManagerNode::onControlModeChangeRequest(const std::string & control_mode)
{
  if(control_mode == POSITION_CONTROL ||
     control_mode == IMPEDANCE_CONTROL ||
     control_mode == TORQUE_CONTROL)
  {

  }

  // if (control_mode == POSITION_CONTROL) {
  //   return robot_manager_->setPositionControlMode();
  // } else if (control_mode == IMPEDANCE_CONTROL) {
  //   try {
  //     return robot_manager_->setJointImpedanceControlMode(
  //       joint_stiffness_,
  //       joint_damping_);
  //   } catch (const std::exception & e) {
  //     RCLCPP_ERROR(robot_manager_node_->get_logger(), e.what());
  //   }
  //   return false;
  // } else {
  //   RCLCPP_ERROR(
  //     robot_manager_node_->get_logger(), "Control mode should be '%s' or '%s'",
  //     POSITION_CONTROL.c_str(), IMPEDANCE_CONTROL.c_str());
  //   return false;
  // }
}
}  // namespace kuka_rox

int main(int argc, char * argv[])
{
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor;
  auto node = std::make_shared<kuka_rox::RobotManagerNode>();
  executor.add_node(node->get_node_base_interface());
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
