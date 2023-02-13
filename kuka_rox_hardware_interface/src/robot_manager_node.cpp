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

using namespace controller_manager_msgs::srv;  // NOLINT

namespace kuka_rox
{

RobotManagerNode::RobotManagerNode()
: kroshu_ros2_core::ROS2BaseLCNode("robot_manager")
{
  RCLCPP_INFO(
    get_logger(), "Starting Robot Manager Node init"
  );

  auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
  qos.reliable();
  cbg_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  change_hardware_state_client_ =
    this->create_client<SetHardwareComponentState>(
    "controller_manager/set_hardware_component_state", qos.get_rmw_qos_profile(), cbg_
    );
  change_controller_state_client_ =
    this->create_client<SwitchController>(
    "controller_manager/switch_controller", qos.get_rmw_qos_profile(), cbg_
    );

  auto is_configured_qos = rclcpp::QoS(rclcpp::KeepLast(1));
  is_configured_qos.best_effort();

  is_configured_pub_ = this->create_publisher<std_msgs::msg::Bool>(
    "robot_manager/is_configured",
    is_configured_qos);

  control_mode_map_.emplace(std::make_pair(POSITION_CONTROL, std::vector<std::string>()));
  control_mode_map_.emplace(std::make_pair(IMPEDANCE_CONTROL, std::vector<std::string>()));
  control_mode_map_.emplace(std::make_pair(TORQUE_CONTROL, std::vector<std::string>()));
  control_mode_map_.at(POSITION_CONTROL).resize(1);
  control_mode_map_.at(IMPEDANCE_CONTROL).resize(2);
  control_mode_map_.at(TORQUE_CONTROL).resize(1);

  this->registerParameter<std::string>(
    "control_mode", POSITION_CONTROL, kroshu_ros2_core::ParameterSetAccessRights {true, true, false,
      false, false}, [this](const std::string & control_mode) {
      return this->onControlModeChangeRequest(control_mode);
    });
  this->registerParameter<std::string>(
    POSITION_CONTROLLER_NAME_PARAM, "", kroshu_ros2_core::ParameterSetAccessRights {true, true,
      false, false, false}, [this](const std::string & controller_name) {
      return this->onControllerNameChangeRequest(controller_name, POSITION_CONTROLLER_NAME_PARAM);
    });
  this->registerParameter<std::string>(
    IMPEDANCE_CONTROLLER_NAME_PARAM, "", kroshu_ros2_core::ParameterSetAccessRights {true, true,
      false, false, false}, [this](const std::string & controller_name) {
      return this->onControllerNameChangeRequest(controller_name, IMPEDANCE_CONTROLLER_NAME_PARAM);
    });
  this->registerParameter<std::string>(
    TORQUE_CONTROLLER_NAME_PARAM, "", kroshu_ros2_core::ParameterSetAccessRights {true, true, false,
      false, false}, [this](const std::string & controller_name) {
      return this->onControllerNameChangeRequest(controller_name, TORQUE_CONTROLLER_NAME_PARAM);
    });

  RCLCPP_INFO(
    get_logger(), "Robot Manager Node init finished without error"
  );
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
RobotManagerNode::on_configure(const rclcpp_lifecycle::State &)
{
  // Configure hardware interface
  auto hw_request =
    std::make_shared<SetHardwareComponentState::Request>();
  hw_request->name = "iisy_hardware";
  hw_request->target_state.label = "inactive";
  auto hw_response =
    kroshu_ros2_core::sendRequest<SetHardwareComponentState::Response>(
    change_hardware_state_client_, hw_request, 0, 2000);
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
    std::make_shared<SetHardwareComponentState::Request>();
  hw_request->name = "iisy_hardware";
  hw_request->target_state.label = "inactive";
  auto hw_response =
    kroshu_ros2_core::sendRequest<SetHardwareComponentState::Response>(
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
RobotManagerNode::on_activate(const rclcpp_lifecycle::State &)
{
  // Activate hardware interface
  auto hw_request =
    std::make_shared<SetHardwareComponentState::Request>();
  hw_request->name = "iisy_hardware";
  hw_request->target_state.label = "active";
  auto hw_response =
    kroshu_ros2_core::sendRequest<SetHardwareComponentState::Response>(
    change_hardware_state_client_, hw_request, 0, 2000);
  if (!hw_response || !hw_response->ok) {
    RCLCPP_ERROR(get_logger(), "Could not activate hardware interface");
    // 'unset config' does not exist, safe to return
    is_configured_msg_.data = false;
    is_configured_pub_->publish(is_configured_msg_);
    return FAILURE;
  }

  // Activate joint state broadcaster
  auto controller_request =
    std::make_shared<SwitchController::Request>();
  controller_request->strictness = SwitchController::Request::STRICT;
  controller_request->activate_controllers = std::vector<std::string>{"joint_state_broadcaster"};
  auto controller_response =
    kroshu_ros2_core::sendRequest<SwitchController::Response>(
    change_controller_state_client_, controller_request, 0, 2000
    );
  if (!controller_response || !controller_response->ok) {
    RCLCPP_ERROR(get_logger(), "Could not start joint state broadcaster");
    // TODO(Svastits): deactivate HW interface
    is_configured_msg_.data = false;
    is_configured_pub_->publish(is_configured_msg_);
    return FAILURE;
  }

  auto control_mode = this->get_parameter("control_mode").as_string();

  if (control_mode_map_.find(control_mode) == control_mode_map_.end()) {
    RCLCPP_ERROR(
      get_logger(), "Not valid control mode, control mode set to: %s",
      control_mode.c_str());
    return ERROR;
  }
  controller_names_ = control_mode_map_.at(control_mode);

  // Activate RT commander
  controller_request->strictness = SwitchController::Request::STRICT;
  controller_request->activate_controllers = controller_names_;
  controller_response =
    kroshu_ros2_core::sendRequest<SwitchController::Response>(
    change_controller_state_client_, controller_request, 0, 2000
    );
  if (!controller_response || !controller_response->ok) {
    RCLCPP_ERROR(get_logger(), "Could not  activate controller");
    // TODO(Svastits): deactivate HW interface
    is_configured_msg_.data = false;
    is_configured_pub_->publish(is_configured_msg_);
    return FAILURE;
  }

  return SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
RobotManagerNode::on_deactivate(const rclcpp_lifecycle::State &)
{
  // Deactivate hardware interface
  auto hw_request =
    std::make_shared<SetHardwareComponentState::Request>();
  hw_request->name = "iisy_hardware";
  hw_request->target_state.label = "inactive";
  auto hw_response =
    kroshu_ros2_core::sendRequest<SetHardwareComponentState::Response>(
    change_hardware_state_client_, hw_request, 0, 2000);
  if (!hw_response || !hw_response->ok) {
    RCLCPP_ERROR(get_logger(), "Could not deactivate hardware interface");
    return ERROR;
  }

  RCLCPP_INFO(get_logger(), "Deactivated LBR iisy hardware interface");

  // Stop RT controllers
  controller_names_.emplace_back("joint_state_broadcaster");
  auto controller_request =
    std::make_shared<SwitchController::Request>();
  // With best effort strictness, deactivation succeeds if specific controller is not active
  controller_request->strictness =
    SwitchController::Request::BEST_EFFORT;
  controller_request->deactivate_controllers = controller_names_;
  auto controller_response =
    kroshu_ros2_core::sendRequest<SwitchController::Response>(
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
  if (control_mode_map_.find(control_mode) != control_mode_map_.end()) {
    RCLCPP_INFO(get_logger(), "Control mode changed to %s", control_mode.c_str());
    return true;
  } else {
    RCLCPP_WARN(
      get_logger(), "Could not change control mode, %s is not a valid control mode",
      control_mode.c_str());
    return false;
  }
}

bool RobotManagerNode::onControllerNameChangeRequest(
  const std::string & controller_name,
  const std::string & controller_name_param)
{
  try {
    if (controller_name_param == POSITION_CONTROLLER_NAME_PARAM) {
      control_mode_map_.at(POSITION_CONTROL).at(0) = controller_name;
      control_mode_map_.at(IMPEDANCE_CONTROL).at(0) = controller_name;
    } else if (controller_name_param == IMPEDANCE_CONTROLLER_NAME_PARAM) {
      control_mode_map_.at(IMPEDANCE_CONTROL).at(1) = controller_name;
    } else if (controller_name_param == TORQUE_CONTROLLER_NAME_PARAM) {
      control_mode_map_.at(TORQUE_CONTROL).at(0) = controller_name;
    } else {
      RCLCPP_WARN(get_logger(), "Unknown controller name param added.");
      return false;
    }
    RCLCPP_INFO(
      get_logger(), "The %s parameter changed to %s.",
      controller_name_param.c_str(), controller_name.c_str()
    );
  } catch (const std::out_of_range & e) {
    RCLCPP_ERROR(get_logger(), "Out of range error: %s", e.what());
  }
  return true;
}
}  // namespace kuka_rox

int main(int argc, char * argv[])
{
  setvbuf(stdout, nullptr, _IONBF, BUFSIZ);

  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor;
  auto node = std::make_shared<kuka_rox::RobotManagerNode>();
  executor.add_node(node->get_node_base_interface());
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
