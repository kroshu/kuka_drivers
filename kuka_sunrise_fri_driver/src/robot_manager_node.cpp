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

#include "communication_helpers/ros2_control_tools.hpp"
#include "communication_helpers/service_tools.hpp"
#include "kuka_drivers_core/controller_names.hpp"
#include "kuka_drivers_core/hardware_event.hpp"

#include "kuka_sunrise_fri_driver/robot_manager_node.hpp"

using namespace controller_manager_msgs::srv;  // NOLINT

namespace kuka_sunrise_fri_driver
{
RobotManagerNode::RobotManagerNode() : kuka_drivers_core::ROS2BaseLCNode("robot_manager")
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

  auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
  qos.reliable();
  cbg_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  event_cbg_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  change_hardware_state_client_ = this->create_client<SetHardwareComponentState>(
    "controller_manager/set_hardware_component_state", qos.get_rmw_qos_profile(), cbg_);
  change_controller_state_client_ = this->create_client<SwitchController>(
    "controller_manager/switch_controller", qos.get_rmw_qos_profile(), cbg_);

  auto is_configured_qos = rclcpp::QoS(rclcpp::KeepLast(1));
  is_configured_qos.best_effort();
  is_configured_pub_ =
    this->create_publisher<std_msgs::msg::Bool>("robot_manager/is_configured", is_configured_qos);

  fri_config_pub_ = this->create_publisher<kuka_driver_interfaces::msg::FriConfiguration>(
    "fri_configuration_controller/set_fri_config", rclcpp::SystemDefaultsQoS());

  control_mode_pub_ = this->create_publisher<std_msgs::msg::UInt32>(
    "control_mode_handler/control_mode", rclcpp::SystemDefaultsQoS());

  joint_imp_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
    "joint_group_impedance_controller/commands", rclcpp::SystemDefaultsQoS());

  rclcpp::SubscriptionOptions sub_options;
  sub_options.callback_group = event_cbg_;
  event_subscriber_ = this->create_subscription<std_msgs::msg::UInt8>(
    "event_broadcaster/hardware_event", rclcpp::SystemDefaultsQoS(),
    [this](const std_msgs::msg::UInt8::SharedPtr msg) { this->EventSubscriptionCallback(msg); },
    sub_options);

  registerStaticParameter<std::string>(
    "robot_model", "lbr_iiwa14_r820", kuka_drivers_core::ParameterSetAccessRights{false, false},
    [this](const std::string & robot_model)
    { return this->onRobotModelChangeRequest(robot_model); });

  registerStaticParameter<std::string>(
    "controller_ip", "", kuka_drivers_core::ParameterSetAccessRights{false, false},
    [this](const std::string & controller_ip) { return this->ValidateIPAdress(controller_ip); });

  registerParameter<int>(
    "send_period_ms", 10, kuka_drivers_core::ParameterSetAccessRights{true, false},
    [this](const int & send_period) { return this->onSendPeriodChangeRequest(send_period); });

  registerParameter<int>(
    "receive_multiplier", 1, kuka_drivers_core::ParameterSetAccessRights{true, false},
    [this](const int & receive_multiplier)
    { return this->onReceiveMultiplierChangeRequest(receive_multiplier); });

  registerParameter<int>(
    "control_mode", static_cast<int>(kuka_drivers_core::ControlMode::JOINT_POSITION_CONTROL),
    kuka_drivers_core::ParameterSetAccessRights{true, false},
    [this](int control_mode) { return this->onControlModeChangeRequest(control_mode); });

  registerParameter<std::string>(
    "position_controller_name", "", kuka_drivers_core::ParameterSetAccessRights{true, false},
    [this](const std::string & controller_name)
    {
      return this->onControllerNameChangeRequest(
        controller_name, kuka_drivers_core::ControllerType::JOINT_POSITION_CONTROLLER_TYPE);
    });

  registerParameter<std::string>(
    "torque_controller_name", "", kuka_drivers_core::ParameterSetAccessRights{true, false},
    [this](const std::string & controller_name)
    {
      return this->onControllerNameChangeRequest(
        controller_name, kuka_drivers_core::ControllerType::TORQUE_CONTROLLER_TYPE);
    });

  registerParameter<std::vector<double>>(
    "joint_stiffness", joint_stiffness_, kuka_drivers_core::ParameterSetAccessRights{true, false},
    [this](const std::vector<double> & joint_stiffness)
    { return this->onJointStiffnessChangeRequest(joint_stiffness); });

  registerParameter<std::vector<double>>(
    "joint_damping", joint_damping_, kuka_drivers_core::ParameterSetAccessRights{true, false},
    [this](const std::vector<double> & joint_damping)
    { return this->onJointDampingChangeRequest(joint_damping); });
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
RobotManagerNode::on_configure(const rclcpp_lifecycle::State &)
{
  // Publish control mode parameter to notify control_mode_handler of initial control mode
  control_mode_pub_->publish(control_mode_msg_);

  // Publish FRI configuration to notify fri_configuration_controller of initial values
  setFriConfiguration(send_period_ms_, receive_multiplier_);

  // Configure hardware interface
  if (!kuka_drivers_core::changeHardwareState(
        change_hardware_state_client_, robot_model_,
        lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE))
  {
    RCLCPP_ERROR(get_logger(), "Could not configure hardware interface");
    return FAILURE;
  }

  // Start non-RT controllers
  if (!kuka_drivers_core::changeControllerState(
        change_controller_state_client_,
        {kuka_drivers_core::FRI_CONFIGURATION_CONTROLLER, kuka_drivers_core::CONTROL_MODE_HANDLER,
         kuka_drivers_core::EVENT_BROADCASTER, kuka_drivers_core::JOINT_GROUP_IMPEDANCE_CONTROLLER},
        {}))
  {
    RCLCPP_ERROR(get_logger(), "Could not activate configuration controllers");
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
  // Stop non-RT controllers
  // With best effort strictness, cleanup succeeds if specific controller is not active
  if (!kuka_drivers_core::changeControllerState(
        change_controller_state_client_, {},
        {kuka_drivers_core::FRI_CONFIGURATION_CONTROLLER, kuka_drivers_core::CONTROL_MODE_HANDLER,
         kuka_drivers_core::EVENT_BROADCASTER, kuka_drivers_core::JOINT_GROUP_IMPEDANCE_CONTROLLER},
        SwitchController::Request::BEST_EFFORT))
  {
    RCLCPP_ERROR(get_logger(), "Could not stop controllers");
    return ERROR;
  }

  // Cleanup hardware interface
  // If it is inactive, cleanup will also succeed
  if (!kuka_drivers_core::changeHardwareState(
        change_hardware_state_client_, robot_model_,
        lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED))
  {
    RCLCPP_ERROR(get_logger(), "Could not clean up hardware interface");
    return FAILURE;
  }

  if (is_configured_pub_->is_activated())
  {
    is_configured_msg_.data = false;
    is_configured_pub_->publish(is_configured_msg_);
    is_configured_pub_->on_deactivate();
  }

  return SUCCESS;
}

// TODO(Svastits): can we check if necessary 5s has passed after deactivation?
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
RobotManagerNode::on_activate(const rclcpp_lifecycle::State &)
{
  // Publish the values of the joint impedance parameters to the controller
  std_msgs::msg::Float64MultiArray joint_imp_msg;
  for (int i = 0; i < 7; i++)
  {
    joint_imp_msg.data.push_back(joint_stiffness_[i]);
    joint_imp_msg.data.push_back(joint_damping_[i]);
  }
  joint_imp_pub_->publish(joint_imp_msg);

  // Activate hardware interface
  if (!kuka_drivers_core::changeHardwareState(
        change_hardware_state_client_, robot_model_,
        lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE))
  {
    RCLCPP_ERROR(get_logger(), "Could not activate hardware interface");
    return FAILURE;
  }

  // Workaround until controller_manager/jtc bug is fixed:
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  // Activate joint state broadcaster and controller for given control mode
  if (!kuka_drivers_core::changeControllerState(
        change_controller_state_client_,
        {kuka_drivers_core::JOINT_STATE_BROADCASTER, kuka_drivers_core::FRI_STATE_BROADCASTER,
         GetControllerName()},
        {kuka_drivers_core::JOINT_GROUP_IMPEDANCE_CONTROLLER}))
  {
    RCLCPP_ERROR(get_logger(), "Could not activate RT controllers");
    this->on_deactivate(get_current_state());
    return FAILURE;
  }
  return SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
RobotManagerNode::on_deactivate(const rclcpp_lifecycle::State &)
{
  // Deactivate hardware interface
  // If it is inactive, deactivation will also succeed
  if (!kuka_drivers_core::changeHardwareState(
        change_hardware_state_client_, robot_model_,
        lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE))
  {
    RCLCPP_ERROR(get_logger(), "Could not deactivate hardware interface");
    return ERROR;
  }

  // Stop RT controllers
  // With best effort strictness, deactivation succeeds if specific controller is not active
  if (!kuka_drivers_core::changeControllerState(
        change_controller_state_client_, {kuka_drivers_core::JOINT_GROUP_IMPEDANCE_CONTROLLER},
        {GetControllerName(), kuka_drivers_core::JOINT_STATE_BROADCASTER,
         kuka_drivers_core::FRI_STATE_BROADCASTER},
        SwitchController::Request::BEST_EFFORT))
  {
    RCLCPP_ERROR(get_logger(), "Could not deactivate RT controllers");
    return ERROR;
  }

  RCLCPP_INFO(
    get_logger(), "Successfully deactivated driver, reactivation is possible after 5 seconds");
  return SUCCESS;
}

bool RobotManagerNode::onRobotModelChangeRequest(const std::string & robot_model)
{
  auto ns = std::string(get_namespace());
  // Remove '/' from namespace (even empty namespace contains one '/')
  ns.erase(ns.begin());

  // Add '_' to prefix
  if (ns.size() > 0)
  {
    ns += "_";
  }
  robot_model_ = ns + robot_model;
  return true;
}

bool RobotManagerNode::onControlModeChangeRequest(int control_mode)
{
  switch (static_cast<kuka_drivers_core::ControlMode>(control_mode))
  {
    case kuka_drivers_core::ControlMode::JOINT_POSITION_CONTROL:
      break;
    case kuka_drivers_core::ControlMode::JOINT_IMPEDANCE_CONTROL:
      // TODO(Svastits): check whether this is necessary for impedance mode too
      [[fallthrough]];
    case kuka_drivers_core::ControlMode::JOINT_TORQUE_CONTROL:
      if (send_period_ms_ > 5)
      {
        RCLCPP_ERROR(
          get_logger(),
          "Unable to set non-position control mode, if send period is bigger than 5 [ms]");
        return false;
      }
      break;
    default:
      RCLCPP_ERROR(get_logger(), "Tried to change to a not implemented control mode");
      return false;
  }

  // Publish the control mode to controller handler
  control_mode_msg_.data = control_mode;
  control_mode_pub_->publish(control_mode_msg_);
  RCLCPP_INFO(get_logger(), "Control mode change successful");

  return true;
}

bool RobotManagerNode::onSendPeriodChangeRequest(int send_period)
{
  if (send_period < 1 || send_period > 100)
  {
    RCLCPP_ERROR(get_logger(), "Send period milliseconds must be >=1 && <=100");
    return false;
  }

  if (send_period * receive_multiplier_ > 10)
  {
    RCLCPP_ERROR(get_logger(), "Control signal send period must not be bigger than 10 ms");
    return false;
  }

  send_period_ms_ = send_period;
  setFriConfiguration(send_period_ms_, receive_multiplier_);
  return true;
}

bool RobotManagerNode::onReceiveMultiplierChangeRequest(const int & receive_multiplier)
{
  if (receive_multiplier < 1)
  {
    RCLCPP_ERROR(get_logger(), "Receive multiplier must be >=1");
    return false;
  }

  if (receive_multiplier * send_period_ms_ > 10)
  {
    RCLCPP_ERROR(get_logger(), "Control signal send period must be bigger than 10 ms");
    return false;
  }

  receive_multiplier_ = receive_multiplier;
  setFriConfiguration(send_period_ms_, receive_multiplier_);
  return true;
}

bool RobotManagerNode::ValidateIPAdress(std::string_view controller_ip) const
{
  // Check IP validity
  size_t i = 0;
  std::vector<std::string> split_ip;
  auto pos = controller_ip.find('.');
  while (pos != std::string_view::npos)
  {
    split_ip.emplace_back(controller_ip.substr(i, pos - i));
    i = ++pos;
    pos = controller_ip.find('.', pos);
  }
  split_ip.emplace_back(controller_ip.substr(i, controller_ip.length()));

  if (split_ip.size() != 4)
  {
    RCLCPP_ERROR(get_logger(), "Valid IP must have 3 '.' delimiters");
    return false;
  }

  for (const auto & ip : split_ip)
  {
    if (
      ip.empty() || (ip.find_first_not_of("[0123456789]") != std::string::npos) || stoi(ip) > 255 ||
      stoi(ip) < 0)
    {
      RCLCPP_ERROR(get_logger(), "Valid IP must contain only numbers between 0 and 255");
      return false;
    }
  }
  return true;
}

bool RobotManagerNode::onControllerNameChangeRequest(
  std::string_view controller_name, kuka_drivers_core::ControllerType controller_type)
{
  switch (controller_type)
  {
    case kuka_drivers_core::ControllerType::JOINT_POSITION_CONTROLLER_TYPE:
      joint_pos_controller_name_ = controller_name;
      break;
    case kuka_drivers_core::ControllerType::TORQUE_CONTROLLER_TYPE:
      joint_torque_controller_name_ = controller_name;
      break;
    default:
      // This should actually never happen
      RCLCPP_ERROR(get_logger(), "Invalid controller type");
      return false;
  }
  return true;
}

// the joint impedannce attributes cannot be modified in FRI after activation, therefore only one
// controller controls in each control mode
std::string RobotManagerNode::GetControllerName() const
{
  switch (static_cast<kuka_drivers_core::ControlMode>(control_mode_msg_.data))
  {
    case kuka_drivers_core::ControlMode::JOINT_POSITION_CONTROL:
      return joint_pos_controller_name_;
    case kuka_drivers_core::ControlMode::JOINT_IMPEDANCE_CONTROL:
      return joint_pos_controller_name_;
    case kuka_drivers_core::ControlMode::JOINT_TORQUE_CONTROL:
      return joint_torque_controller_name_;
    default:
      throw std::runtime_error("Stored control mode is not allowed");
  }
}

void RobotManagerNode::setFriConfiguration(int send_period_ms, int receive_multiplier) const
{
  kuka_driver_interfaces::msg::FriConfiguration msg;
  msg.receive_multiplier = receive_multiplier;
  msg.send_period_ms = send_period_ms;
  fri_config_pub_->publish(msg);
}

bool RobotManagerNode::onJointStiffnessChangeRequest(const std::vector<double> & joint_stiffness)
{
  if (joint_stiffness.size() != 7)
  {
    RCLCPP_ERROR(get_logger(), "Invalid parameter array length for parameter joint stiffness");
    return false;
  }
  for (double js : joint_stiffness)
  {
    if (js < 0)
    {
      RCLCPP_ERROR(get_logger(), "Joint stiffness values must be >=0");
      return false;
    }
  }
  joint_stiffness_ = joint_stiffness;
  return true;
}

bool RobotManagerNode::onJointDampingChangeRequest(const std::vector<double> & joint_damping)
{
  if (joint_damping.size() != 7)
  {
    RCLCPP_ERROR(get_logger(), "Invalid parameter array length for parameter joint damping");
    return false;
  }
  for (double jd : joint_damping)
  {
    if (jd < 0 || jd > 1)
    {
      RCLCPP_ERROR(get_logger(), "Joint damping values must be >=0 && <=1");
      return false;
    }
  }
  joint_damping_ = joint_damping;
  return true;
}

void RobotManagerNode::EventSubscriptionCallback(const std_msgs::msg::UInt8::SharedPtr msg)
{
  switch (static_cast<kuka_drivers_core::HardwareEvent>(msg->data))
  {
    case kuka_drivers_core::HardwareEvent::ERROR:
      RCLCPP_INFO(get_logger(), "External control stopped");
      if (this->get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
      {
        this->deactivate();
      }
      else if (
        this->get_current_state().id() == lifecycle_msgs::msg::State::TRANSITION_STATE_ACTIVATING)
      {
        // Handle case, when error is received while still activating
        this->on_deactivate(get_current_state());
      }
      break;
    default:
      break;
  }
}

}  // namespace kuka_sunrise_fri_driver

int main(int argc, char * argv[])
{
  setvbuf(stdout, nullptr, _IONBF, BUFSIZ);

  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor;
  auto node = std::make_shared<kuka_sunrise_fri_driver::RobotManagerNode>();
  executor.add_node(node->get_node_base_interface());
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
