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

#include <grpcpp/create_channel.h>

#include "communication_helpers/ros2_control_tools.hpp"
#include "communication_helpers/service_tools.hpp"

#include "kuka_iiqka_eac_driver/robot_manager_node.hpp"

using namespace controller_manager_msgs::srv;  // NOLINT
using namespace lifecycle_msgs::msg;           // NOLINT
using namespace kuka::motion::external;        // NOLINT

namespace kuka_eac
{
// TODO(Komaromi): Re-add "control_mode_handler" controller to controller_handlers constructor
// after controller handler properly implemented with working initial control mode change
RobotManagerNode::RobotManagerNode()
: kuka_drivers_core::ROS2BaseLCNode("robot_manager"),
  controller_handler_({
    "joint_state_broadcaster",
  })
#ifdef NON_MOCK_SETUP
  ,
  control_mode_change_finished_(false)
#endif
{
  RCLCPP_DEBUG(get_logger(), "Starting Robot Manager Node init");

  auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
  qos.reliable();
  cbg_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  change_hardware_state_client_ = this->create_client<SetHardwareComponentState>(
    "controller_manager/set_hardware_component_state", qos.get_rmw_qos_profile(), cbg_);
  change_controller_state_client_ = this->create_client<SwitchController>(
    "controller_manager/switch_controller", qos.get_rmw_qos_profile(), cbg_);

  auto is_configured_qos = rclcpp::QoS(rclcpp::KeepLast(1));
  is_configured_qos.best_effort();

  is_configured_pub_ =
    this->create_publisher<std_msgs::msg::Bool>("robot_manager/is_configured", is_configured_qos);

  control_mode_pub_ = this->create_publisher<std_msgs::msg::UInt32>(
    "control_mode_handler/control_mode", rclcpp::SystemDefaultsQoS());

  // Register parameters
  this->registerParameter<std::string>(
    "position_controller_name", "joint_trajectory_controller",
    kuka_drivers_core::ParameterSetAccessRights{true, true, false},
    [this](const std::string & controller_name)
    {
      return this->controller_handler_.UpdateControllerName(
        kuka_drivers_core::ControllerType::JOINT_POSITION_CONTROLLER_TYPE, controller_name);
    });
  this->registerParameter<std::string>(
    "impedance_controller_name", "joint_group_impedance_controllers",
    kuka_drivers_core::ParameterSetAccessRights{true, true, false},
    [this](const std::string & controller_name)
    {
      return this->controller_handler_.UpdateControllerName(
        kuka_drivers_core::ControllerType::JOINT_IMPEDANCE_CONTROLLER_TYPE, controller_name);
    });
  this->registerParameter<std::string>(
    "torque_controller_name", "effort_controller",
    kuka_drivers_core::ParameterSetAccessRights{true, true, false},
    [this](const std::string & controller_name)
    {
      return this->controller_handler_.UpdateControllerName(
        kuka_drivers_core::ControllerType::TORQUE_CONTROLLER_TYPE, controller_name);
    });
  this->registerParameter<int>(
    "control_mode", static_cast<int>(ExternalControlMode::JOINT_POSITION_CONTROL),
    kuka_drivers_core::ParameterSetAccessRights{true, true, true},
    [this](int control_mode) { return this->onControlModeChangeRequest(control_mode); });
  this->registerStaticParameter<std::string>(
    "controller_ip", "", kuka_drivers_core::ParameterSetAccessRights{true, false, false},
    [this](const std::string &) { return true; });
  this->registerStaticParameter<std::string>(
    "robot_model", "lbr_iisy3_r760",
    kuka_drivers_core::ParameterSetAccessRights{true, false, false},
    [this](const std::string & robot_model)
    { return this->onRobotModelChangeRequest(robot_model); });

#ifdef NON_MOCK_SETUP
  RCLCPP_INFO(
    get_logger(), "IP address of controller: %s",
    this->get_parameter("controller_ip").as_string().c_str());

  stub_ = kuka::ecs::v1::ExternalControlService::NewStub(grpc::CreateChannel(
    this->get_parameter("controller_ip").as_string() + ":49335",
    grpc::InsecureChannelCredentials()));
#endif
}
RobotManagerNode::~RobotManagerNode()
{
#ifdef NON_MOCK_SETUP
  if (context_ != nullptr)
  {
    context_->TryCancel();
  }
#endif
  if (observe_thread_.joinable())
  {
    observe_thread_.join();
  }
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
RobotManagerNode::on_configure(const rclcpp_lifecycle::State &)
{
  // Publish control mode parameter to notify control_mode_handler of initial control mode
  auto message = std_msgs::msg::UInt32();
  message.data = static_cast<uint32_t>(this->get_parameter("control_mode").as_int());
  control_mode_pub_->publish(message);

  // Configure hardware interface
  if (!kuka_drivers_core::changeHardwareState(
        change_hardware_state_client_, robot_model_, State::PRIMARY_STATE_INACTIVE))
  {
    RCLCPP_ERROR(get_logger(), "Could not configure hardware interface");
    return FAILURE;
  }

  is_configured_pub_->on_activate();
  is_configured_msg_.data = true;
  is_configured_pub_->publish(is_configured_msg_);

  // Activate control mode handler
  if (!kuka_drivers_core::changeControllerState(
        change_controller_state_client_, {"control_mode_handler"}, {}))
  {
    RCLCPP_ERROR(get_logger(), "Could not activate control mode handler");
    // TODO(Svastits): this can be removed if rollback is implemented properly
    this->on_cleanup(get_current_state());
    return FAILURE;
  }

  RCLCPP_INFO(get_logger(), "Activated control mode handler");

  return SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
RobotManagerNode::on_cleanup(const rclcpp_lifecycle::State &)
{
  // Deactivate control mode handler
  if (!kuka_drivers_core::changeControllerState(
        change_controller_state_client_, {}, {"control_mode_handler"}))
  {
    RCLCPP_ERROR(get_logger(), "Could not deactivate control mode handler");
  }

  // Clean up hardware interface
  if (!kuka_drivers_core::changeHardwareState(
        change_hardware_state_client_, robot_model_, State::PRIMARY_STATE_UNCONFIGURED))
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
  // TODO(Svastits): add else branch, and throw exception(?)
  return SUCCESS;
}

void RobotManagerNode::ObserveControl()
{
#ifdef NON_MOCK_SETUP
  context_ = std::make_unique<::grpc::ClientContext>();
  kuka::ecs::v1::ObserveControlStateRequest observe_request;
  std::unique_ptr<grpc::ClientReader<kuka::ecs::v1::CommandState>> reader(
    stub_->ObserveControlState(context_.get(), observe_request));

  kuka::ecs::v1::CommandState response;

  while (reader->Read(&response))
  {
    switch (static_cast<int>(response.event()))
    {
      case kuka::ecs::v1::CommandEvent::CONTROL_MODE_SWITCH:
      {
        std::lock_guard<std::mutex> lk(control_mode_cv_m_);
        control_mode_change_finished_ = true;
      }
        RCLCPP_INFO(get_logger(), "Command mode switched in the robot controller");
        control_mode_cv_.notify_all();
        break;
      case kuka::ecs::v1::CommandEvent::STOPPED:
      case kuka::ecs::v1::CommandEvent::ERROR:
        RCLCPP_INFO(get_logger(), "External control stopped");
        terminate_ = true;
        if (this->get_current_state().id() == State::PRIMARY_STATE_ACTIVE)
        {
          this->deactivate();
        }
        else if (this->get_current_state().id() == State::TRANSITION_STATE_ACTIVATING)
        {
          // TODO(Svastits): this can be removed if rollback is implemented properly
          this->on_deactivate(get_current_state());
        }
        break;
      default:
        break;
    }
  }
#endif
}

// TODO(Svastits): rollback in case of failures
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
RobotManagerNode::on_activate(const rclcpp_lifecycle::State &)
{
#ifdef NON_MOCK_SETUP
  if (context_ != nullptr)
  {
    context_->TryCancel();
  }
#endif
  // Join observe thread, necessary if previous activation failed
  if (observe_thread_.joinable())
  {
    observe_thread_.join();
  }
  terminate_ = false;
  // Subscribe to stream of state changes
  observe_thread_ = std::thread(&RobotManagerNode::ObserveControl, this);

  // Activate hardware interface
  if (!kuka_drivers_core::changeHardwareState(
        change_hardware_state_client_, robot_model_, State::PRIMARY_STATE_ACTIVE, 5000))
  {
    RCLCPP_ERROR(get_logger(), "Could not activate hardware interface");
    return FAILURE;
  }

  // Select controllers
  auto control_mode = this->get_parameter("control_mode").as_int();
  std::pair<std::vector<std::string>, std::vector<std::string>> new_controllers;

  try
  {
    new_controllers =
      controller_handler_.GetControllersForSwitch(kuka_drivers_core::ControlMode(control_mode));
  }
  catch (const std::exception & e)
  {
    RCLCPP_ERROR(get_logger(), "Error while activating controllers: %s", e.what());
    return ERROR;
  }

  // Deactivate list for activation should always be empty, safety check
  if (!new_controllers.second.empty())
  {
    RCLCPP_ERROR(
      get_logger(),
      "Controller handler state is improper, active controller list not empty before activation");
    return FAILURE;
  }

  // Activate RT controller(s)
  if (!kuka_drivers_core::changeControllerState(
        change_controller_state_client_, new_controllers.first, new_controllers.second))
  {
    RCLCPP_ERROR(get_logger(), "Could not activate RT controllers");
    this->on_deactivate(get_current_state());
    return FAILURE;
  }

  controller_handler_.ApproveControllerActivation();
  if (!controller_handler_.ApproveControllerDeactivation())
  {
    RCLCPP_ERROR(
      get_logger(),
      "Controller handler state is improper, active controller list was modified before approval");
  }

  RCLCPP_INFO(get_logger(), "Successfully activated controllers");

  // Return failure if control is stopped while in state activating
  if (terminate_)
  {
    RCLCPP_ERROR(get_logger(), "UDP communication could not be set up");
    return FAILURE;
  }

  return SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
RobotManagerNode::on_deactivate(const rclcpp_lifecycle::State &)
{
  // Deactivate hardware interface
  // Deactivation was not stable with 2000 ms timeout
  if (!kuka_drivers_core::changeHardwareState(
        change_hardware_state_client_, robot_model_, State::PRIMARY_STATE_INACTIVE, 3000))
  {
    RCLCPP_ERROR(get_logger(), "Could not deactivate hardware interface");
    return ERROR;
  }

  // Stop RT controllers
  if (!kuka_drivers_core::changeControllerState(
        change_controller_state_client_, {}, controller_handler_.GetControllersForDeactivation(),
        SwitchController::Request::BEST_EFFORT))
  {
    RCLCPP_ERROR(get_logger(), "Could not stop RT controllers");
    return ERROR;
  }

  if (!controller_handler_.ApproveControllerDeactivation())
  {
    RCLCPP_ERROR(
      get_logger(),
      "Controller handler state is improper, active controller list was modified before approval");
  }

  RCLCPP_INFO(get_logger(), "Successfully stopped controllers");
  return SUCCESS;
}

bool RobotManagerNode::onControlModeChangeRequest(int control_mode)
{
  if (param_declared_ && this->get_parameter("control_mode").as_int() == control_mode)
  {
    RCLCPP_WARN(get_logger(), "Tried to change control mode to the one currently used");
    return true;
  }

  RCLCPP_INFO(get_logger(), "Control mode change requested");
  if (
    control_mode == static_cast<int>(kuka_drivers_core::ControlMode::CARTESIAN_POSITION_CONTROL) ||
    control_mode == static_cast<int>(kuka_drivers_core::ControlMode::CARTESIAN_IMPEDANCE_CONTROL) ||
    control_mode == static_cast<int>(kuka_drivers_core::ControlMode::WRENCH_CONTROL) ||
    control_mode == static_cast<int>(kuka_drivers_core::ControlMode::JOINT_VELOCITY_CONTROL) ||
    control_mode == static_cast<int>(kuka_drivers_core::ControlMode::CARTESIAN_VELOCITY_CONTROL))
  {
    RCLCPP_ERROR(get_logger(), "Tried to change to a not implemented control mode");
    return false;
  }

  std::pair<std::vector<std::string>, std::vector<std::string>> switch_controllers;

  bool is_active_state =
    get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE;

  // Determine which controllers to activate and deactivate
  try
  {
    switch_controllers =
      controller_handler_.GetControllersForSwitch(kuka_drivers_core::ControlMode(control_mode));
  }
  catch (const std::exception & e)
  {
    RCLCPP_ERROR(get_logger(), "Error while control mode change: %s", e.what());
    return false;
  }

  // Activate controllers needed for the new control mode
  if (is_active_state)
  {
    if (
      !switch_controllers.first.empty() &&
      !kuka_drivers_core::changeControllerState(
        change_controller_state_client_, switch_controllers.first, {}))
    {
      RCLCPP_ERROR(get_logger(), "Could not activate controllers for new control mode");
      // TODO(Svastits): this can be removed if rollback is implemented properly
      this->on_deactivate(get_current_state());
      return false;
    }
    controller_handler_.ApproveControllerActivation();
  }

  // Publish the control mode to controller handler
  auto message = std_msgs::msg::UInt32();
  message.data = control_mode;
  control_mode_pub_->publish(message);
  RCLCPP_INFO(get_logger(), "Control mode change process has started");

  if (is_active_state)
  {
    // The driver is in active state

#ifdef NON_MOCK_SETUP
    // Wait for ObserveControl to approve that the robot succefully changed control mode
    std::unique_lock<std::mutex> control_mode_lk(this->control_mode_cv_m_);

    if (!this->control_mode_cv_.wait_for(
          control_mode_lk, std::chrono::milliseconds(2000),
          [this]() { return this->control_mode_change_finished_; }))
    {
      // Control Mode change timeout reached
      RCLCPP_ERROR(get_logger(), "Timeout reached while waiting for robot to change control mode.");
      this->on_deactivate(get_current_state());
      return false;
    }
    control_mode_change_finished_ = false;
    control_mode_lk.unlock();
    RCLCPP_INFO(get_logger(), "Robot Controller finished control mode change");
#endif

    // Deactivate unnecessary controllers
    if (
      !switch_controllers.second.empty() &&
      !kuka_drivers_core::changeControllerState(
        change_controller_state_client_, {}, switch_controllers.second))
    {
      RCLCPP_ERROR(get_logger(), "Could not deactivate controllers for new control mode");
      // TODO(Svastits): this can be removed if rollback is implemented properly
      this->on_deactivate(get_current_state());
      return false;
    }
    if (!controller_handler_.ApproveControllerDeactivation())
    {
      RCLCPP_ERROR(
        get_logger(),
        "Controller handler state is improper, active controller list was modified"
        "before approval");
    }
  }

  RCLCPP_INFO(
    get_logger(), "Successfully changed control mode to %s",
    ExternalControlMode_Name(control_mode).c_str());
  param_declared_ = true;
  return true;
}

bool RobotManagerNode::onRobotModelChangeRequest(const std::string & robot_model)
{
  auto ns = std::string(this->get_namespace());
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
}  // namespace kuka_eac

int main(int argc, char * argv[])
{
  setvbuf(stdout, nullptr, _IONBF, BUFSIZ);

  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor;
  auto node = std::make_shared<kuka_eac::RobotManagerNode>();
  executor.add_node(node->get_node_base_interface());
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
