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


#include <grpcpp/create_channel.h>

using namespace controller_manager_msgs::srv;  // NOLINT
using namespace lifecycle_msgs::msg;  // NOLINT
using namespace kuka::motion::external;  // NOLINT

namespace kuka_rox
{
RobotManagerNode::RobotManagerNode()
: kroshu_ros2_core::ROS2BaseLCNode("robot_manager"), controller_handler_({"joint_state_broadcaster",
      "control_mode_handler"})
#ifdef NON_MOCK_SETUP
  , control_mode_change_finished_(false)
#endif
{
  RCLCPP_INFO(get_logger(), "Starting Robot Manager Node init");

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

  control_mode_pub_ = this->create_publisher<std_msgs::msg::UInt32>(
    "control_mode", rclcpp::SystemDefaultsQoS()
  );

  // Register parameters
  this->registerParameter<int>(
    "control_mode", static_cast<int>(ExternalControlMode::POSITION_CONTROL),
    kroshu_ros2_core::ParameterSetAccessRights{true, true,
      true, false, false}, [this](int control_mode) {
      return this->onControlModeChangeRequest(control_mode);
    });
  this->registerStaticParameter<std::string>(
    "robot_model", "LBRiisy3R760",
    kroshu_ros2_core::ParameterSetAccessRights{true, false,
      false, false, false}, [this](const std::string & robot_model) {
      return this->onRobotModelChangeRequest(robot_model);
    });
  this->registerParameter<std::string>(
    "position_controller_name", "", kroshu_ros2_core::ParameterSetAccessRights {true, true,
      false, false, false}, [this](const std::string & controller_name) {
      return this->controller_handler_.UpdateControllerName(
        kroshu_ros2_core::ControllerType::JOINT_POSITION_CONTROLLER_TYPE,
        controller_name);
    });
  this->registerParameter<std::string>(
    "impedance_controller_name", "", kroshu_ros2_core::ParameterSetAccessRights {true, true,
      false, false, false}, [this](const std::string & controller_name) {
      return this->controller_handler_.UpdateControllerName(
        kroshu_ros2_core::ControllerType::JOINT_IMPEDANCE_CONTROLLER_TYPE,
        controller_name);
    });
  this->registerParameter<std::string>(
    "torque_controller_name", "", kroshu_ros2_core::ParameterSetAccessRights {true, true, false,
      false, false}, [this](const std::string & controller_name) {
      return this->controller_handler_.UpdateControllerName(
        kroshu_ros2_core::ControllerType::TORQUE_CONTROLLER_TYPE,
        controller_name);
    });
  this->registerStaticParameter<std::string>(
    "controller_ip", "", kroshu_ros2_core::ParameterSetAccessRights {true, false, false,
      false, false}, [this](const std::string &) {
      return true;
    });

#ifdef NON_MOCK_SETUP
  RCLCPP_INFO(
    get_logger(), "IP address of controller: %s", this->get_parameter(
      "controller_ip").as_string().c_str());

  stub_ =
    kuka::ecs::v1::ExternalControlService::NewStub(
    grpc::CreateChannel(
      this->get_parameter("controller_ip").as_string() + ":49335",
      grpc::InsecureChannelCredentials()));
#endif
}
RobotManagerNode::~RobotManagerNode()
{
#ifdef NON_MOCK_SETUP
  if (context_ != nullptr) {
    context_->TryCancel();
  }
#endif
  if (observe_thread_.joinable()) {
    observe_thread_.join();
  }
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
RobotManagerNode::on_configure(const rclcpp_lifecycle::State &)
{
  // Publish control mode paramater
  auto message = std_msgs::msg::UInt32();
  message.data = static_cast<uint32_t>(this->get_parameter("control_mode").as_int());
  control_mode_pub_->publish(message);

  // Configure hardware interface
  auto hw_request =
    std::make_shared<SetHardwareComponentState::Request>();
  hw_request->name = robot_model_;
  hw_request->target_state.id = State::PRIMARY_STATE_INACTIVE;
  auto hw_response =
    kroshu_ros2_core::sendRequest<SetHardwareComponentState::Response>(
    change_hardware_state_client_, hw_request, 0, 2000);
  if (!hw_response || !hw_response->ok) {
    RCLCPP_ERROR(get_logger(), "Could not configure hardware interface");
    return FAILURE;
  }
  RCLCPP_INFO(get_logger(), "Successfully configured hardware interface");

  is_configured_pub_->on_activate();
  is_configured_msg_.data = true;
  is_configured_pub_->publish(is_configured_msg_);
  return SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
RobotManagerNode::on_cleanup(const rclcpp_lifecycle::State &)
{
  // Clean up hardware interface
  auto hw_request =
    std::make_shared<SetHardwareComponentState::Request>();
  hw_request->name = robot_model_;
  hw_request->target_state.id = State::PRIMARY_STATE_UNCONFIGURED;
  auto hw_response =
    kroshu_ros2_core::sendRequest<SetHardwareComponentState::Response>(
    change_hardware_state_client_, hw_request, 0, 2000);
  if (!hw_response || !hw_response->ok) {
    RCLCPP_ERROR(get_logger(), "Could not clean up hardware interface");
    return FAILURE;
  }

  if (is_configured_pub_->is_activated()) {
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

  while (reader->Read(&response)) {
    switch (static_cast<int>(response.event())) {
      case kuka::ecs::v1::CommandEvent::COMMAND_MODE_SWITCH:
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
        if (this->get_current_state().id() == State::PRIMARY_STATE_ACTIVE) {
          this->deactivate();
        } else if (this->get_current_state().id() == State::TRANSITION_STATE_ACTIVATING) {
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
  if (context_ != nullptr) {
    context_->TryCancel();
  }
#endif
  // Join observe thread, necessary if previous activation failed
  if (observe_thread_.joinable()) {
    observe_thread_.join();
  }
  terminate_ = false;
  // Subscribe to stream of state changes
  observe_thread_ = std::thread(&RobotManagerNode::ObserveControl, this);

  // Activate hardware interface
  auto hw_request =
    std::make_shared<SetHardwareComponentState::Request>();
  hw_request->name = robot_model_;
  hw_request->target_state.id = State::PRIMARY_STATE_ACTIVE;
  auto hw_response =
    kroshu_ros2_core::sendRequest<SetHardwareComponentState::Response>(
    change_hardware_state_client_, hw_request, 0, 2000);
  if (!hw_response || !hw_response->ok) {
    RCLCPP_ERROR(get_logger(), "Could not activate hardware interface");
    return FAILURE;
  }

  // Select controllers
  auto control_mode = this->get_parameter("control_mode").as_int();

  std::pair<std::vector<std::string>, std::vector<std::string>> new_controllers;
  try {
    new_controllers = controller_handler_.GetControllersForSwitch(
      kroshu_ros2_core::ControlMode(
        control_mode));
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Error while activating controllers: %s", e.what());
    return ERROR;
  }

  // Activate RT controller(s)
  auto controller_request =
    std::make_shared<SwitchController::Request>();
  controller_request->strictness = SwitchController::Request::STRICT;
  controller_request->activate_controllers = new_controllers.first;
  if (!new_controllers.second.empty()) {
    // This should never happen
    controller_request->deactivate_controllers = new_controllers.second;
    RCLCPP_ERROR(
      get_logger(),
      "Controller handler state is improper");
    RCLCPP_ERROR(
      get_logger(),
      "Active controller list is not empty before activation");
  }

  auto controller_response =
    kroshu_ros2_core::sendRequest<SwitchController::Response>(
    change_controller_state_client_, controller_request, 0, 2000
    );
  if (!controller_response || !controller_response->ok) {
    RCLCPP_ERROR(get_logger(), "Could not  activate controller");
    // TODO(Svastits): this can be removed if rollback is implemented properly
    this->on_deactivate(get_current_state());
    return FAILURE;
  }
  controller_handler_.ApproveControllerActivation();
  controller_handler_.ApproveControllerDeactivation();
  RCLCPP_INFO(get_logger(), "Successfully activated controllers");


  // Return failure if control is stopped while in state activating
  if (terminate_) {
    RCLCPP_ERROR(get_logger(), "UDP communication could not be set up");
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
  hw_request->name = robot_model_;
  hw_request->target_state.id = State::PRIMARY_STATE_INACTIVE;
  auto hw_response =
    kroshu_ros2_core::sendRequest<SetHardwareComponentState::Response>(
    change_hardware_state_client_, hw_request, 0, 3000);   // was not stable with 2000 ms timeout
  if (!hw_response || !hw_response->ok) {
    RCLCPP_ERROR(get_logger(), "Could not deactivate hardware interface");
    return ERROR;
  }

  RCLCPP_INFO(get_logger(), "Deactivated LBR iisy hardware interface");


  // Stop RT controllers
  auto controller_request =
    std::make_shared<SwitchController::Request>();
  // With best effort strictness, deactivation succeeds if specific controller is not active
  controller_request->strictness =
    SwitchController::Request::BEST_EFFORT;
  controller_request->deactivate_controllers =
    controller_handler_.GetControllersForDeactivation();
  auto controller_response =
    kroshu_ros2_core::sendRequest<SwitchController::Response>(
    change_controller_state_client_, controller_request, 0, 2000
    );
  if (!controller_response || !controller_response->ok) {
    RCLCPP_ERROR(get_logger(), "Could not stop controllers");
    return ERROR;
  }
  controller_handler_.ApproveControllerDeactivation();

  RCLCPP_INFO(get_logger(), "Successfully stopped controllers");
  return SUCCESS;
}

bool RobotManagerNode::onControlModeChangeRequest(int control_mode)
{
  try {
    RCLCPP_INFO(get_logger(), "Control mode change requested");
    // TODO(komaromi): Remove this if a new control mode is supported
    if (control_mode ==
      static_cast<int>(kroshu_ros2_core::ControlMode::CARTESIAN_POSITION_CONTROL_MODE) ||
      control_mode ==
      static_cast<int>(kroshu_ros2_core::ControlMode::CARTESIAN_IMPEDANCE_CONTROL_MODE) ||
      control_mode == static_cast<int>(kroshu_ros2_core::ControlMode::WRENCH_CONTROL_MODE))
    {
      RCLCPP_ERROR(get_logger(), "Tried to change to a not implemented control mode");
      return false;
    }

    auto controller_request =
      std::make_shared<SwitchController::Request>();
    std::pair<std::vector<std::string>, std::vector<std::string>> switch_controllers;

    bool isActiveState = get_current_state().id() ==
      lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE;

    // Activate controllers
    if (isActiveState) {
      // The driver is in active state

      // Asks for witch controller to activate and deactivate
      switch_controllers = controller_handler_.GetControllersForSwitch(
        kroshu_ros2_core::ControlMode(control_mode));

      // Call request for activateing controllers for the new control mode
      if (!switch_controllers.first.empty()) {
        controller_request->activate_controllers = switch_controllers.first;
        controller_request->strictness = SwitchController::Request::STRICT;
        auto controller_response =
          kroshu_ros2_core::sendRequest<SwitchController::Response>(
          change_controller_state_client_, controller_request, 0, 2000
          );
        if (!controller_response || !controller_response->ok) {
          RCLCPP_ERROR(get_logger(), "Could not activate controllers for new control mode");
          // TODO(Svastits): this can be removed if rollback is implemented properly
          this->on_deactivate(get_current_state());
          return false;
        }
      }
      controller_handler_.ApproveControllerActivation();
    }

    // Publish the control mode to controller handler
    auto message = std_msgs::msg::UInt32();
    message.data = control_mode;
    control_mode_pub_->publish(message);
    RCLCPP_INFO(get_logger(), "Control mode change process has started");

    if (isActiveState) {
      // The driver is in active state

#ifdef NON_MOCK_SETUP
      // Wait for ObserControl to approve tha robot succefully changed control mode
      std::unique_lock<std::mutex> control_mode_lk(this->control_mode_cv_m_);

      if (!this->control_mode_cv_.wait_for(
          control_mode_lk, std::chrono::milliseconds(control_mode_change_timeout_), [this]() {
            return this->control_mode_change_finished_;
          }))
      {
        // Control Mode change timeout reached
        RCLCPP_ERROR(
          get_logger(),
          "Timeout reached while waiting for robot to change control mode.");
        this->on_deactivate(get_current_state());
        return false;
      }
      control_mode_change_finished_ = false;
      control_mode_lk.unlock();
#endif

      // Deactivate controllers

      // Call request for deactivating controllers for the new control mode
      if (!switch_controllers.second.empty()) {
        controller_request->activate_controllers.clear();
        controller_request->deactivate_controllers = switch_controllers.second;
        controller_request->strictness = SwitchController::Request::STRICT;
        auto controller_response =
          kroshu_ros2_core::sendRequest<SwitchController::Response>(
          change_controller_state_client_, controller_request, 0, 2000
          );
        if (!controller_response || !controller_response->ok) {
          RCLCPP_ERROR(get_logger(), "Could not deactivate controllers for new control mode");
          // TODO(Svastits): this can be removed if rollback is implemented properly
          this->on_deactivate(get_current_state());
          return false;
        }
      }
      controller_handler_.ApproveControllerDeactivation();
    }

    RCLCPP_INFO(
      get_logger(), "Successfully changed control mode to %s", ExternalControlMode_Name(
        control_mode).c_str());
    return true;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Error while control mode change: %s", e.what());
    return false;
  }
}

bool RobotManagerNode::onRobotModelChangeRequest(const std::string & robot_model)
{
  robot_model_ = robot_model;
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
