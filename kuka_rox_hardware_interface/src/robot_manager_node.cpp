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
#include "nanopb/kuka/motion/external/control_signal_internal.pb.hh"
#include "nanopb/kuka/motion/external/external_command.pb.hh"


#include <grpcpp/create_channel.h>

using namespace controller_manager_msgs::srv;  // NOLINT
using namespace lifecycle_msgs::msg;  // NOLINT
using namespace kuka::motion::external;  // NOLINT

namespace kuka_rox
{
RobotManagerNode::RobotManagerNode()
: kroshu_ros2_core::ROS2BaseLCNode("robot_manager")
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

  control_mode_publisher_ = this->create_publisher<std_msgs::msg::UInt32>(
    "control_mode", rclcpp::SystemDefaultsQoS()
  );

  // Create control_mode_map structure
  control_mode_map_.emplace(
    std::make_pair(
      ExternalControlMode::POSITION_CONTROL,
      std::vector<std::string>(STANDARD_MODE_IF_SIZE)));
  control_mode_map_.emplace(
    std::make_pair(
      ExternalControlMode::JOINT_IMPEDANCE_CONTROL,
      std::vector<std::string>(IMPEDANCE_MODE_IF_SIZE)));
  control_mode_map_.emplace(
    std::make_pair(
      ExternalControlMode::TORQUE_CONTROL,
      std::vector<std::string>(STANDARD_MODE_IF_SIZE)));

  control_mode_map_.at(ExternalControlMode::POSITION_CONTROL).at(0) = "joint_state_broadcaster";
  control_mode_map_.at(ExternalControlMode::JOINT_IMPEDANCE_CONTROL).at(0) =
    "joint_state_broadcaster";
  control_mode_map_.at(ExternalControlMode::TORQUE_CONTROL).at(0) = "joint_state_broadcaster";

  // Register parameters
  this->registerParameter<int>(
    "control_mode", static_cast<int>(ExternalControlMode::POSITION_CONTROL),
    kroshu_ros2_core::ParameterSetAccessRights{true, true,
      true, false, false}, [this](int control_mode) {
      return this->onControlModeChangeRequest(control_mode);
    });
  this->registerParameter<std::string>(
    "position_controller_name", "", kroshu_ros2_core::ParameterSetAccessRights {true, true,
      false, false, false}, [this](const std::string & controller_name) {
      control_mode_map_.at(ExternalControlMode::POSITION_CONTROL).at(1) = controller_name;
      control_mode_map_.at(ExternalControlMode::JOINT_IMPEDANCE_CONTROL).at(1) = controller_name;
      return true;
    });
  this->registerParameter<std::string>(
    "impedance_controller_name", "", kroshu_ros2_core::ParameterSetAccessRights {true, true,
      false, false, false}, [this](const std::string & controller_name) {
      control_mode_map_.at(ExternalControlMode::JOINT_IMPEDANCE_CONTROL).at(2) = controller_name;
      return true;
    });
  this->registerParameter<std::string>(
    "torque_controller_name", "", kroshu_ros2_core::ParameterSetAccessRights {true, true, false,
      false, false}, [this](const std::string & controller_name) {
      control_mode_map_.at(ExternalControlMode::TORQUE_CONTROL).at(1) = controller_name;
      return true;
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
  // Configure hardware interface
  auto hw_request =
    std::make_shared<SetHardwareComponentState::Request>();
  hw_request->name = "iisy_hardware";
  hw_request->target_state.id = State::PRIMARY_STATE_INACTIVE;
  auto hw_response =
    kroshu_ros2_core::sendRequest<SetHardwareComponentState::Response>(
    change_hardware_state_client_, hw_request, 0, 2000);
  if (!hw_response || !hw_response->ok) {
    RCLCPP_ERROR(get_logger(), "Could not configure hardware interface");
    return FAILURE;
  }

  // Activate Control Mode Handler
  auto control_mode_request =
    std::make_shared<SwitchController::Request>();
  control_mode_request->strictness = SwitchController::Request::STRICT;
  control_mode_request->activate_controllers = {"control_mode_handler"};
  auto control_mode_response =
    kroshu_ros2_core::sendRequest<SwitchController::Response>(
    change_controller_state_client_, control_mode_request, 0, 2000
    );
  if (!control_mode_response || !control_mode_response->ok) {
    RCLCPP_ERROR(get_logger(), "Could not  activate control_mode_handler");
  }
  RCLCPP_INFO(get_logger(), "Successfully activated control_mode_handler");

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
  hw_request->name = "iisy_hardware";
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
        RCLCPP_INFO(get_logger(), "Command mode switched");
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
  hw_request->name = "iisy_hardware";
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
  controller_names_ = control_mode_map_.at(ExternalControlMode(control_mode));

  // TODO (Komaromi): controller_names_ have to follow accuratly the active controllers

  // Activate RT controller(s)
  auto controller_request =
    std::make_shared<SwitchController::Request>();
  controller_request->strictness = SwitchController::Request::STRICT;
  controller_request->activate_controllers = controller_names_;

  // auto range = control_mode_map_.equal_range(ExternalControlMode(control_mode));
  // std::transform(
  //   range.first, range.second, std::back_inserter(controller_request->activate_controllers),
  //   [](std::pair<kuka::motion::external::ExternalControlMode, std::string> element) {
  //     return element.second;
  //   });

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
  hw_request->name = "iisy_hardware";
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
  controller_request->deactivate_controllers = controller_names_;
  auto controller_response =
    kroshu_ros2_core::sendRequest<SwitchController::Response>(
    change_controller_state_client_, controller_request, 0, 2000
    );
  if (!controller_response || !controller_response->ok) {
    RCLCPP_ERROR(get_logger(), "Could not stop controllers");
    return ERROR;
  }
  controller_names_.clear();

  RCLCPP_INFO(get_logger(), "Successfully stopped controllers");
  return SUCCESS;
}

bool RobotManagerNode::onControlModeChangeRequest(int control_mode)
{
  if (control_mode_map_.find(ExternalControlMode(control_mode)) != control_mode_map_.end()) {
    RCLCPP_INFO(
      get_logger(), "Control mode changed to %s", ExternalControlMode_Name(control_mode).c_str());
    // TODO (komaromi): change active controllers if control mode changed mid active state
    if (get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
      auto controller_request =
        std::make_shared<SwitchController::Request>();

      auto new_controllers = control_mode_map_.at(ExternalControlMode(control_mode));
      auto actualy_active_controllers = controller_names_;

      for (auto controller_it = new_controllers.begin(); controller_it != new_controllers.end();
        std::next(controller_it))
      {
        auto temp = std::find(
          actualy_active_controllers.begin(), actualy_active_controllers.end(),
          *controller_it);
        if (temp != actualy_active_controllers.end()) {
          new_controllers.erase(controller_it);
          actualy_active_controllers.erase(temp);
        }
      }

      controller_request->activate_controllers = new_controllers;
      controller_request->deactivate_controllers = actualy_active_controllers;

      controller_request->strictness = SwitchController::Request::STRICT;
      auto controller_response =
        kroshu_ros2_core::sendRequest<SwitchController::Response>(
        change_controller_state_client_, controller_request, 0, 2000
        );
      if (!controller_response || !controller_response->ok) {
        RCLCPP_ERROR(get_logger(), "Could not  activate controller");
        // TODO(Svastits): this can be removed if rollback is implemented properly
        this->on_deactivate(get_current_state());
        return false;
      }
      RCLCPP_INFO(get_logger(), "Successfully activated controllers");
    }

    // TODO (komaromi): publish the control mode to controller handler
    auto message = std_msgs::msg::UInt32();
    message.data = control_mode;
    control_mode_publisher_->publish(message);
    return true;
  } else {
    RCLCPP_WARN(
      get_logger(), "Could not change control mode, %s is not a valid control mode",
      ExternalControlMode_Name(control_mode).c_str());
    return false;
  }
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
