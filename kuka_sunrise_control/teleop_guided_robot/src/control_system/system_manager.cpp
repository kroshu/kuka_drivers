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

#include <string>
#include <memory>
#include <thread>

#include "control_system/system_manager.hpp"

namespace control_system
{

SystemManager::SystemManager(
  const std::string & node_name,
  const rclcpp::NodeOptions & options)
: ROS2BaseLCNode(node_name, options)
{
  qos_.reliable();
  cbg_ = this->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);
  change_robot_manager_state_client_ = this->create_client<
    std_srvs::srv::SetBool>(
    ROBOT_INTERFACE + "/set_commanding_state",
    qos_.get_rmw_qos_profile(), cbg_);
  robot_commanding_state_subscription_ = this->create_subscription<
    std_msgs::msg::Bool>(
    ROBOT_INTERFACE + "/commanding_state_changed", qos_,
    [this](std_msgs::msg::Bool::SharedPtr msg) {
      this->robotCommandingStateChanged(msg->data);
    });

  get_state_client_ =
    this->create_client<kuka_sunrise_interfaces::srv::GetState>(
    "robot_control/get_fri_state");
  manage_processing_publisher_ = this->create_publisher<std_msgs::msg::Bool>(
    "system_manager/manage", 1);
  manage_processing_publisher_->on_activate();
  auto trigger_change_callback = [this](
    std_srvs::srv::Trigger::Request::SharedPtr,
    std_srvs::srv::Trigger::Response::SharedPtr response) {
      response->success = true;

      if (this->get_current_state().label() == "active") {
        std_msgs::msg::Bool activate;
        activate.data = false;
        manage_processing_publisher_->publish(activate);
        if (this->deactivate().label() != "inactive") {
          response->success = false;
        }
        RCLCPP_WARN(
          get_logger(),
          "Motion stopped externally, deactivating controls and managers");
      } else {
        RCLCPP_WARN(
          get_logger(),
          "Invalid request, system manager not active");
      }
    };
  trigger_change_service_ = this->create_service<std_srvs::srv::Trigger>(
    "system_manager/trigger_change", trigger_change_callback);
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn SystemManager::
on_configure(
  const rclcpp_lifecycle::State &)
{
  if (!changeState(
      ROBOT_INTERFACE,
      lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE))
  {
    return FAILURE;
  }
  if (!changeState(
      JOINT_CONTROLLER,
      lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE))
  {
    if (!changeState(
        ROBOT_INTERFACE,
        lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP))
    {
      RCLCPP_ERROR(get_logger(), "Could not solve differing states, restart needed");
    }
    return ERROR;
  }

  if (!changeState(CONTROL_LOGIC, lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE)) {
    return FAILURE;
  }
  return SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn SystemManager::on_cleanup(
  const rclcpp_lifecycle::State &)
{
  if (!changeState(
      ROBOT_INTERFACE,
      lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP))
  {
    return FAILURE;
  }

  if (!changeState(
      JOINT_CONTROLLER,
      lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP))
  {
    return FAILURE;
  }
  if (!changeState(CONTROL_LOGIC, lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP)) {
    return FAILURE;
  }

  return SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
SystemManager::on_activate(const rclcpp_lifecycle::State &)
{
  if (!changeState(
      ROBOT_INTERFACE,
      lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE))
  {
    return FAILURE;
  }
  if (!changeState(
      JOINT_CONTROLLER,
      lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE))
  {
    if (!changeState(
        ROBOT_INTERFACE,
        lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE))
    {
      RCLCPP_ERROR(get_logger(), "Could not solve differing states, restart needed");
      return ERROR;
    }
    return FAILURE;
  }
  if (!robot_control_active_ && !changeRobotCommandingState(true)) {
    if (!changeState(
        ROBOT_INTERFACE,
        lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE) ||
      !changeState(
        JOINT_CONTROLLER,
        lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE))
    {
      RCLCPP_ERROR(get_logger(), "Could not solve differing states, restart needed");
      return ERROR;
    }
    return FAILURE;
  }
  if (!changeState(CONTROL_LOGIC, lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE)) {
    return FAILURE;
  }

  polling_thread_ = std::thread(&SystemManager::monitoringLoop, this);
  robot_control_active_ = true;
  std_msgs::msg::Bool activate;
  activate.data = true;
  manage_processing_publisher_->publish(activate);
  return SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn SystemManager::
on_deactivate(
  const rclcpp_lifecycle::State &)
{
  if (robot_control_active_ && !changeRobotCommandingState(false)) {
    return FAILURE;
  }
  robot_control_active_ = false;
  if (polling_thread_.joinable()) {polling_thread_.join();}
  if (!changeState(
      ROBOT_INTERFACE,
      lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE))
  {
    return FAILURE;
  }
  if (!changeState(
      JOINT_CONTROLLER,
      lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE))
  {
    return FAILURE;
  }

  if (!changeState(CONTROL_LOGIC, lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE)) {
    return FAILURE;
  }

  return SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
SystemManager::on_shutdown(const rclcpp_lifecycle::State & state)
{
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn result =
    SUCCESS;
  switch (state.id()) {
    case lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE:
      result = this->on_deactivate(get_current_state());
      if (result != SUCCESS) {
        break;
      }
      result = this->on_cleanup(get_current_state());
      break;
    case lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE:
      result = this->on_cleanup(get_current_state());
      break;
    case lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED:
      break;
    default:
      break;
  }
  if (!changeState(
      ROBOT_INTERFACE,
      lifecycle_msgs::msg::Transition::TRANSITION_UNCONFIGURED_SHUTDOWN))
  {
    return FAILURE;
  }
  if (!changeState(
      JOINT_CONTROLLER,
      lifecycle_msgs::msg::Transition::TRANSITION_UNCONFIGURED_SHUTDOWN))
  {
    return FAILURE;
  }

  if (!changeState(
      CONTROL_LOGIC,
      lifecycle_msgs::msg::Transition::TRANSITION_UNCONFIGURED_SHUTDOWN))
  {
    return FAILURE;
  }


  return result;
}

void SystemManager::monitoringLoop()
{
  while (this->get_current_state().label() == "active") {
    getFRIState();
    std::this_thread::sleep_for(sleeping_time_ms_);
  }
  RCLCPP_WARN(get_logger(), "Stopping monitoring loop");
}

bool SystemManager::changeState(
  const std::string & node_name,
  std::uint8_t transition)
{
  auto client = this->create_client<lifecycle_msgs::srv::ChangeState>(
    node_name + "/change_state", qos_.get_rmw_qos_profile(), cbg_);
  auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
  request->transition.id = transition;
  if (!client->wait_for_service(std::chrono::milliseconds(2000))) {
    RCLCPP_ERROR(get_logger(), "Wait for service failed");
    return false;
  }
  auto future_result = client->async_send_request(request);
  auto future_status = kuka_sunrise::wait_for_result(
    future_result,
    std::chrono::milliseconds(3000));
  if (future_status != std::future_status::ready) {
    RCLCPP_ERROR(get_logger(), "Future status not ready, could not change state of " + node_name);
    return false;
  }
  if (future_result.get()->success) {
    return true;
  } else {
    RCLCPP_ERROR(get_logger(), "Future result not success, could not change state of " + node_name);
    return false;
  }
}

void SystemManager::getFRIState()
{
  while (!get_state_client_->wait_for_service(std::chrono::milliseconds(1000))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Interrupted while waiting for the service. Exiting.");
      return;
    }
    RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
  }
  auto request = std::make_shared<
    kuka_sunrise_interfaces::srv::GetState::Request>();

  auto response_received_callback =
    [this](
    rclcpp::Client<kuka_sunrise_interfaces::srv::GetState>::SharedFuture future) {
      auto result = future.get();
      lbr_state_ = static_cast<int>(result->data);

      // Two consecutive non-four states are needed for shutdown
      // to avoid unnecessary stops
      if (lbr_state_ != 4 && !stop_) {
        stop_ = true;
      } else if (lbr_state_ != 4 && stop_) {
        std_msgs::msg::Bool activate;
        activate.data = false;
        manage_processing_publisher_->publish(activate);
      } else {
        stop_ = false;
      }
      RCLCPP_DEBUG(this->get_logger(), "State: %i", lbr_state_);
    };
  auto future_result = get_state_client_->async_send_request(
    request,
    response_received_callback);
}

// Activate the ActivatableInterface of robot_manager_node
bool SystemManager::changeRobotCommandingState(bool is_active)
{
  auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
  request->data = is_active;
  if (!change_robot_manager_state_client_->wait_for_service(
      std::chrono::milliseconds(2000)))
  {
    RCLCPP_ERROR(get_logger(), "Wait for service failed");
    return false;
  }
  auto future_result =
    change_robot_manager_state_client_->async_send_request(request);
  auto future_status = kuka_sunrise::wait_for_result(
    future_result,
    std::chrono::milliseconds(3000));
  if (future_status != std::future_status::ready) {
    RCLCPP_ERROR(get_logger(), "Future status not ready, could not change robot commanding state");
    return false;
  }
  if (future_result.get()->success) {
    robot_control_active_ = true;
    return true;
  } else {
    RCLCPP_ERROR(
      get_logger(),
      "Future result not success, could not change robot commanding state");
    return false;
  }
}

void SystemManager::robotCommandingStateChanged(bool is_active)
{
  if (is_active == false && this->get_current_state().label() == "active") {
    robot_control_active_ = false;
    // TODO(resizoltan): check if successful
    this->deactivate();
  }
}

}  // namespace control_system

int main(int argc, char * argv[])
{
  setvbuf(stdout, nullptr, _IONBF, BUFSIZ);

  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor;
  auto node = std::make_shared<control_system::SystemManager>(
    "system_manager", rclcpp::NodeOptions());
  executor.add_node(node->get_node_base_interface());
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
