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

#include "kuka_sunrise/robot_control_client.hpp"
#include "kuka_sunrise/robot_commander.hpp"
#include "kuka_sunrise/robot_observer.hpp"

namespace kuka_sunrise
{
CallbackReturn RobotControlClient::on_init(const hardware_interface::HardwareInfo & system_info)
{
  if (hardware_interface::SystemInterface::on_init(system_info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }
  hw_states_.resize(info_.joints.size());
  hw_commands_.resize(info_.joints.size());

  robot_commander_ = std::make_unique<RobotCommander>(robotCommand(), robotState());
  robot_observer_ = std::make_unique<RobotObserver>(robotState());

  for (const hardware_interface::ComponentInfo & joint : info_.joints) {

    if (joint.command_interfaces.size() != 1) {
      RCLCPP_FATAL(
        rclcpp::get_logger("RobotControlClient"),
        "expecting exactly 1 command interface");
      return CallbackReturn::ERROR;
    }

    // TODO(Svastits): enable effort interface too
    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
      RCLCPP_FATAL(
        rclcpp::get_logger(
          "RobotControlClient"), "expecting only POSITION command interface");
      return CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 1) {
      RCLCPP_FATAL(rclcpp::get_logger("RobotControlClient"), "expecting exactly 1 state interface");
      return CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
      RCLCPP_FATAL(
        rclcpp::get_logger(
          "RobotControlClient"), "expecting only POSITION state interface");
      return CallbackReturn::ERROR;
    }
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn RobotControlClient::on_configure(const rclcpp_lifecycle::State &)
{
  return CallbackReturn::SUCCESS;
}

CallbackReturn RobotControlClient::on_activate(const rclcpp_lifecycle::State &)
{
  return CallbackReturn::SUCCESS;
}

CallbackReturn RobotControlClient::on_deactivate(const rclcpp_lifecycle::State &)
{

  return CallbackReturn::SUCCESS;
}


/*RobotControlClient::RobotControlClient()
: receive_multiplier_(1), receive_counter_(0)
{
  robot_observer_ = std::make_unique<RobotObserver>(robotState(), robot_control_node);
  robot_commander_ = std::make_unique<RobotCommander>(
    robotCommand(), robotState(),
    robot_control_node);
  auto command_srv_callback = [this](
    kuka_sunrise_interfaces::srv::SetInt::Request::SharedPtr request,
    kuka_sunrise_interfaces::srv::SetInt::Response::SharedPtr response) {
      if (this->setReceiveMultiplier(request->data)) {
        response->success = true;
      } else {
        response->success = false;
      }
    };
  set_receive_multiplier_service_ = robot_control_node_->create_service<
    kuka_sunrise_interfaces::srv::SetInt>("set_receive_multiplier", command_srv_callback);
}*/

RobotControlClient::~RobotControlClient()
{
  robot_commander_->deactivate();
}

bool RobotControlClient::activate()
{
  // TODO(Svastits): activating the robot_observer should be moved to the on_activate function
  //   of the node! As of now, activating the driver nodes in themselves do not activate
  //   the observer, and the monitoring mode is not working on the ROS2 side
  //   (the publisher is not active, joint states are not sent)
  this->ActivatableInterface::activate();
  robot_commander_->activate();
  robot_observer_->activate();
  return true;  // TODO(resizoltan) check if successful
}

bool RobotControlClient::deactivate()
{
  this->ActivatableInterface::deactivate();
  robot_commander_->deactivate();
  robot_observer_->deactivate();
  return true;  // TODO(resizoltan) check if successful
}

void RobotControlClient::monitor()
{
  rclcpp::Time stamp = ros_clock_.now();
  robot_observer_->publishRobotState(stamp);
}

void RobotControlClient::waitForCommand()
{
  rclcpp::Time stamp = ros_clock_.now();
  robot_observer_->publishRobotState(stamp);
  if (++receive_counter_ == receive_multiplier_) {
    robot_commander_->updateCommand(stamp);
    receive_counter_ = 0;
  }
}

void RobotControlClient::command()
{
  rclcpp::Time stamp = ros_clock_.now();
  robot_observer_->publishRobotState(stamp);
  if (++receive_counter_ == receive_multiplier_) {
    robot_commander_->updateCommand(stamp);
    receive_counter_ = 0;
  }
}

bool RobotControlClient::setReceiveMultiplier(int receive_multiplier)
{
  if (1) {
    receive_multiplier_ = receive_multiplier;
    return true;
  } else {
    return false;
  }
}

hardware_interface::return_type RobotControlClient::read(
  const rclcpp::Time &,
  const rclcpp::Duration &)
{
  if (!is_active_) {
    return hardware_interface::return_type::ERROR;
  }

  if(!client_application_.client_app_read())
  {
	  RCLCPP_ERROR(rclcpp::get_logger("ClientApplication"), "Failed to read data from controller");
	  return hardware_interface::return_type::ERROR;
  }

  // get the position and efforts and share them with exposed state interfaces
  const double* position = robotState().getMeasuredJointPosition();
  hw_states_.assign(position, position+KUKA::FRI::LBRState::NUMBER_OF_JOINTS);
  const double* torque = robotState().getMeasuredTorque();
  hw_torques_.assign(torque, torque+KUKA::FRI::LBRState::NUMBER_OF_JOINTS);
  // const double* external_torque = robotState().getExternalTorque();
  // TODO(Svastits): add external torque interface

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type RobotControlClient::write(
  const rclcpp::Time &,
  const rclcpp::Duration &)
{
  if (!is_active_) {
    RCLCPP_INFO(rclcpp::get_logger("RobotControlClient"), "Controller deactivated");
    return hardware_interface::return_type::ERROR;
  }


  client_application_.client_app_update();

  for (size_t i = 0; i < info_.joints.size(); i++) {
    RCLCPP_INFO(
      rclcpp::get_logger(
        "RobotControlClient"), "Got command %.5f for joint %ld!", hw_commands_[i], i);
  }

  client_application_.client_app_write();

  return hardware_interface::return_type::OK;
}

std::vector<hardware_interface::StateInterface> RobotControlClient::export_state_interfaces()
{
  RCLCPP_INFO(rclcpp::get_logger("RobotControlClient"), "export_state_interfaces()");

  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < info_.joints.size(); i++) {
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name,
        hardware_interface::HW_IF_POSITION,
        &hw_states_[i]));

    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name,
        hardware_interface::HW_IF_EFFORT,
        &hw_torques_[i]));
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> RobotControlClient::export_command_interfaces()
{
  RCLCPP_INFO(rclcpp::get_logger("RobotControlClient"), "export_command_interfaces()");

  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < info_.joints.size(); i++) {
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        info_.joints[i].name,
        hardware_interface::HW_IF_POSITION,
        &hw_commands_[i]));

    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        info_.joints[i].name,
        hardware_interface::HW_IF_EFFORT,
        &hw_effort_command_[i]));
  }
  return command_interfaces;
}

}  // namespace kuka_sunrise
