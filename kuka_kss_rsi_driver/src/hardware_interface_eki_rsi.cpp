// Copyright 2025 Kristof Pasztor
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

#include <chrono>
#include <regex>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

#include "kuka_drivers_core/hardware_interface_types.hpp"
#include "kuka_kss_rsi_driver/event_observer_eki_rsi.hpp"
#include "kuka_kss_rsi_driver/hardware_interface_eki_rsi.hpp"

namespace kuka_kss_rsi_driver
{

CallbackReturn HardwareInterface::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  hw_states_.resize(info_.joints.size(), 0.0);
  hw_commands_.resize(info_.joints.size(), 0.0);

  for (const auto & joint : info_.joints)
  {
    bool interfaces_ok = CheckJointInterfaces(joint);
    if (!interfaces_ok)
    {
      return CallbackReturn::ERROR;
    }
  }

  RCLCPP_INFO(logger_, "Controller IP: %s", info_.hardware_parameters["controller_ip"].c_str());

  cycle_time_command_ = 0.0;
  drives_enabled_command_ = 1.0;
  hw_control_mode_command_ = 0.0;
  server_state_ = 0.0;

  first_write_done_ = false;
  is_active_ = false;
  msg_received_ = false;
  prev_drives_enabled_ = true;

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> HardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_[i]);
  }

  state_interfaces.emplace_back(
    hardware_interface::STATE_PREFIX, hardware_interface::SERVER_STATE, &server_state_);

  status_manager_.RegisterStateInterfaces(state_interfaces);

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> HardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_[i]);
  }

  command_interfaces.emplace_back(
    hardware_interface::CONFIG_PREFIX, hardware_interface::CONTROL_MODE, &hw_control_mode_command_);

  command_interfaces.emplace_back(
    hardware_interface::CONFIG_PREFIX, hardware_interface::DRIVE_STATE, &drives_enabled_command_);

  command_interfaces.emplace_back(
    hardware_interface::CONFIG_PREFIX, hardware_interface::CYCLE_TIME, &cycle_time_command_);

  return command_interfaces;
}

CallbackReturn HardwareInterface::on_configure(const rclcpp_lifecycle::State &)
{
  const bool connection_successful = ConnectToController();

  // Wait for the response to arrive from the controller
  while (!init_report_.sequence_complete)
  {
    std::this_thread::sleep_for(INIT_WAIT_DURATION);
  }

  if (!init_report_.ok)
  {
    RCLCPP_ERROR(
      logger_, "The driver is incompatible with the current hardware and software setup: %s",
      init_report_.reason.c_str());
    robot_ptr_.reset();
  }
  else
  {
    RCLCPP_INFO(logger_, "The driver is compatible with the current hardware and software setup");
  }

  return connection_successful && init_report_.ok ? CallbackReturn::SUCCESS : CallbackReturn::ERROR;
}

CallbackReturn HardwareInterface::on_cleanup(const rclcpp_lifecycle::State &)
{
  robot_ptr_.reset();
  return CallbackReturn::SUCCESS;
}

CallbackReturn HardwareInterface::on_activate(const rclcpp_lifecycle::State &)
{
  if (!status_manager_.IsKrcInExtMode())
  {
    RCLCPP_ERROR(logger_, "KRC not in EXT. Switch to EXT to activate.");
    return CallbackReturn::FAILURE;
  }

  const auto control_mode =
    static_cast<kuka::external::control::ControlMode>(hw_control_mode_command_);

  kuka::external::control::Status control_status = robot_ptr_->StartControlling(control_mode);
  if (control_status.return_code == kuka::external::control::ReturnCode::ERROR)
  {
    RCLCPP_ERROR(logger_, "Starting external control failed: %s", control_status.message);
    return CallbackReturn::FAILURE;
  }

  prev_control_mode_ = static_cast<kuka_drivers_core::ControlMode>(hw_control_mode_command_);

  stop_requested_ = false;

  // We must first receive the initial position of the robot
  // We set a longer timeout, since the first message might not arrive all that fast
  Read(5 * READ_TIMEOUT_MS);
  std::copy(hw_states_.cbegin(), hw_states_.cend(), hw_commands_.begin());
  Write();

  msg_received_ = false;
  first_write_done_ = true;
  is_active_ = true;

  RCLCPP_INFO(logger_, "Received position data from robot controller!");

  return CallbackReturn::SUCCESS;
}

CallbackReturn HardwareInterface::on_deactivate(const rclcpp_lifecycle::State &)
{
  set_stop_flag();
  return CallbackReturn::SUCCESS;
}

return_type HardwareInterface::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  status_manager_.UpdateStateInterfaces();

  if (!is_active_)
  {
    std::this_thread::sleep_for(IDLE_SLEEP_DURATION);
    return return_type::OK;
  }

  Read(READ_TIMEOUT_MS);
  return return_type::OK;
}

return_type HardwareInterface::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  if (!ShouldWriteJointCommands())
  {
    if (ChangeDriveState() || ChangeCycleTime())
    {
      return return_type::OK;
    }

    return return_type::OK;
  }

  Write();

  return return_type::OK;
}

void HardwareInterface::set_server_event(kuka_drivers_core::HardwareEvent event)
{
  std::lock_guard<std::mutex> lk(event_mutex_);
  last_event_ = event;
}

std::string FindRobotModelInUrdfName(const std::string & input)
{
  // Regex pattern to match robot model names
  std::regex pattern(
    "kr\\d+_r\\d+_[a-z\\d]*(?=\\b)",
    std::regex_constants::ECMAScript | std::regex_constants::icase);
  std::smatch match;
  std::string last_match;
  auto search_start = input.cbegin();

  while (std::regex_search(search_start, input.cend(), match, pattern))
  {
    last_match = match.str();
    search_start = match.suffix().first;
  }

  // Convert to lowercase
  std::transform(last_match.begin(), last_match.end(), last_match.begin(), ::tolower);

  // Remove hyphens and underscores
  last_match.erase(std::remove(last_match.begin(), last_match.end(), '_'), last_match.end());

  return last_match;
}

std::string ProcessKrcReportedRobotName(const std::string & input)
{
  std::string processed = input;

  // Convert to lowercase
  std::transform(processed.begin(), processed.end(), processed.begin(), ::tolower);

  // Remove hyphens and spaces
  processed.erase(std::remove(processed.begin(), processed.end(), '-'), processed.end());
  processed.erase(std::remove(processed.begin(), processed.end(), '_'), processed.end());
  processed.erase(std::remove(processed.begin(), processed.end(), ' '), processed.end());

  return processed;
}

void HardwareInterface::eki_init(const InitializationData & init_data)
{
  const std::string expected = FindRobotModelInUrdfName(info_.hardware_parameters["robot_name"]);
  const std::string reported = ProcessKrcReportedRobotName(init_data.model_name);
  if (reported.find(expected) == std::string::npos)
  {
    std::ostringstream oss;
    oss << "Robot model mismatch: Expected '" << expected
        << "' to be a substring of reported model '" << reported << "'";
    init_report_ = {true, false, oss.str()};
    return;
  }

  if (info_.joints.size() != init_data.GetTotalAxisCount())
  {
    std::ostringstream oss;
    oss << "Mismatch in axis count: Driver expects " << info_.joints.size()
        << ", but EKI server reported " << init_data.GetTotalAxisCount();
    init_report_ = {true, false, oss.str()};
    return;
  }

  init_report_ = {true, true, ""};
}

void HardwareInterface::initialize_command_interfaces(
  kuka_drivers_core::ControlMode control_mode, RsiCycleTime cycle_time)
{
  prev_control_mode_ = control_mode;
  prev_cycle_time_ = cycle_time;
  hw_control_mode_command_ = static_cast<double>(control_mode);
  cycle_time_command_ = static_cast<double>(cycle_time);
}

bool HardwareInterface::ConnectToController()
{
  RCLCPP_INFO(logger_, "Initiating connection setup to the robot controller...");

  kuka::external::control::kss::Configuration config;
  config.kli_ip_address = info_.hardware_parameters["controller_ip"];
  config.dof = info_.joints.size();

  robot_ptr_ = std::make_unique<kuka::external::control::kss::eki::Robot>(config);

  auto status = robot_ptr_->RegisterEventHandler(std::move(std::make_unique<EventObserver>(this)));
  if (status.return_code == kuka::external::control::ReturnCode::ERROR)
  {
    RCLCPP_ERROR(logger_, "Creating event observer failed: %s", status.message);
  }

  status = robot_ptr_->RegisterEventHandlerExtension(std::make_unique<EventHandlerExtension>(this));
  if (status.return_code == kuka::external::control::ReturnCode::ERROR)
  {
    RCLCPP_INFO(logger_, "Creating event handler extension failed: %s", status.message);
  }

  status = robot_ptr_->Setup();
  if (status.return_code != kuka::external::control::ReturnCode::OK)
  {
    RCLCPP_ERROR(logger_, "Setup failed: %s", status.message);
    return false;
  }

  status = robot_ptr_->RegisterStatusResponseHandler(
    std::make_unique<StatusUpdateHandler>(this, &status_manager_));
  if (status.return_code != kuka::external::control::ReturnCode::OK)
  {
    RCLCPP_ERROR(logger_, "Registering status response handler failed: %s", status.message);
    return false;
  }

  RCLCPP_INFO(logger_, "Successfully established connection to the robot controller!");

  return true;
}

bool HardwareInterface::ShouldWriteJointCommands() const
{
  return is_active_ && msg_received_ && first_write_done_ && prev_drives_enabled_;
}

void HardwareInterface::Read(const int64_t request_timeout)
{
  std::chrono::milliseconds timeout(request_timeout);
  const auto motion_state_status = robot_ptr_->ReceiveMotionState(timeout);

  msg_received_ = motion_state_status.return_code == kuka::external::control::ReturnCode::OK;
  if (msg_received_)
  {
    const auto & req_message = robot_ptr_->GetLastMotionState();
    const auto & positions = req_message.GetMeasuredPositions();
    std::copy(positions.cbegin(), positions.cend(), hw_states_.begin());
  }

  std::lock_guard<std::mutex> lck(event_mutex_);
  server_state_ = static_cast<double>(last_event_);
}

void HardwareInterface::Write()
{
  // Write values to hardware interface
  auto & control_signal = robot_ptr_->GetControlSignal();
  control_signal.AddJointPositionValues(hw_commands_.cbegin(), hw_commands_.cend());

  const auto control_mode = static_cast<kuka_drivers_core::ControlMode>(hw_control_mode_command_);
  const bool control_mode_change_requested = control_mode != prev_control_mode_;
  kuka::external::control::Status send_reply_status;
  if (stop_requested_)
  {
    RCLCPP_INFO(logger_, "Sending stop signal");
    is_active_ = false;
    first_write_done_ = false;
    msg_received_ = false;
    send_reply_status = robot_ptr_->StopControlling();
  }
  else if (control_mode_change_requested)
  {
    // TODO(pasztork): Test this branch once other control modes become available.
    RCLCPP_INFO(logger_, "Requesting control mode change");
    send_reply_status = robot_ptr_->SwitchControlMode(
      static_cast<kuka::external::control::ControlMode>(hw_control_mode_command_));
    prev_control_mode_ = control_mode;
  }
  else
  {
    send_reply_status = robot_ptr_->SendControlSignal();
  }

  if (send_reply_status.return_code != kuka::external::control::ReturnCode::OK)
  {
    RCLCPP_ERROR(logger_, "Sending reply failed: %s", send_reply_status.message);
    throw std::runtime_error("Error sending reply");
  }
}

bool HardwareInterface::CheckJointInterfaces(const hardware_interface::ComponentInfo & joint) const
{
  if (joint.command_interfaces.size() != 1)
  {
    RCLCPP_FATAL(logger_, "Expecting exactly 1 command interface");
    return false;
  }

  if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
  {
    RCLCPP_FATAL(logger_, "Expecting only POSITION command interface");
    return false;
  }

  if (joint.state_interfaces.size() != 1)
  {
    RCLCPP_FATAL(logger_, "Expecting exactly 1 state interface");
    return false;
  }

  if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
  {
    RCLCPP_FATAL(logger_, "Expecting only POSITION state interface");
    return false;
  }

  return true;
}

bool HardwareInterface::ChangeDriveState()
{
  const bool drives_enabled = drives_enabled_command_ == 1.0;
  if (prev_drives_enabled_ != drives_enabled)
  {
    if (drives_enabled)
    {
      RCLCPP_INFO(logger_, "Turning on drives");
      robot_ptr_->TurnOnDrives();
    }
    else
    {
      RCLCPP_INFO(logger_, "Turning off drives");
      robot_ptr_->TurnOffDrives();
    }
    prev_drives_enabled_ = drives_enabled;
    return true;
  }
  return false;
}

bool HardwareInterface::ChangeCycleTime()
{
  const RsiCycleTime cycle_time = static_cast<RsiCycleTime>(cycle_time_command_);

  if (prev_cycle_time_ != cycle_time)
  {
    robot_ptr_->SetCycleTime(cycle_time);
    prev_cycle_time_ = cycle_time;
    return true;
  }

  return false;
}

}  // namespace kuka_kss_rsi_driver

PLUGINLIB_EXPORT_CLASS(kuka_kss_rsi_driver::HardwareInterface, hardware_interface::SystemInterface)
