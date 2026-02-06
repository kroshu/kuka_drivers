// Copyright 2025 KUKA Hungaria Kft.
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
#include <thread>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

#include "kuka_drivers_core/hardware_interface_types.hpp"
#include "kuka_rsi_driver/event_observers.hpp"
#include "kuka_rsi_driver/hardware_interface_eki_rsi.hpp"

#include "kuka/external-control-sdk/kss/eki/initialization_data.h"

namespace kuka_rsi_driver
{

CallbackReturn KukaEkiRsiHardwareInterface::on_init(const hardware_interface::HardwareInfo & info)
{
  if (KukaRSIHardwareInterfaceBase::on_init(info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  RCLCPP_INFO(logger_, "Controller IP: %s", info_.hardware_parameters.at("controller_ip").c_str());

  auto it = info_.hardware_parameters.find("verify_robot_model");
  verify_robot_model_ = (it != info_.hardware_parameters.end() && it->second == "True");
  RCLCPP_INFO(
    logger_, "Robot model verification: %s", verify_robot_model_ ? "enabled" : "disabled");

  cycle_time_command_ = 0.0;
  hw_control_mode_command_ = 0.0;

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::CommandInterface>
KukaEkiRsiHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  command_interfaces = KukaRSIHardwareInterfaceBase::export_command_interfaces();

  command_interfaces.emplace_back(
    hardware_interface::CONFIG_PREFIX, hardware_interface::CONTROL_MODE, &hw_control_mode_command_);

  command_interfaces.emplace_back(
    hardware_interface::CONFIG_PREFIX, hardware_interface::CYCLE_TIME, &cycle_time_command_);

  return command_interfaces;
}

std::vector<hardware_interface::StateInterface>
KukaEkiRsiHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  state_interfaces = KukaRSIHardwareInterfaceBase::export_state_interfaces();

  status_manager_.RegisterStateInterfaces(state_interfaces);

  return state_interfaces;
}

CallbackReturn KukaEkiRsiHardwareInterface::on_configure(const rclcpp_lifecycle::State &)
{
  kuka::external::control::kss::Configuration eki_config;
  eki_config.installed_interface =
    kuka::external::control::kss::Configuration::InstalledInterface::EKI_RSI;
  eki_config.kli_ip_address = info_.hardware_parameters.at("controller_ip");
  eki_config.client_ip = info_.hardware_parameters.at("client_ip");
  eki_config.client_port = std::stoi(info_.hardware_parameters.at("client_port"));

  if (!SetupRobot(
        eki_config, std::make_unique<EventObserver>(this),
        std::make_unique<EkiEventHandlerExtension>(this)))
  {
    return CallbackReturn::ERROR;
  }

  RCLCPP_INFO(
    logger_, "Network setup successful - Controller: %s:%d, RSI listening on %s:%d",
    eki_config.kli_ip_address.c_str(), eki_config.eki_port, eki_config.client_ip.c_str(),
    eki_config.client_port);

  auto status = robot_ptr_->RegisterStatusResponseHandler(
    std::make_unique<StatusUpdateHandler>(this, &status_manager_));
  if (status.return_code != kuka::external::control::ReturnCode::OK)
  {
    RCLCPP_ERROR(logger_, "Registering status response handler failed: %s", status.message);
    return CallbackReturn::ERROR;
  }

  // Wait for the response to arrive from the controller
  std::unique_lock<std::mutex> lk{init_mtx_};
  init_cv_.wait(lk, [this] { return init_report_.sequence_complete; });

  if (verify_robot_model_)
  {
    if (!init_report_.ok)
    {
      RCLCPP_ERROR(
        logger_, "The driver is incompatible with the current hardware and software setup: %s",
        init_report_.reason.c_str());
      robot_ptr_.reset();
      return CallbackReturn::ERROR;
    }
    RCLCPP_INFO(logger_, "The driver is compatible with the current hardware and software setup");
  }
  else
  {
    RCLCPP_INFO(logger_, "Robot model verification is disabled, proceeding with the connection.");
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn KukaEkiRsiHardwareInterface::on_activate(const rclcpp_lifecycle::State & state)
{
  return KukaRSIHardwareInterfaceBase::extended_activation(state);
}

CallbackReturn KukaEkiRsiHardwareInterface::on_deactivate(const rclcpp_lifecycle::State & state)
{
  return KukaRSIHardwareInterfaceBase::extended_deactivation(state);
}

return_type KukaEkiRsiHardwareInterface::read(
  const rclcpp::Time & time, const rclcpp::Duration & duration)
{
  status_manager_.UpdateStateInterfaces();

  return KukaRSIHardwareInterfaceBase::read(time, duration);
}

std::string FindRobotModelInUrdfName(const std::string & input)
{
  // Regex pattern to match robot model names
  std::regex pattern(
    "kr\\d+_r\\d+_[a-z\\d]*", std::regex_constants::ECMAScript | std::regex_constants::icase);
  std::smatch match;
  std::string last_match;
  auto search_start = input.cbegin();

  while (std::regex_search(search_start, input.cend(), match, pattern))
  {
    last_match = match.str();
    search_start = match.suffix().first;
  }

  // Remove underscores
  last_match.erase(std::remove(last_match.begin(), last_match.end(), '_'), last_match.end());
  return last_match;
}

std::string ProcessKrcReportedRobotName(const std::string & input)
{
  std::string trimmed = input.substr(1);
  std::size_t space_pos = trimmed.find(' ');
  std::string robot_model = trimmed.substr(0, space_pos);
  std::transform(robot_model.begin(), robot_model.end(), robot_model.begin(), ::tolower);
  return robot_model;
}

void KukaEkiRsiHardwareInterface::eki_init(const InitializationData & init_data)
{
  {
    std::lock_guard<std::mutex> lk{init_mtx_};
    std::string expected = FindRobotModelInUrdfName(info_.hardware_parameters.at("name"));

    const auto * eki_init_data =
      dynamic_cast<const kuka::external::control::kss::eki::EKIInitializationData *>(&init_data);
    if (!eki_init_data)
    {
      init_report_ = {true, false, "Unexpected initialization data type for EKI"};
      init_cv_.notify_one();
      return;
    }

    std::string reported = ProcessKrcReportedRobotName(eki_init_data->model_name);
    if (
      expected.find(reported) == std::string::npos && reported.find(expected) == std::string::npos)
    {
      std::ostringstream oss;
      oss << "Robot model mismatch detected: expected model '" << expected << "', but found '"
          << reported << "'";
      init_report_ = {true, false, oss.str()};
    }
    else if (info_.joints.size() != init_data.GetTotalAxisCount())
    {
      std::ostringstream oss;
      oss << "Mismatch in axis count: Driver expects " << info_.joints.size()
          << ", but EKI server reported " << init_data.GetTotalAxisCount();
      init_report_ = {true, false, oss.str()};
    }
    else
    {
      init_report_ = {true, true, ""};
    }
  }
  init_cv_.notify_one();
}

void KukaEkiRsiHardwareInterface::Read(const int64_t request_timeout)
{
  if (status_manager_.IsEmergencyStopActive())
  {
    RCLCPP_ERROR(logger_, "Emergency stop detected!");
    set_server_event(kuka_drivers_core::HardwareEvent::ERROR);
  }
  else if (!status_manager_.DrivesPowered())
  {
    RCLCPP_ERROR(logger_, "Drives are not powered!");
    set_server_event(kuka_drivers_core::HardwareEvent::ERROR);
  }

  KukaRSIHardwareInterfaceBase::Read(request_timeout);
}

void KukaEkiRsiHardwareInterface::CreateRobotInstance(
  const kuka::external::control::kss::Configuration & config)
{
  robot_ptr_ = std::make_unique<kuka::external::control::kss::eki::Robot>(config);
}
}  // namespace kuka_rsi_driver

PLUGINLIB_EXPORT_CLASS(
  kuka_rsi_driver::KukaEkiRsiHardwareInterface, hardware_interface::SystemInterface)
