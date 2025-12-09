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

#ifndef KUKA_RSI_DRIVER__EVENT_OBSERVER_MXA_RSI_HPP_
#define KUKA_RSI_DRIVER__EVENT_OBSERVER_MXA_RSI_HPP_

#include <string>

#include "kuka_drivers_core/hardware_event.hpp"
#include "kuka_rsi_driver/hardware_interface_mxa_rsi.hpp"

namespace kuka_rsi_driver
{
class EventObserverMxa : public kuka::external::control::EventHandler
{
public:
  explicit EventObserverMxa(KukaMxaRsiHardwareInterface * hw_interface)
  : logger_(rclcpp::get_logger("KukaMxaRsiHardwareInterface")), hw_interface_(hw_interface)
  {
  }

  void OnSampling() override
  {
    hw_interface_->set_server_event(kuka_drivers_core::HardwareEvent::CONTROL_STARTED);
    RCLCPP_INFO(logger_, "External control is active");
  }

  void OnControlModeSwitch(const std::string &) override
  {
    hw_interface_->set_server_event(kuka_drivers_core::HardwareEvent::CONTROL_MODE_SWITCH);
    RCLCPP_INFO(logger_, "Control mode switch is in progress");
  }

  void OnStopped(const std::string &) override
  {
    hw_interface_->set_server_event(kuka_drivers_core::HardwareEvent::CONTROL_STOPPED);
    RCLCPP_INFO(logger_, "External control stopped");
  }

  void OnError(const std::string & reason) override
  {
    hw_interface_->set_server_event(kuka_drivers_core::HardwareEvent::ERROR);
    RCLCPP_ERROR(logger_, "External control stopped due to an error: %s", reason.c_str());
  }

private:
  const rclcpp::Logger logger_;
  KukaMxaRsiHardwareInterface * hw_interface_;
};

class EventHandlerExtensionMxa : public kuka::external::control::kss::mxa::IEventHandlerExtension
{
public:
  explicit EventHandlerExtensionMxa(KukaMxaRsiHardwareInterface * hw_interface)
  : logger_(rclcpp::get_logger("KukaMxaRsiHardwareInterface")), hw_interface_(hw_interface)
  {
  }

  void OnConnected(const kuka::external::control::kss::mxa::InitializationData & init_data) override
  {
    hw_interface_->set_server_event(kuka_drivers_core::HardwareEvent::COMMAND_ACCEPTED);
    RCLCPP_INFO(logger_, "Client successfully established a connection to the mxAutomation server");
    hw_interface_->mxa_init(init_data);
  }

private:
  const rclcpp::Logger logger_;
  KukaMxaRsiHardwareInterface * hw_interface_;
};

class StatusUpdateHandler : public kuka::external::control::kss::mxa::IStatusUpdateHandler
{
public:
  StatusUpdateHandler(KukaMxaRsiHardwareInterface * hw_interface, StatusManager * status_manager)
  : hw_interface_{hw_interface}, status_manager_{status_manager}, first_update_{true}
  {
  }

  void OnStatusUpdateReceived(
    const kuka::external::control::kss::mxa::StatusUpdate & update) override
  {
    if (first_update_)
    {
      hw_interface_->initialize_command_interfaces(
        static_cast<kuka_drivers_core::ControlMode>(update.control_mode_), update.cycle_time_);
      first_update_ = false;
    }
    status_manager_->SetStatusInterfaces(update);
  }

private:
  KukaMxaRsiHardwareInterface * hw_interface_;
  StatusManager * status_manager_;
  bool first_update_;
};

}  // namespace kuka_rsi_driver

#endif  // KUKA_RSI_DRIVER__EVENT_OBSERVER_MXA_RSI_HPP_
