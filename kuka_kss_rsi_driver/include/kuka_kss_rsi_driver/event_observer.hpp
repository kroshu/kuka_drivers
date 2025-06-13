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

#ifndef KUKA_KSS_RSI_DRIVER__EVENT_OBSERVER_HPP_
#define KUKA_KSS_RSI_DRIVER__EVENT_OBSERVER_HPP_

#include <string>

#include "kuka_drivers_core/hardware_event.hpp"
#include "kuka_kss_rsi_driver/hardware_interface.hpp"

namespace kuka_kss_rsi_driver
{
class EventObserver : public kuka::external::control::EventHandler
{
public:
  explicit EventObserver(HardwareInterface * hw_interface) : hw_interface_(hw_interface) {}

  void OnSampling() override
  {
    hw_interface_->SetServerEvent(kuka_drivers_core::HardwareEvent::CONTROL_STARTED);
    RCLCPP_INFO(hw_interface_->GetLogger(), "External control is active");
  }

  void OnControlModeSwitch(const std::string &) override
  {
    hw_interface_->SetServerEvent(kuka_drivers_core::HardwareEvent::CONTROL_MODE_SWITCH);
    RCLCPP_INFO(hw_interface_->GetLogger(), "Control mode switch is in progress");
  }

  void OnStopped(const std::string &) override
  {
    hw_interface_->SetServerEvent(kuka_drivers_core::HardwareEvent::CONTROL_STOPPED);
    RCLCPP_INFO(hw_interface_->GetLogger(), "External control stopped");
  }

  void OnError(const std::string & reason) override
  {
    hw_interface_->SetServerEvent(kuka_drivers_core::HardwareEvent::ERROR);
    RCLCPP_ERROR(
      hw_interface_->GetLogger(), "External control stopped due to an error: %s", reason.c_str());
    hw_interface_->SetStopFlag();
  }

private:
  HardwareInterface * hw_interface_;
};

class EventHandlerExtension : public kuka::external::control::kss::IEventHandlerExtension
{
public:
  explicit EventHandlerExtension(HardwareInterface * hw_interface) : hw_interface_(hw_interface) {}

  void OnConnected(const kuka::external::control::kss::InitializationData & init_data) override
  {
    hw_interface_->SetServerEvent(kuka_drivers_core::HardwareEvent::COMMAND_ACCEPTED);
    hw_interface_->HandleInitialization(init_data);
  }

private:
  HardwareInterface * hw_interface_;
};

class StatusUpdateHandler : public kuka::external::control::kss::IStatusUpdateHandler
{
public:
  StatusUpdateHandler(HardwareInterface * hw_interface, StatusManager * status_manager)
  : hw_interface_{hw_interface}, status_manager_{status_manager}, first_update_{true}
  {
  }

  void OnStatusUpdateReceived(const kuka::external::control::kss::StatusUpdate & update) override
  {
    if (first_update_)
    {
      hw_interface_->InitializeCommandInterfaces(
        static_cast<kuka_drivers_core::ControlMode>(update.control_mode_), update.cycle_time_);
      first_update_ = false;
    }
    status_manager_->SetStatusInterfaces(update);
  }

private:
  HardwareInterface * hw_interface_;
  StatusManager * status_manager_;
  bool first_update_;
};

}  // namespace kuka_kss_rsi_driver

#endif  // KUKA_KSS_RSI_DRIVER__EVENT_OBSERVER_HPP_
