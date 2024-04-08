// Copyright 2024 Márk Szitanics
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

#ifndef KUKA_IIQKA_EAC_DRIVER__EVENT_OBSERVER_HPP_
#define KUKA_IIQKA_EAC_DRIVER__EVENT_OBSERVER_HPP_

#include <string>
#include "rclcpp/macros.hpp"

#include "kuka/external-control-sdk/iiqka/sdk.h"
#include "kuka_drivers_core/hardware_event.hpp"
#include "kuka_iiqka_eac_driver/hardware_interface.hpp"

namespace kuka_eac
{
class KukaEACEventObserver : public kuka::external::control::EventHandler
{
public:
  explicit KukaEACEventObserver(KukaEACHardwareInterface * hw_interface)
  : hw_interface_(hw_interface)
  {
  }
  void OnSampling() override
  {
    hw_interface_->set_server_event(kuka_drivers_core::HardwareEvent::CONTROL_STARTED);
    RCLCPP_INFO(rclcpp::get_logger("KukaEACHardwareInterface"), "External control is active");
  }
  void OnControlModeSwitch(const std::string &) override
  {
    hw_interface_->set_server_event(kuka_drivers_core::HardwareEvent::CONTROL_MODE_SWITCH);
    RCLCPP_INFO(
      rclcpp::get_logger("KukaEACHardwareInterface"), "Control mode switch is in progress");
    hw_interface_->reset_cycle_count();
  }
  void OnStopped(const std::string &) override
  {
    hw_interface_->set_server_event(kuka_drivers_core::HardwareEvent::CONTROL_STOPPED);
    RCLCPP_INFO(rclcpp::get_logger("KukaEACHardwareInterface"), "External control finished");
    hw_interface_->set_stop_flag();
  }
  void OnError(const std::string & reason) override
  {
    hw_interface_->set_server_event(kuka_drivers_core::HardwareEvent::ERROR);
    RCLCPP_ERROR(
      rclcpp::get_logger("KukaEACHardwareInterface"), "External control stopped by an error");
    RCLCPP_ERROR(rclcpp::get_logger("KukaEACHardwareInterface"), reason.c_str());
    hw_interface_->set_stop_flag();
  }

private:
  KukaEACHardwareInterface * hw_interface_;
};

}  // namespace kuka_eac

#endif  // KUKA_IIQKA_EAC_DRIVER__EVENT_OBSERVER_HPP_
