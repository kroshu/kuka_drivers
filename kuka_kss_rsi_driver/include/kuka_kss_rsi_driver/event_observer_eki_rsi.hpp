// Copyright 2025 Kristóf Pásztor
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

#ifndef KUKA_KSS_RSI_DRIVER__EVENT_OBSERVER_EKI_RSI_HPP_
#define KUKA_KSS_RSI_DRIVER__EVENT_OBSERVER_EKI_RSI_HPP_

#include "kuka/external-control-sdk/kss/sdk.h"
#include "kuka_drivers_core/hardware_event.hpp"
#include "kuka_kss_rsi_driver/hardware_interface_eki_rsi.hpp"

namespace kuka_kss_rsi_driver
{
class KukaRSIEventObserver : public kuka::external::control::EventHandler
{
public:
  explicit KukaRSIEventObserver(KukaRSIHardwareInterface * hw_interface)
  : logger_(rclcpp::get_logger("KukaRSIHardwareInterface")),
    hw_interface_(hw_interface),
    control_started_(false),
    test_error_received_(false)
  {
  }

  void OnSampling() override
  {
    hw_interface_->set_server_event(kuka_drivers_core::HardwareEvent::CONTROL_STARTED);
    control_started_ = true;
    test_error_received_ = false;
    RCLCPP_INFO(logger_, "External control is active");
  }

  void OnControlModeSwitch(const std::string &) override
  {
    hw_interface_->set_server_event(kuka_drivers_core::HardwareEvent::CONTROL_MODE_SWITCH);
    control_started_ = false;
    test_error_received_ = false;
    RCLCPP_INFO(logger_, "Control mode switch is in progress");
  }

  void OnStopped(const std::string &) override
  {
    hw_interface_->set_server_event(kuka_drivers_core::HardwareEvent::CONTROL_STOPPED);
    RCLCPP_INFO(logger_, "External control stopped");
    hw_interface_->set_stop_flag();
  }

  void OnError(const std::string & reason) override
  {
    // The EKI server sends a test error
    // This has to be checked, but no reaction is necessary
    if (control_started_ && !test_error_received_)
    {
      RCLCPP_ERROR(logger_, "Test error event received!");
      test_error_received_ = true;
      return;
    }

    hw_interface_->set_server_event(kuka_drivers_core::HardwareEvent::ERROR);
    RCLCPP_ERROR(logger_, "External control stopped due to an error: %s", reason.c_str());
    hw_interface_->set_stop_flag();
  }

private:
  const rclcpp::Logger logger_;
  KukaRSIHardwareInterface * hw_interface_;
  bool control_started_;
  bool test_error_received_;
};
}  // namespace kuka_kss_rsi_driver

#endif  // KUKA_KSS_RSI_DRIVER__EVENT_OBSERVER_EKI_RSI_HPP_
