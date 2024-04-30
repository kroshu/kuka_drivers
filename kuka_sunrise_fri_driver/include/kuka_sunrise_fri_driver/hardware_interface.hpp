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

#ifndef KUKA_SUNRISE_FRI_DRIVER__HARDWARE_INTERFACE_HPP_
#define KUKA_SUNRISE_FRI_DRIVER__HARDWARE_INTERFACE_HPP_

#include <condition_variable>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

#include "hardware_interface/system_interface.hpp"
#include "kuka_drivers_core/control_mode.hpp"
#include "kuka_drivers_core/hardware_event.hpp"

#include "fri_client_sdk/HWIFClientApplication.hpp"
#include "fri_client_sdk/friClientIf.h"
#include "fri_client_sdk/friLBRClient.h"
#include "fri_client_sdk/friUdpConnection.h"
#include "kuka_sunrise_fri_driver/fri_connection.hpp"
#include "kuka_sunrise_fri_driver/visibility_control.h"

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace kuka_sunrise_fri_driver
{

enum class IOTypes
{
  ANALOG = 0,
  DIGITAL = 1,
  BOOLEAN = 2,
};

static std::unordered_map<std::string, IOTypes> const types = {
  {"analog", IOTypes::ANALOG}, {"digital", IOTypes::DIGITAL}, {"boolean", IOTypes::BOOLEAN}};

class KukaFRIHardwareInterface : public hardware_interface::SystemInterface,
                                 public KUKA::FRI::LBRClient
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(KukaFRIHardwareInterface)

  // Set UDP timeout to 10 ms to enable checking return value of client_app_read()
  KUKA_SUNRISE_FRI_DRIVER_PUBLIC KukaFRIHardwareInterface()
  : client_application_(udp_connection_, *this)
  {
  }
  KUKA_SUNRISE_FRI_DRIVER_PUBLIC CallbackReturn
  on_init(const hardware_interface::HardwareInfo & info) override;
  KUKA_SUNRISE_FRI_DRIVER_PUBLIC CallbackReturn
  on_configure(const rclcpp_lifecycle::State & previous_state) override;
  KUKA_SUNRISE_FRI_DRIVER_PUBLIC CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State & previous_state) override;
  KUKA_SUNRISE_FRI_DRIVER_PUBLIC CallbackReturn
  on_activate(const rclcpp_lifecycle::State & previous_state) override;
  KUKA_SUNRISE_FRI_DRIVER_PUBLIC CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  KUKA_SUNRISE_FRI_DRIVER_PUBLIC std::vector<hardware_interface::StateInterface>
  export_state_interfaces() override;
  KUKA_SUNRISE_FRI_DRIVER_PUBLIC std::vector<hardware_interface::CommandInterface>
  export_command_interfaces() override;

  KUKA_SUNRISE_FRI_DRIVER_PUBLIC hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
  KUKA_SUNRISE_FRI_DRIVER_PUBLIC hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
  KUKA_SUNRISE_FRI_DRIVER_PUBLIC void updateCommand(const rclcpp::Time & stamp);
  KUKA_SUNRISE_FRI_DRIVER_PUBLIC bool setReceiveMultiplier(int receive_multiplier);

  KUKA_SUNRISE_FRI_DRIVER_PUBLIC void waitForCommand() final;
  KUKA_SUNRISE_FRI_DRIVER_PUBLIC void command() final;

  class InvalidGPIOTypeException : public std::runtime_error
  {
  public:
    explicit InvalidGPIOTypeException(const std::string & gpio_type)
    : std::runtime_error(
        "GPIO type '" + gpio_type +
        "' is not supported, possible ones are 'analog', 'digital' or 'boolean'")
    {
    }
  };

private:
  KUKA_SUNRISE_FRI_DRIVER_LOCAL bool FRIConfigChanged();

  bool active_read_ = false;
  std::string controller_ip_;
  KUKA::FRI::UdpConnection udp_connection_ = KUKA::FRI::UdpConnection(10);
  KUKA::FRI::HWIFClientApplication client_application_;
  std::shared_ptr<FRIConnection> fri_connection_;
  rclcpp::Clock ros_clock_;

  // Command interface must be of type double, but controller can set only integers
  // this is a temporary solution, until runtime parameters are supported for hardware interfaces
  double control_mode_ = 0;  // default to undefined
  double receive_multiplier_ = 1;
  double send_period_ms_ = 10;
  int client_port_ = 30200;
  std::string client_ip_ = "0.0.0.0";
  int receive_counter_ = 0;
  bool torque_command_mode_ = false;

  int prev_period_ = 0;
  int prev_multiplier_ = 0;

  // State and command interfaces
  std::vector<double> hw_position_commands_;
  std::vector<double> hw_torque_commands_;
  std::vector<double> hw_stiffness_commands_;
  std::vector<double> hw_damping_commands_;

  std::vector<double> hw_position_states_;
  std::vector<double> hw_torque_states_;
  std::vector<double> hw_ext_torque_states_;

  double server_state_ = 0;

  std::mutex event_mutex_;

  kuka_drivers_core::HardwareEvent last_event_ =
    kuka_drivers_core::HardwareEvent::HARDWARE_EVENT_UNSPECIFIED;

  static const int TCP_SERVER_PORT = 30000;
  static const int DOF = 7;

  struct RobotState
  {
    double tracking_performance_ = 1;
    // Enum values represented with doubles (as all interfaces must be pointers to doubles)
    double session_state_ = 0;
    double connection_quality_ = 0;
    double command_mode_ = 0;
    double safety_state_ = 0;
    double control_mode_ = 0;
    double operation_mode_ = 0;
    double drive_state_ = 0;
    double overlay_type_ = 0;
  };

  RobotState robot_state_;

  void activateFrictionCompensation(double * values) const;
  void onError();

  KUKA_SUNRISE_FRI_DRIVER_LOCAL IOTypes getType(const std::string & type_string) const
  {
    auto it = types.find(type_string);
    if (it != types.end())
    {
      return it->second;
    }
    else
    {
      throw InvalidGPIOTypeException(type_string);
    }
  }

  class GPIOReader
  {
  public:
    double & getData() { return data_; }
    const std::string & getName() const { return name_; }
    GPIOReader(const std::string & name, IOTypes type, const KUKA::FRI::LBRState & state)
    : name_(name), type_(type), state_(state)
    {
    }
    void getValue()
    {
      switch (type_)
      {
        case IOTypes::ANALOG:
          data_ = state_.getAnalogIOValue(name_.c_str());
          break;
        case IOTypes::DIGITAL:
          data_ = static_cast<double>(state_.getDigitalIOValue(name_.c_str()));
          break;
        case IOTypes::BOOLEAN:
          data_ = state_.getBooleanIOValue(name_.c_str());
          break;
      }
    }

  private:
    const std::string name_;
    IOTypes type_;
    const KUKA::FRI::LBRState & state_;
    double data_;
  };

  class GPIOWriter
  {
  public:
    double & getData() { return data_; }
    const std::string & getName() const { return name_; }
    GPIOWriter(
      const std::string & name, IOTypes type, KUKA::FRI::LBRCommand & command, double initial_value)
    : name_(name), type_(type), command_(command), data_(initial_value)
    {
    }
    void setValue()
    {
      switch (type_)
      {
        case IOTypes::ANALOG:
          command_.setAnalogIOValue(name_.c_str(), data_);
          break;
        case IOTypes::DIGITAL:
          command_.setDigitalIOValue(name_.c_str(), static_cast<uint64_t>(data_));
          break;
        case IOTypes::BOOLEAN:
          command_.setBooleanIOValue(name_.c_str(), static_cast<bool>(data_));
          break;
      }
    }

  private:
    const std::string name_;
    IOTypes type_;
    KUKA::FRI::LBRCommand & command_;
    double data_;
  };

  std::vector<GPIOWriter> gpio_inputs_;
  std::vector<GPIOReader> gpio_outputs_;
};
}  // namespace kuka_sunrise_fri_driver

#endif  // KUKA_SUNRISE_FRI_DRIVER__HARDWARE_INTERFACE_HPP_
