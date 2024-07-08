// Copyright 2022 Aron Svastits
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

#ifndef KUKA_KMR_IISY_EAC_DRIVER__HARDWARE_INTERFACE_HPP_
#define KUKA_KMR_IISY_EAC_DRIVER__HARDWARE_INTERFACE_HPP_


#include "kuka_iiqka_drivers_core/hardware_interface_base.hpp"

#include "kuka_kmr_iisy_eac_driver/visibility_control.h"
#include <geometry_msgs/msg/twist.hpp>
#include "geometry_msgs/msg/pose.hpp"
#include <tf2/LinearMath/Matrix3x3.h>

using hardware_interface::return_type;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace kuka_eac
{
class KukaKMRiisyEACHardwareInterface : public KukaEACHardwareInterfaceBase
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(KukaKMRiisyEACHardwareInterface)

  KUKA_KMR_IISY_EAC_DRIVER_PUBLIC CallbackReturn
  on_init(const hardware_interface::HardwareInfo & info) override;

  KUKA_KMR_IISY_EAC_DRIVER_PUBLIC std::vector<hardware_interface::StateInterface>
  export_state_interfaces() override;

  KUKA_KMR_IISY_EAC_DRIVER_PUBLIC std::vector<hardware_interface::CommandInterface>
  export_command_interfaces() override;

  KUKA_KMR_IISY_EAC_DRIVER_PUBLIC return_type
  read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  KUKA_KMR_IISY_EAC_DRIVER_PUBLIC return_type
  write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  KUKA_KMR_IISY_EAC_DRIVER_LOCAL bool SetupRobot();
  KUKA_KMR_IISY_EAC_DRIVER_LOCAL bool SetupQoS();

  std::vector<double> hw_twist_commands_;
  
  std::vector<double> hw_position_states_;
  std::vector<double> hw_twist_state_;
  std::vector<double> hw_pose_state_;

};
}  // namespace kuka_eac

#endif  // KUKA_KMR_IISY_EAC_DRIVER__HARDWARE_INTERFACE_HPP_
