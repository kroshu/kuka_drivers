// Copyright 2023 Aron Svastits
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

#ifndef KUKA_DRIVERS_CORE__HARDWARE_INTERFACE_TYPES_HPP_
#define KUKA_DRIVERS_CORE__HARDWARE_INTERFACE_TYPES_HPP_

namespace hardware_interface
{
/* Custom interfaces */
// Constant defining stiffness interface
static constexpr char HW_IF_STIFFNESS[] = "stiffness";
// Constant defining damping interface
static constexpr char HW_IF_DAMPING[] = "damping";
// Constant defining external torque interface
static constexpr char HW_IF_EXTERNAL_TORQUE[] = "external_torque";

// Constand defining cartesian coordinate interfaces
static constexpr char HW_IF_X[] = "x";
static constexpr char HW_IF_Y[] = "y";
static constexpr char HW_IF_Z[] = "z";
static constexpr char HW_IF_A[] = "a";
static constexpr char HW_IF_B[] = "b";
static constexpr char HW_IF_C[] = "c";
static constexpr char HW_IF_QX[] = "qx";
static constexpr char HW_IF_QY[] = "qy";
static constexpr char HW_IF_QZ[] = "qz";
static constexpr char HW_IF_QW[] = "qw";
static constexpr char HW_IF_CART_PREFIX[] = "dummy_cart_joint";

/* Interface prefixes */
// Constant defining prefix for I/O interfaces
static constexpr char IO_PREFIX[] = "gpio";
// Constant defining prefix for interfaces of "configuration controllers"
static constexpr char CONFIG_PREFIX[] = "runtime_config";
// Constant defining prefix for fri state
static constexpr char FRI_STATE_PREFIX[] = "fri_state";
// Constant defining prefix for states
static constexpr char STATE_PREFIX[] = "state";
// Constant defining wrench interface prefixes
static constexpr char HW_IF_WRENCH_PREFIX[] = "wrench";
static constexpr char HW_IF_FORCE_PREFIX[] = "force";
static constexpr char HW_IF_TORQUE_PREFIX[] = "torque";
/* Configuration interfaces */
// Constant defining control_mode configuration interface
static constexpr char CONTROL_MODE[] = "control_mode";
// Constant defining the receive multiplier interface needed for FRI
static constexpr char RECEIVE_MULTIPLIER[] = "receive_multiplier";
static constexpr char SEND_PERIOD[] = "send_period_ms";

/* FRI state interfaces */
static constexpr char SESSION_STATE[] = "session_state";
static constexpr char CONNECTION_QUALITY[] = "connection_quality";
static constexpr char SAFETY_STATE[] = "safety_state";
static constexpr char COMMAND_MODE[] = "command_mode";
static constexpr char OPERATION_MODE[] = "operation_mode";
static constexpr char DRIVE_STATE[] = "drive_state";
static constexpr char OVERLAY_TYPE[] = "overlay_type";
static constexpr char TRACKING_PERFORMANCE[] = "tracking_performance";

// Constant defining server_state interface necessary for event broadcasting
static constexpr char SERVER_STATE[] = "server_state";

}  // namespace hardware_interface

#endif  // KUKA_DRIVERS_CORE__HARDWARE_INTERFACE_TYPES_HPP_
