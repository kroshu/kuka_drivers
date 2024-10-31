// Copyright 2024 Aron Svastits
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

#ifndef KUKA_DRIVERS_CORE__HARDWARE_EVENT_HPP_
#define KUKA_DRIVERS_CORE__HARDWARE_EVENT_HPP_

namespace kuka_drivers_core
{
/**
 * @brief Enum for controller-side events
 */
enum class HardwareEvent : std::uint8_t
{
  HARDWARE_EVENT_UNSPECIFIED = 0,
  COMMAND_ACCEPTED = 2,
  CONTROL_STARTED = 3,
  CONTROL_STOPPED = 4,
  CONTROL_MODE_SWITCH = 5,
  ERROR = 6
};

}  // namespace kuka_drivers_core

#endif  // KUKA_DRIVERS_CORE__HARDWARE_EVENT_HPP_
