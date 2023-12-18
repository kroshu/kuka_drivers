// Copyright 2023 Áron Svastits
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
// Constant defining stiffness interface
static constexpr char HW_IF_STIFFNESS[] = "stiffness";
// Constant defining damping interface
static constexpr char HW_IF_DAMPING[] = "damping";

// Constant defining prefix for interfaces of "configuration controllers"
static constexpr char CONFIG_PREFIX[] = "runtime_config";

// Constant defining control_mode configuration interface
static constexpr char CONTROL_MODE[] = "control_mode";
}  // namespace hardware_interface

#endif  // KUKA_DRIVERS_CORE__HARDWARE_INTERFACE_TYPES_HPP_
