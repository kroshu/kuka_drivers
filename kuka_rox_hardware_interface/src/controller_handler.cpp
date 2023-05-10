// Copyright 2023 Komáromi Sándor
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

#include "kuka_rox_hw_interface/controller_handler.hpp"

using namespace kuka::motion::external;  // NOLINT

namespace kuka_rox
{
controller_handler::controller_handler(std::vector<std::string> fixed_controllers)
{
  control_mode_map_.emplace(
    std::make_pair(
      ExternalControlMode::POSITION_CONTROL,
      std::vector<std::string>(POSITION_CONTROLLERS_SIZE)));
  control_mode_map_.emplace(
    std::make_pair(
      ExternalControlMode::JOINT_IMPEDANCE_CONTROL,
      std::vector<std::string>(IMPEDANCE_CONTROLLERS_SIZE)));
  control_mode_map_.emplace(
    std::make_pair(
      ExternalControlMode::TORQUE_CONTROL,
      std::vector<std::string>(TORQUE_CONTROLLERS_SIZE)));
}

bool controller_handler::Update_controller_name(
  const controller_handler::controller_type controller_type,
  const std::string & controller_name)
{


}

std::pair<std::vector<std::string>, std::vector<std::string>>
controller_handler::Get_new_controllers(
  kuka::motion::external::ExternalControlMode new_control_mode)
{

}


std::vector<std::string> controller_handler::Get_active_controllers()
{

}
}   // namespace kuka_rox
