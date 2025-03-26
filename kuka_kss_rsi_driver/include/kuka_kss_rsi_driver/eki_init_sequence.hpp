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

#include <hardware_interface/system_interface.hpp>

#include "kuka/external-control-sdk/kss/robot.h"

using AxisType = kuka::external::control::AxisType;
using InitializationData = kuka::external::control::InitializationData;

namespace kuka_kss_rsi_driver
{

struct InitSequenceReport
{
  bool ok;

  // This will only be filled out if something is non-compliant
  std::string reason;

  static constexpr size_t MAX_REASON_LENGTH = 128;
};

std::string AxisTypeToString(const AxisType axis_type)
{
  switch (axis_type)
  {
    case AxisType::LINEAR:
      return "prismatic";
    case AxisType::ROTATIONAL:
      return "revolute";
    case AxisType::ENDLESS:
      return "continuous";
    default:
      return "unkown";
  }
}

InitSequenceReport CheckInitDataCompliance(
  const hardware_interface::HardwareInfo & info, const InitializationData & init_data)
{
  // TODO: Also check model name, mechanical reduction and RPM values, once all are available

  if (info.joints.size() != init_data.GetTotalAxisCount())
  {
    char buffer[InitSequenceReport::MAX_REASON_LENGTH];
    sprintf(
      buffer, "Mismatch in axis count: Driver expects %ld, but EKI server reported %d",
      info.joints.size(), init_data.GetTotalAxisCount());
    return {false, buffer};
  }

  for (size_t i = 0; i < info.joints.size(); ++i)
  {
    std::string reported_type = AxisTypeToString(init_data.axis_type[i]);
    if (info.joints[i].type != reported_type)
    {
      char buffer[InitSequenceReport::MAX_REASON_LENGTH];
      sprintf(
        buffer, "Axis type mismatch at index %ld: Expected %s, but reported %s", i,
        info.joints[i].type.c_str(), reported_type.c_str());
      return {false, buffer};
    }
  }

  return {true, ""};
}

}  // namespace kuka_kss_rsi_driver
