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

#include "fri_client/friLBRCommand.h"

namespace KUKA
{
namespace FRI
{

void LBRCommand::setJointPosition(const double *values)
{
  (void)values;
  return;
}

void LBRCommand::setWrench(const double *wrench)
{
  (void)wrench;
  return;
}

void LBRCommand::setTorque(const double *torques)
{
  (void)torques;
  return;
}

void LBRCommand::setBooleanIOValue(const char *name, const bool value)
{
  (void)name;
  (void)value;
  return;
}

void LBRCommand::setDigitalIOValue(const char *name, const unsigned long long value)
{
  (void)name;
  (void)value;
  return;
}

void LBRCommand::setAnalogIOValue(const char *name, const double value)
{
  (void)name;
  (void)value;
  return;
}

}  // namespace FRI

}  // namespace KUKA
