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

#include <vector>

#include "fri_client/friClientIf.h"
#include "fri_client/friTransformationClient.h"

namespace KUKA
{
namespace FRI
{

struct ClientData
{
};

TransformationClient::TransformationClient() :
    _data()
{
}

double TransformationClient::getSampleTime() const
{
  return 0.0;
}

EConnectionQuality TransformationClient::getConnectionQuality() const
{
  return POOR;
}

const std::vector<const char*>& TransformationClient::getRequestedTransformationIDs() const
{
  return std::vector<const char*>();
}

const unsigned int TransformationClient::getTimestampSec() const
{
  const unsigned int i = 0;
  return i;
}

const unsigned int TransformationClient::getTimestampNanoSec() const
{
  const unsigned int i = 0;
  return i;
}

void TransformationClient::setTransformation(const char *transformationID,
                                             const double transformationMatrix[3][4],
                                             unsigned int timeSec, unsigned int timeNanoSec)
{
  (void)transformationID;
  (void)transformationMatrix;
  (void)timeSec;
  (void)timeNanoSec;
  return;
}

void TransformationClient::setBooleanIOValue(const char *name, const bool value)
{
  (void)name;
  (void)value;
  return;
}

void TransformationClient::setDigitalIOValue(const char *name, const unsigned long long value)
{
  (void)name;
  (void)value;
  return;
}

void TransformationClient::setAnalogIOValue(const char *name, const double value)
{
  (void)name;
  (void)value;
  return;
}

bool TransformationClient::getBooleanIOValue(const char *name) const
{
  (void)name;
  return false;
}

unsigned long long TransformationClient::getDigitalIOValue(const char *name) const
{
  (void)name;
  return 0;
}

double TransformationClient::getAnalogIOValue(const char *name) const
{
  (void)name;
  return 0.0;
}

}  // namespace FRI

}  // namespace KUKA

