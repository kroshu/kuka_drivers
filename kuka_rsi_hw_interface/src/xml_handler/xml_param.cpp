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

#include <cstring>
#include <stdexcept>
#include <iostream>

#include "xml_handler/xml_param.hpp"

namespace xml
{
int XMLParam::PrintParam(char * & buffer_it, int & size_left)
{
  int idx = 0;
  switch (param_type_) {
    case XMLType::BOOL:
      idx = snprintf(buffer_it, size_left, "%u", GetParamValue<bool>());
      break;

    case XMLType::LONG:
      idx = snprintf(buffer_it, size_left, "%lu", GetParamValue<int64_t>());
      break;

    case XMLType::DOUBLE:
      idx = snprintf(buffer_it, size_left, "%f", GetParamValue<double>());
      break;

    case XMLType::STRING:
      idx = GetParamValue<XMLString>().PrintString(buffer_it, size_left);
      break;
    default:
      throw std::logic_error("Parameter type not supported");
      break;
  }
  if (idx < 0 || idx > size_left) {
    throw std::range_error("Out of the buffers range");
  } else {
    buffer_it += idx;
    size_left -= idx;
  }
  return idx;
}

std::ostream & operator<<(std::ostream & out, class XMLParam & param)
{
  std::visit([&](auto && arg) {out << arg;}, param.param_value_);
  return out;
}
}  // namespace xml
