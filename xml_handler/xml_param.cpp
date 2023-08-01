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

#include "xml_param.hpp"

namespace xml
{
size_t XMLParam::ParamSprint(char * & buffer_it, const size_t buffer_size)
{
  int idx = 0;
  switch (param_type_) {
    case XMLType::BOOL:
      idx = sprintf(buffer_it, "%u", std::get<0>(param_));
      break;

    case XMLType::LONG:
      idx = sprintf(buffer_it, "%lu", std::get<1>(param_));
      break;

    case XMLType::DOUBLE:
      idx = sprintf(buffer_it, "%f", std::get<2>(param_));
      break;

    case XMLType::STRING:
      idx = std::snprintf(
        buffer_it, std::get<3>(param_).length_, "%s",
        std::get<3>(param_).data_ptr_);
      break;
    default:
      throw std::logic_error("Parameter type not supported");
      break;
  }
  if ((size_t)buffer_it + idx >= buffer_size) {
    throw std::range_error("Out of the buffers range");
  }
  buffer_it += idx;
  return idx;
}
}
