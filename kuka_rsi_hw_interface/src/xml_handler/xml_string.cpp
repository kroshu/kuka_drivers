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
#include <iostream>

#include "xml_handler/xml_string.hpp"

namespace xml
{
int XMLString::PrintString(char * & buffer_it, const int & size_left)
{
  if (static_cast<int>(length_) + 1 > size_left) {
    return -1;
  } else {
    snprintf(buffer_it, length_ + 1, "%s", data_ptr_);
    return length_;
  }
}

std::ostream & operator<<(std::ostream & out, XMLString & xml_str)
{
  for (size_t i = 0; i < xml_str.length_; i++) {
    out << xml_str.data_ptr_[i];
  }
  return out;
}
}  // namespace xml