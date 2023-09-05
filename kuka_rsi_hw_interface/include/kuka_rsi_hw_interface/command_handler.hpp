// Copyright 2023 Komaromi Sándor
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

#ifndef KUKA_RSI_HW_INTERFACE__COMMAND_HANDLER_HPP_
#define KUKA_RSI_HW_INTERFACE__COMMAND_HANDLER_HPP_

#include <pugixml.hpp>

class command_handler
{
private:
  pugi::xml_document state_doc;
  char * state_buffer;

public:
  command_handler(const size_t buffer_size);
  ~command_handler() = default;

  void Decode(const char * const buffer, const size_t buffer_size);
};


#endif  // KUKA_RSI_HW_INTERFACE__COMMAND_HANDLER_HPP_
