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

#include <string>

#include "rsi_command_handler.hpp"

namespace kuka_rsi_hw_interface
{
rsi_command_handler::rsi_command_handler()
{
  // Later thi should be defined by the rsi xml
  command_data_structure_.name_ = "Rob";
  command_data_structure_.params_["TYPE"] = std::make_pair(nullptr, 0);
  // how to get string: std::get<XML_STRING_TYPE>(command_data_structure_.params_["TYPE"]).first
  XML::xml_element Out("Out");
  Out.params_.emplace("01");
  Out.params_.emplace("02");
  Out.params_.emplace("03");
  Out.params_.emplace("04");
  Out.params_.emplace("05");
  XML::xml_element Override("Overrride");
  command_data_structure_.childs_.emplace_back(Out);
  command_data_structure_.childs_.emplace_back(Override);
}

void rsi_command_handler::Decode(char * buffer, size_t buffer_size)
{
  char * string_start_ptr = buffer;
  size_t string_length = 0;
  size_t i = 0;

  // fast forward for the first open bracket
  for (; i < buffer_size || buffer[i] != '<'; i++) {
  }

  // Check one node
  // Get string
  string_start_ptr = &buffer[i + 1];
  for (; buffer[i] != ' '; i++, string_length++) {
  }

}
}
