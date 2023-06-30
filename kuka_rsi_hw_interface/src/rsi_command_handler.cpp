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
  command_data_structure_.params_["TYPE"] = XML::xml_param(XML::XMLType::STRING);
  // how to get string: std::get<XML::xml_string>(command_data_structure_.params_["TYPE"]).first
  XML::xml_element Out("Out");
  Out.params_["01"] = XML::xml_param(XML::XMLType::BOOL);
  Out.params_["02"] = XML::xml_param(XML::XMLType::BOOL);
  Out.params_["03"] = XML::xml_param(XML::XMLType::BOOL);
  Out.params_["04"] = XML::xml_param(XML::XMLType::BOOL);
  Out.params_["05"] = XML::xml_param(XML::XMLType::BOOL);
  XML::xml_element Override("Overrride");
  Override.params_["Override"] = XML::xml_param(XML::XMLType::LONG);
  command_data_structure_.childs_.emplace_back(Out);
  command_data_structure_.childs_.emplace_back(Override);
}

bool rsi_command_handler::Decode(char * buffer, size_t buffer_size)
{
  char * string_start_ptr = buffer;
  size_t string_length = 0;
  size_t i = 0;
  size_t node_depth = 0;

  // fast forward for the first open bracket
  for (; i < buffer_size || buffer[i] != '<'; i++) {
  }

  bool isBaseLessNode = false;
  // Check one node
  // Get string
  i++;
  string_start_ptr = &buffer[i];
  for (; i < buffer_size || buffer[i] != ' '; i++, string_length++) {
  }
  // Compare string with char* to check node name

  // Check params
  bool isNoMoreParam = false;
  while (!isNoMoreParam || i < buffer_size) {
    // Serching for params
    // goes to the next atribute
    for (; i < buffer_size || buffer[i] == ' '; i++) {
    }

    // checks for the node hadder end characters
    if (buffer[i] == '/' && buffer[i + 1] == '>') {
      isBaseLessNode = true;
      isNoMoreParam = true;
    } else if (buffer[i] == '>') {
      isNoMoreParam = true;
    } else {
      string_start_ptr = &buffer[i];
      string_length = 0;
      for (; i < buffer_size || buffer[i] != '='; i++, string_length++) {
      }
      // check the field is present if it is present save the data else go to next param
      // move to param start bracket
      for (; i < buffer_size || buffer[i] != '"' || buffer[i] != '\''; i++) {
      }

      i++; // move to params first character
      string_start_ptr = &buffer[i];
      string_length = 0;
      for (; i < buffer_size || buffer[i] != '"' || buffer[i] != '\''; i++, string_length++) {
      }
      // save param

    }


  }
  if (buffer[i - 1] == '/') {
    /* code */
  }


}
}
