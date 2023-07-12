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
RSICommandHandler::RSICommandHandler()
{
  // Later this should be defined by the rsi xml
  state_data_structure_.name_ = "Rob";
  state_data_structure_.params_["TYPE"] = xml::XMLParam(xml::XMLType::STRING);
  // how to get string: std::get<XML::xml_string>(command_data_structure_.params_["TYPE"]).first
  xml::XMLElement Out("Out");
  Out.params_["01"] = xml::XMLParam(xml::XMLType::BOOL);
  Out.params_["02"] = xml::XMLParam(xml::XMLType::BOOL);
  Out.params_["03"] = xml::XMLParam(xml::XMLType::BOOL);
  Out.params_["04"] = xml::XMLParam(xml::XMLType::BOOL);
  Out.params_["05"] = xml::XMLParam(xml::XMLType::BOOL);
  xml::XMLElement Override("Overrride");
  Override.params_["Override"] = xml::XMLParam(xml::XMLType::LONG);
  state_data_structure_.childs_.emplace_back(Out);
  state_data_structure_.childs_.emplace_back(Override);
}

bool RSICommandHandler::Decode(const char * const buffer, const size_t buffer_size)
{
  int buffer_idx = 0;
  detectNode(state_data_structure_, buffer, buffer_idx, buffer_size);
}

void RSICommandHandler::detectNode(
  xml::XMLElement & element, const char * const buffer, int & buffer_idx, const size_t buffer_size)
{
  xml::XMLString node_name(buffer, 0);
  // fast forward for the first open bracket
  for (; buffer_idx < buffer_size && *(buffer + buffer_idx) != '<'; buffer_idx++) {
  }
  if (buffer_idx < buffer_size) {
    // TODO (Komaromi): Do something when not valid
  }
  buffer_idx++;
  // Validate nodes name
  if (!element.IsNameValid(node_name, buffer, buffer_idx)) {
    // TODO (Komaromi): Do something when not valid
  }

  // Validate nodes params
  size_t numOfParam = 0;
  bool isBaseLessNode = false;
  bool isNoMoreParam = false;
  xml::XMLString node_param;
  while (!isNoMoreParam && buffer_idx < buffer_size) {
    // fast forward to the next non-space character
    for (; *(buffer + buffer_idx) == ' '; buffer_idx++) {
    }
    if (buffer_idx >= buffer_size) {
      // TODO (Komaromi): Do something when end of string
    }
    // Check for the nodes hadders end characters
    if (*(buffer + buffer_idx) == '/' && *(buffer + buffer_idx + 1) == '>') {
      isBaseLessNode = true;
      isNoMoreParam = true;
    } else if (*(buffer + buffer_idx) == '>') {
      isNoMoreParam = true;
    } else {
      // If not the end of the nodes hadder decode param
      if (!element.IsParamNameValid(node_param, buffer, buffer_idx)) {
        // TODO (Komaromi): Do something when not valid
      }
      for (; buffer_idx < buffer_size && *(buffer + buffer_idx) != '"'; buffer_idx++) {
      }
      if (buffer_idx >= buffer_size) {
        // TODO (Komaromi): Do something when end of string
      }
      buffer_idx++;
      if (!element.CastParam(node_param, buffer, buffer_idx)) {
        // TODO (Komaromi): Do something when not valid
      }
      buffer_idx++;
      numOfParam++;
    }
  }
  for (; buffer_idx < buffer_size && *(buffer + buffer_idx) != '>'; buffer_idx++) {
  }
  if (buffer_idx >= buffer_size) {
    // TODO (Komaromi): Do something when end of string
  }
  buffer_idx++;

  if (isBaseLessNode && numOfParam != element.params_.size()) {
    // TODO (Komaromi): Do something when not valid
  }
  // fast forward if
  for (; *(buffer + buffer_idx) == ' '; buffer_idx++) {
  }
  if (buffer_idx >= buffer_size) {
    // TODO (Komaromi): Do something when end of string
  }
  if (!isBaseLessNode) {
    if (*(buffer + buffer_idx) != '<') {
      // Node base is data
      if (!element.CastParam(node_name, buffer, buffer_idx)) {
        // TODO (Komaromi): Do something when not valid
      }
    } else {
      // node base has childs
      for (auto && child : element.childs_) {
        detectNode(child, buffer, buffer_idx, buffer_size);
      }
    }
    for (; buffer_idx < buffer_size && *(buffer + buffer_idx) != '<'; buffer_idx++) {// maybe use strstr()
    }
    if (buffer_idx >= buffer_size) {
      // TODO (Komaromi): Do something when end of string
    }
    if (*(buffer + buffer_idx + 1) != '/') {
      // TODO (Komaromi): Do something when wrong chield number
    }
    buffer_idx += 2;
    if (!element.IsNameValid(node_name, buffer, buffer_idx)) {
      // TODO (Komaromi): Do something when not valid
    }
  }


//########### Copy #####################


  // xml::XMLString node_name(buffer, 0);
  // // fast forward for the first open bracket
  // for (; **buffer_it != '\0' && **buffer_it != '<'; *buffer_it++) {
  // }
  // if (**buffer_it == '\0') {
  //   // TODO (Komaromi): Do something when end of string
  // }
  // *buffer_it++;
  // // Validate nodes name
  // if (!element.IsNameValid(node_name, buffer_it)) {
  //   // TODO (Komaromi): Do something when not valid
  // }

  // // Validate nodes params
  // size_t numOfParam = 0;
  // bool isBaseLessNode = false;
  // bool isNoMoreParam = false;
  // xml::XMLString node_param;
  // while (!isNoMoreParam && **buffer_it != '\0') {
  //   // fast forward to the next non-space character
  //   for (; **buffer_it == ' '; *buffer_it++) {
  //   }
  //   if (**buffer_it == '\0') {
  //     // TODO (Komaromi): Do something when end of string
  //   }
  //   // Check for the nodes hadders end characters
  //   if (**buffer_it == '/' && *(*buffer_it + 1) == '>') {
  //     isBaseLessNode = true;
  //     isNoMoreParam = true;
  //   } else if (**buffer_it == '>') {
  //     isNoMoreParam = true;
  //   } else {
  //     // If not the end of the nodes hadder decode param
  //     if (!element.IsParamNameValid(node_param, buffer_it)) {
  //       // TODO (Komaromi): Do something when not valid
  //     }
  //     for (; **buffer_it != '\0' && **buffer_it != '"'; *buffer_it++) {
  //     }
  //     if (**buffer_it == '\0') {
  //       // TODO (Komaromi): Do something when end of string
  //     }
  //     *buffer_it++;
  //     if (!element.CastParam(node_param, buffer_it)) {
  //       // TODO (Komaromi): Do something when not valid
  //     }
  //     *buffer_it++;
  //     numOfParam++;
  //   }
  // }
  // for (; **buffer_it != '\0' && **buffer_it != '>'; *buffer_it++) {
  // }
  // if (**buffer_it == '\0') {
  //   // TODO (Komaromi): Do something when end of string
  // }
  // *buffer_it++;

  // if (isBaseLessNode && numOfParam != element.params_.size()) {
  //   // TODO (Komaromi): Do something when not valid
  // }
  // // fast forward if
  // for (; **buffer_it == ' '; *buffer_it++) {
  // }
  // if (**buffer_it == '\0') {
  //   // TODO (Komaromi): Do something when end of string
  // }
  // if (!isBaseLessNode) {
  //   if (**buffer_it != '<') {
  //     // Node base is data
  //     if (!element.CastParam(node_name, buffer_it)) {
  //       // TODO (Komaromi): Do something when not valid
  //     }
  //   } else {
  //     // node base has childs
  //     for (auto && child : element.childs_) {
  //       detectNode(child, buffer_it);
  //     }
  //   }
  //   for (; **buffer_it != '\0' && **buffer_it != '<'; *buffer_it++) {// maybe use strstr()
  //   }
  //   if (**buffer_it == '\0') {
  //     // TODO (Komaromi): Do something when end of string
  //   }
  //   if (*(*buffer_it + 1) != '/') {
  //     // TODO (Komaromi): Do something when wrong chield number
  //   }
  //   *buffer_it += 2;
  //   if (!element.IsNameValid(node_name, buffer_it)) {
  //     // TODO (Komaromi): Do something when not valid
  //   }
  // }
}
}
