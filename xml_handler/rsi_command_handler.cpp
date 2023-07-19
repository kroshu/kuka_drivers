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
#include <cstring>
#include <stdexcept>
#include <iostream>

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
  xml::XMLElement Override("Override");
  Override.params_["Override"] = xml::XMLParam(xml::XMLType::LONG);
  state_data_structure_.childs_.emplace_back(Out);
  state_data_structure_.childs_.emplace_back(Override);
}

bool RSICommandHandler::Decode(char * const buffer, const size_t buffer_size)
{
  std::cout << "Decode started" << std::endl;
  auto buffer_it = buffer;
  try {
    detectNode(state_data_structure_, buffer, buffer_it, buffer_size);
  } catch (const std::exception & e) {
    std::cout << "### ERROR ###" << std::endl;
    std::cerr << e.what() << '\n';
    return false;
  }
  std::cout << "Decode finished" << std::endl;
  return true;
}

void RSICommandHandler::detectNode(
  xml::XMLElement & element, char * const buffer, char * & buffer_it,
  const size_t buffer_size)
{
  xml::XMLString node_name(buffer, 0);
  // fast forward for the first open bracket
  for (; *buffer_it != '<'; buffer_it++) {
    if ((int)(buffer_it - buffer) >= (int)buffer_size) {
      throw std::range_error("Out of the buffers range");
    }
  }
  buffer_it++;
  // Validate nodes name
  if (!element.IsNameValid(node_name, buffer_it)) {
    char err_msg[150] = "";
    strcat(err_msg, "The detected name (");
    if (node_name.length_ > 0) {
      strcat(err_msg, node_name.c_str());
    }
    strcat(err_msg, ") does not match the elements name ");
    strcat(err_msg, element.name_.c_str());
    throw std::logic_error(err_msg);
  }
  std::cout << "Name found: " << node_name.c_str() << std::endl;

  // Validate nodes params
  size_t numOfParam = 0;
  bool isBaseLessNode = false;
  bool isNoMoreParam = false;
  xml::XMLString node_param;
  while (!isNoMoreParam && buffer_it - buffer < buffer_size) {
    // fast forward to the next non-space character
    for (; *buffer_it == ' '; buffer_it++) {
    }
    if (buffer_it - buffer >= buffer_size) {
      throw std::range_error("Out of the buffers range");
    }
    // Check for the nodes hadders end characters
    if (*buffer_it == '/' && *(buffer_it + 1) == '>') {
      isBaseLessNode = true;
      isNoMoreParam = true;
    } else if (*buffer_it == '>') {
      isNoMoreParam = true;
    } else {
      if (!element.IsParamNameValid(node_param, buffer_it)) {
        char err_msg[100] = "";
        strcat(err_msg, "The detected parameter (");
        strcat(err_msg, node_param.c_str());
        strcat(err_msg, ") dose not match any of the ");
        strcat(err_msg, element.name_.c_str());
        strcat(err_msg, "elements parameters");
        throw std::logic_error(err_msg);
      }
      for (; buffer_it - buffer < buffer_size && *buffer_it != '"'; buffer_it++) {
      }
      if (buffer_it - buffer >= buffer_size) {
        throw std::range_error("Out of the buffers range");
      }
      buffer_it++;
      if (!element.CastParam(node_param, buffer_it)) {
        char err_msg[100] = "";
        strcat(err_msg, "The not cast the ");
        strcat(err_msg, node_param.c_str());
        strcat(err_msg, " param into the ");
        strcat(err_msg, element.name_.c_str());
        strcat(err_msg, " elements parameter list");
        throw std::logic_error(err_msg);
      }
      buffer_it++;
      numOfParam++;
    }
  }
  for (; buffer_it - buffer < buffer_size && *buffer_it != '>'; buffer_it++) {
  }
  if (buffer_it - buffer >= buffer_size) {
    throw std::range_error("Out of the buffers range");
  }
  buffer_it++;

  if (isBaseLessNode && numOfParam != element.params_.size()) {
    char err_msg[100] = "";
    strcat(err_msg, "The number of parameters found ");
    strcat(err_msg, std::to_string(numOfParam).c_str());
    strcat(err_msg, " param into the ");
    strcat(err_msg, element.name_.c_str());
    strcat(err_msg, " elements parameter list");
    throw std::logic_error(err_msg);
  }
  // fast forward if
  for (; *buffer_it == ' '; buffer_it++) {
  }
  if (buffer_it - buffer >= buffer_size) {
    throw std::range_error("Out of the buffers range");
  }
  if (!isBaseLessNode) {
    if (*buffer_it != '<') {
      // Node base is data
      if (!element.CastParam(node_name, buffer_it)) {
        char err_msg[100] = "";
        strcat(err_msg, "The not cast the ");
        strcat(err_msg, node_param.c_str());
        strcat(err_msg, " param into the ");
        strcat(err_msg, element.name_.c_str());
        strcat(err_msg, " elements parameter list");
        throw std::logic_error(err_msg);
      }
    } else {
      // node base has childs
      for (auto && child : element.childs_) {
        detectNode(child, buffer, buffer_it, buffer_size);
      }
    }
    for (; buffer_it - buffer < buffer_size && *buffer_it != '<'; buffer_it++) {// maybe use strstr()
    }
    if (buffer_it - buffer >= buffer_size) {
      throw std::range_error("Out of the buffers range");
    }
    if (*(buffer_it + 1) != '/') {
      char err_msg[100] = "";
      strcat(err_msg, "Start of an end Node, where there should be none. Error came in the ");
      strcat(err_msg, element.name_.c_str());
      strcat(err_msg, " node.");
      throw std::logic_error(err_msg);
    }
    buffer_it += 2;
    if (!element.IsNameValid(node_name, buffer_it)) {
      char err_msg[100] = "";
      strcat(err_msg, "The detected name (");
      strcat(err_msg, node_name.c_str());
      strcat(err_msg, ") does not match the elements name ");
      strcat(err_msg, element.name_.c_str());
      throw std::logic_error(err_msg);
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
// int main()
// {
//   RSICommandHandler commandHandler();
//   char xml_state[1024] =
//   {
//     "<Rob TYPE=\"KUKA\"><Out 01=\"0\" 02=\"1\" 03=\"0\" 04=\"0\" 05=\"1\"/><Override></Override></Rob>"};
//   commandHandler.Decode();
// }
}
// // Later this should be defined by the rsi xml
// state_data_structure_.name_ = "Rob";
// state_data_structure_.params_["TYPE"] = xml::XMLParam(xml::XMLType::STRING);
// // how to get string: std::get<XML::xml_string>(command_data_structure_.params_["TYPE"]).first
// xml::XMLElement Out("Out");
// Out.params_["01"] = xml::XMLParam(xml::XMLType::BOOL);
// Out.params_["02"] = xml::XMLParam(xml::XMLType::BOOL);
// Out.params_["03"] = xml::XMLParam(xml::XMLType::BOOL);
// Out.params_["04"] = xml::XMLParam(xml::XMLType::BOOL);
// Out.params_["05"] = xml::XMLParam(xml::XMLType::BOOL);
// xml::XMLElement Override("Overrride");
// Override.params_["Override"] = xml::XMLParam(xml::XMLType::LONG);
// state_data_structure_.childs_.emplace_back(Out);
// state_data_structure_.childs_.emplace_back(Override);
