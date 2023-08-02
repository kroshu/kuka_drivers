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
: state_data_structure_("Rob"), command_data_structure_("Sen"), err_buf_("")
{
  // Later this should be defined by the rsi xml
  state_data_structure_.GetParams()["TYPE"] = xml::XMLParam(xml::XMLType::STRING);
  // how to get string: std::get<XML::xml_string>(command_data_structure_.params_["TYPE"]).first
  xml::XMLElement Out("Out");
  Out.GetParams()["01"] = xml::XMLParam(xml::XMLType::BOOL);
  Out.GetParams()["02"] = xml::XMLParam(xml::XMLType::BOOL);
  Out.GetParams()["03"] = xml::XMLParam(xml::XMLType::BOOL);
  Out.GetParams()["04"] = xml::XMLParam(xml::XMLType::BOOL);
  Out.GetParams()["05"] = xml::XMLParam(xml::XMLType::BOOL);
  xml::XMLElement Override("Override");
  Override.GetParams()["Override"] = xml::XMLParam(xml::XMLType::LONG);
  state_data_structure_.GetChilds().emplace_back(Out);
  state_data_structure_.GetChilds().emplace_back(Override);

  command_data_structure_.GetParams()["Type"] = xml::XMLParam(xml::XMLType::STRING);
  xml::XMLElement RKorr("RKorr");
  RKorr.GetParams()["X"] = xml::XMLParam(xml::XMLType::DOUBLE);
  RKorr.GetParams()["Y"] = xml::XMLParam(xml::XMLType::DOUBLE);
  RKorr.GetParams()["Z"] = xml::XMLParam(xml::XMLType::DOUBLE);
  RKorr.GetParams()["A"] = xml::XMLParam(xml::XMLType::DOUBLE);
  RKorr.GetParams()["B"] = xml::XMLParam(xml::XMLType::DOUBLE);
  RKorr.GetParams()["C"] = xml::XMLParam(xml::XMLType::DOUBLE);
  xml::XMLElement DiO("DiO");
  DiO.GetParams()["DiO"] = xml::XMLParam(xml::XMLType::LONG);
  command_data_structure_.GetChilds().emplace_back(RKorr);
  command_data_structure_.GetChilds().emplace_back(DiO);
}

bool RSICommandHandler::Decode(char * const buffer, const size_t buffer_size)
{
  std::cout << "Decode started" << std::endl;
  auto buffer_it = buffer;
  try {
    decodeNode(state_data_structure_, buffer, buffer_it, buffer_size);
  } catch (const std::exception & e) {
    std::cout << "### ERROR ###" << std::endl;
    std::cerr << e.what() << '\n';
    return false;
  }
  std::cout << "Decode finished" << std::endl;
  return true;
}

bool RSICommandHandler::Encode(char * & buffer, const size_t buffer_size)
{
  std::cout << "Decode started" << std::endl;
  auto buffer_it = buffer;
  try {
    encodeNode(command_data_structure_, buffer, buffer_it, buffer_size);
  } catch (const std::exception & e) {
    std::cout << "### ERROR ###" << std::endl;
    std::cerr << e.what() << '\n';
    return false;
  }
  std::cout << "Decode finished" << std::endl;
  return true;
}

void RSICommandHandler::decodeNode(
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
    auto i = std::sprintf(err_buf_, "The detected name (");
    i = std::snprintf(err_buf_ + i, node_name.length_, "%s", node_name.data_ptr_);
    i = std::sprintf(err_buf_, ") does not match the elements name %s.", element.GetName().c_str());
    throw std::logic_error(err_buf_);
  }
  // char str[node_name.length_ + 1];
  // std::snprintf(str, node_name.length_ + 1, "%s", node_name.data_ptr_);
  // std::cout << "Name found: " << str << std::endl;

  // Validate nodes params
  size_t numOfParam = 0;
  bool isBaseLessNode = false;
  bool isNoMoreParam = false;
  xml::XMLString node_param;
  while (!isNoMoreParam && (size_t)(buffer_it - buffer) < buffer_size) {
    // fast forward to the next non-space character
    for (; *buffer_it == ' '; buffer_it++) {
    }
    if ((size_t)(buffer_it - buffer) >= buffer_size) {
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
        auto i = std::sprintf(err_buf_, "The detected parameter (");
        i = std::snprintf(err_buf_ + i, node_param.length_, "%s", node_name.data_ptr_);
        std::sprintf(
          err_buf_ + i, ") dose not match with any of the %s elements parameters.",
          element.GetName().c_str());
        throw std::logic_error(err_buf_);
      }
      for (; (size_t)(buffer_it - buffer) < buffer_size && *buffer_it != '"'; buffer_it++) {
      }
      if ((size_t)(buffer_it - buffer) >= buffer_size) {
        throw std::range_error("Out of the buffers range");
      }
      buffer_it++;
      if (!element.CastParam(node_param, buffer_it)) {
        auto i = std::sprintf(err_buf_, "The not cast the ");
        i = std::snprintf(err_buf_ + i, node_param.length_, "%s", node_param.data_ptr_);
        std::sprintf(
          err_buf_ + i, " param into the %s elements parameter list.",
          element.GetName().c_str());
        throw std::logic_error(err_buf_);
      }
      buffer_it++;
      numOfParam++;
    }
  }
  for (; (size_t)(buffer_it - buffer) < buffer_size && *buffer_it != '>'; buffer_it++) {
  }
  if ((size_t)(buffer_it - buffer) >= buffer_size) {
    throw std::range_error("Out of the buffers range");
  }
  buffer_it++;

  if (isBaseLessNode && numOfParam != element.GetParams().size()) {
    std::sprintf(
      err_buf_,
      "%lu parameter found it does not match with %s elements parameter list size.", numOfParam,
      element.GetName().c_str());
    throw std::logic_error(err_buf_);
  }
  // fast forward if
  for (; *buffer_it == ' '; buffer_it++) {
  }
  if ((size_t)(buffer_it - buffer) >= buffer_size) {
    throw std::range_error("Out of the buffers range");
  }
  if (!isBaseLessNode) {
    if (*buffer_it != '<') {
      // Node base is data
      if (!element.CastParam(node_name, buffer_it)) {
        auto i = std::sprintf(err_buf_, "Could not cast the ");
        i = std::snprintf(err_buf_ + i, node_param.length_, "%s", node_param.data_ptr_);
        std::sprintf(
          err_buf_ + i, " parameter into the %s elements parameter list",
          element.GetName().c_str());
        throw std::logic_error(err_buf_);
      }
    } else {
      // node base has childs
      for (auto && child : element.GetChilds()) {
        decodeNode(child, buffer, buffer_it, buffer_size);
      }
    }
    for (; (size_t)(buffer_it - buffer) < buffer_size && *buffer_it != '<'; buffer_it++) {
    }
    if ((size_t)(buffer_it - buffer) >= buffer_size) {
      throw std::range_error("Out of the buffers range");
    }
    if (*(buffer_it + 1) != '/') {
      std::sprintf(
        err_buf_,
        "Start of an end Node, where there should be none. Error came in the %s node",
        element.GetName().c_str());
      throw std::logic_error(err_buf_);
    }
    buffer_it += 2;
    if (!element.IsNameValid(node_name, buffer_it)) {
      auto i = std::sprintf(err_buf_, "The detected name (");
      i = std::snprintf(err_buf_ + i, node_name.length_, "%s", node_name.data_ptr_);
      std::sprintf(
        err_buf_ + i, ") does not match the elements name: %s",
        element.GetName().c_str());
      throw std::logic_error(err_buf_);
    }
  }
}

void RSICommandHandler::encodeNode(
  xml::XMLElement & element, char * const buffer, char * & buffer_it,
  const size_t buffer_size)
{
  auto idx = sprintf(buffer_it, "<%s ", element.GetName().c_str());
  if ((size_t)(buffer_it - buffer) + idx >= buffer_size) {
    throw std::range_error("Out of the buffers range");
  }
  buffer_it += idx;
  bool isBaseLessNode = false;
  if (element.GetChilds().size() <= 0) {
    isBaseLessNode = true;
  }
  std::cout << "Added params: " << std::endl;
  for (auto && param : element.GetParams()) {
    if (element.GetName() == param.first) {
      isBaseLessNode = false;
    } else {
      idx = sprintf(buffer_it, "%s=\"", param.first.c_str());
      if ((size_t)(buffer_it - buffer) + idx >= buffer_size) {
        throw std::range_error("Out of the buffers range");
      }
      buffer_it += idx;
      param.second.ParamSprint(buffer_it, buffer, buffer_size);
      std::cout << param.first << ", ";
      idx = sprintf(buffer_it, "\" ");
      if ((size_t)(buffer_it - buffer) + idx >= buffer_size) {
        throw std::range_error("Out of the buffers range");
      }
      buffer_it += idx;
    }
  }
  std::cout << std::endl;
  // if (element.GetChilds().size() <= 0 && isBaseLessNode == false) {
  //   std::sprintf(
  //     err_buf_, "The %s node has no chiled and has no base data",
  //     element.GetName().c_str());
  //   throw std::logic_error(err_buf_);
  // }
  if (isBaseLessNode) {
    idx = sprintf(buffer_it, "/>");
    if ((size_t)(buffer_it - buffer) + idx >= buffer_size) {
      throw std::range_error("Out of the buffers range");
    }
    buffer_it += idx;
  } else {
    buffer_it--;
    idx = sprintf(buffer_it, ">");
    if ((size_t)(buffer_it - buffer) + idx >= buffer_size) {
      throw std::range_error("Out of the buffers range");
    }
    buffer_it += idx;
    if (element.GetChilds().size() > 0) {
      // Add childs
      for (auto && child : element.GetChilds()) {
        encodeNode(child, buffer, buffer_it, buffer_size);
      }
    } else {
      // Add data
      element.GetParams().find(element.GetName())->second.ParamSprint(
        buffer_it, buffer,
        buffer_size);
    }
    // Add end bracket
    idx = sprintf(buffer_it, "</%s>", element.GetName().c_str());
    if ((size_t)(buffer_it - buffer) + idx >= buffer_size) {
      throw std::range_error("Out of the buffers range");
    }
    buffer_it += idx;
  }

}
}
