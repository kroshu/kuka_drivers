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

#include "kuka_rsi_hw_interface/rsi_command_handler.hpp"
#include "rclcpp/rclcpp.hpp"

namespace kuka_rsi_hw_interface
{
RSICommandHandler::RSICommandHandler()
: command_data_structure_("Sen"), state_data_structure_("Rob"), err_buf_("")
{
  // State structure
  // Later this should be defined by the rsi xml
  state_data_structure_.Params()["TYPE"] = xml::XMLParam(xml::XMLType::STRING);
  // how to get string: std::get<XML::xml_string>(command_data_structure_.params_["TYPE"]).first
  xml::XMLElement RIst_el("RIst");
  RIst_el.Params()["X"] = xml::XMLParam(xml::XMLType::DOUBLE);
  RIst_el.Params()["Y"] = xml::XMLParam(xml::XMLType::DOUBLE);
  RIst_el.Params()["Z"] = xml::XMLParam(xml::XMLType::DOUBLE);
  RIst_el.Params()["A"] = xml::XMLParam(xml::XMLType::DOUBLE);
  RIst_el.Params()["B"] = xml::XMLParam(xml::XMLType::DOUBLE);
  RIst_el.Params()["C"] = xml::XMLParam(xml::XMLType::DOUBLE);
  xml::XMLElement RSol_el("RSol");
  RSol_el.Params()["X"] = xml::XMLParam(xml::XMLType::DOUBLE);
  RSol_el.Params()["Y"] = xml::XMLParam(xml::XMLType::DOUBLE);
  RSol_el.Params()["Z"] = xml::XMLParam(xml::XMLType::DOUBLE);
  RSol_el.Params()["A"] = xml::XMLParam(xml::XMLType::DOUBLE);
  RSol_el.Params()["B"] = xml::XMLParam(xml::XMLType::DOUBLE);
  RSol_el.Params()["C"] = xml::XMLParam(xml::XMLType::DOUBLE);
  xml::XMLElement AIPos_el("AIPos");
  AIPos_el.Params()["A1"] = xml::XMLParam(xml::XMLType::DOUBLE);
  AIPos_el.Params()["A2"] = xml::XMLParam(xml::XMLType::DOUBLE);
  AIPos_el.Params()["A3"] = xml::XMLParam(xml::XMLType::DOUBLE);
  AIPos_el.Params()["A4"] = xml::XMLParam(xml::XMLType::DOUBLE);
  AIPos_el.Params()["A5"] = xml::XMLParam(xml::XMLType::DOUBLE);
  AIPos_el.Params()["A6"] = xml::XMLParam(xml::XMLType::DOUBLE);
  xml::XMLElement ASPos_el("ASPos");
  ASPos_el.Params()["A1"] = xml::XMLParam(xml::XMLType::DOUBLE);
  ASPos_el.Params()["A2"] = xml::XMLParam(xml::XMLType::DOUBLE);
  ASPos_el.Params()["A3"] = xml::XMLParam(xml::XMLType::DOUBLE);
  ASPos_el.Params()["A4"] = xml::XMLParam(xml::XMLType::DOUBLE);
  ASPos_el.Params()["A5"] = xml::XMLParam(xml::XMLType::DOUBLE);
  ASPos_el.Params()["A6"] = xml::XMLParam(xml::XMLType::DOUBLE);
  xml::XMLElement Delay_el("Delay");
  Delay_el.Params()["D"] = xml::XMLParam(xml::XMLType::LONG);
  xml::XMLElement Ipoc_state_el("IPOC");
  Ipoc_state_el.Params()["IPOC"] = xml::XMLParam(xml::XMLType::LONG);
  state_data_structure_.Childs().emplace_back(RIst_el);
  state_data_structure_.Childs().emplace_back(RSol_el);
  state_data_structure_.Childs().emplace_back(AIPos_el);
  state_data_structure_.Childs().emplace_back(ASPos_el);
  state_data_structure_.Childs().emplace_back(Delay_el);
  state_data_structure_.Childs().emplace_back(Ipoc_state_el);

  // Command structure
  command_data_structure_.Params()["Type"] = xml::XMLParam(xml::XMLType::STRING);
  command_data_structure_.SetParam<xml::XMLString>("Type", xml::XMLString("KROSHU"));
  xml::XMLElement ak_el("AK");
  ak_el.Params()["A1"] = xml::XMLParam(xml::XMLType::DOUBLE);
  ak_el.Params()["A2"] = xml::XMLParam(xml::XMLType::DOUBLE);
  ak_el.Params()["A3"] = xml::XMLParam(xml::XMLType::DOUBLE);
  ak_el.Params()["A4"] = xml::XMLParam(xml::XMLType::DOUBLE);
  ak_el.Params()["A5"] = xml::XMLParam(xml::XMLType::DOUBLE);
  ak_el.Params()["A6"] = xml::XMLParam(xml::XMLType::DOUBLE);
  xml::XMLElement Stop_el("Stop");
  Stop_el.Params()["Stop"] = xml::XMLParam(xml::XMLType::BOOL);
  xml::XMLElement Ipoc_command_el("IPOC");
  Ipoc_command_el.Params()["IPOC"] = xml::XMLParam(xml::XMLType::LONG);
  command_data_structure_.Childs().emplace_back(ak_el);
  command_data_structure_.Childs().emplace_back(Stop_el);
  command_data_structure_.Childs().emplace_back(Ipoc_command_el);
}

bool RSICommandHandler::Decode(char * const buffer, const size_t buffer_size)
{
  auto buffer_it = buffer;
  try {
    decodeNode(this->state_data_structure_, buffer, buffer_it, buffer_size);
    return true;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("CommandHandler"), "%s", e.what());
    return false;
  }
}

int RSICommandHandler::Encode(char * & buffer, const size_t buffer_size)
{
  auto buffer_it = buffer;
  int size_left = buffer_size;
  try {
    encodeNode(this->command_data_structure_, buffer_it, size_left);
    if (buffer_size - size_left != (size_t)(buffer_it - buffer)) {
      throw std::range_error("Range error occured");
    }
    // +1 is for the \0 character
    return buffer_size - size_left + 1;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("CommandHandler"), "%s", e.what());
    return -1;
  }
}

void RSICommandHandler::decodeNode(
  xml::XMLElement & element, char * const buffer, char * & buffer_it,
  const size_t buffer_size)
{
  xml::XMLString node_name(buffer, 0);
  if (*(buffer_it) != '<') {
    std::sprintf(
      err_buf_,
      "Syntax error at the start of %s node",
      element.GetName().c_str());
    throw std::logic_error(err_buf_);
  }
  buffer_it++;
  // Validate nodes name
  if (!element.IsNameValid(node_name, buffer_it)) {
    auto i = std::sprintf(err_buf_, "The detected name (");
    auto err_buff_it = err_buf_ + i;
    i += node_name.PrintString(err_buff_it, ERROR_BUFFER_SIZE - i);
    std::sprintf(
      err_buf_ + i, ") does not match the elements name %s.",
      element.GetName().c_str());
    throw std::logic_error(err_buf_);
  }

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
    // Check for the nodes headers end characters
    if (*buffer_it == '/' && *(buffer_it + 1) == '>') {
      isBaseLessNode = true;
      isNoMoreParam = true;
      buffer_it += 2;
    } else if (*buffer_it == '>') {
      isNoMoreParam = true;
      buffer_it++;
    } else {
      if (!element.IsParamNameValid(node_param, buffer_it)) {
        auto i = std::sprintf(err_buf_, "The detected parameter (");
        auto err_buff_it = err_buf_ + i;
        i += node_param.PrintString(err_buff_it, ERROR_BUFFER_SIZE - i);
        std::sprintf(
          err_buf_ + i, ") does not match with any of the %s elements parameters.",
          element.GetName().c_str());
        throw std::logic_error(err_buf_);
      }
      if (*buffer_it != '=') {
        auto i = sprintf(err_buf_, "In \"");
        auto err_buff_it = err_buf_ + i;
        i += node_param.PrintString(err_buff_it, ERROR_BUFFER_SIZE - i);
        std::sprintf(err_buf_ + i, "\" param syntax error found");
        throw std::logic_error(err_buf_);
      }
      buffer_it++;
      if (*buffer_it != '"') {
        auto i = sprintf(err_buf_, "In \"");
        auto err_buff_it = err_buf_ + i;
        i += node_param.PrintString(err_buff_it, ERROR_BUFFER_SIZE - i);
        std::sprintf(err_buf_ + i, "\" param syntax error found");
        throw std::logic_error(err_buf_);
      }
      buffer_it++;
      if (!element.CastParam(node_param, buffer_it)) {
        auto i = std::sprintf(err_buf_, "Could not cast the ");
        auto err_buff_it = err_buf_ + i;
        i += node_param.PrintString(err_buff_it, ERROR_BUFFER_SIZE - i);
        std::sprintf(
          err_buf_ + i, " param into the %s elements parameter list.",
          element.GetName().c_str());
        throw std::logic_error(err_buf_);
      }
      if (*buffer_it != '"') {
        auto i = sprintf(err_buf_, "In \"");
        auto err_buff_it = err_buf_ + i;
        i += node_param.PrintString(err_buff_it, ERROR_BUFFER_SIZE - i);
        std::sprintf(err_buf_ + i, "\" param syntax error found");
        throw std::logic_error(err_buf_);
      }
      buffer_it++;
      numOfParam++;
    }
  }
  if ((size_t)(buffer_it - buffer) >= buffer_size) {
    throw std::range_error("Out of the buffers range");
  }

  // Could not find all parameter, or found too much
  if (isBaseLessNode && numOfParam != element.Params().size()) {
    std::sprintf(
      err_buf_,
      "%lu parameter found. It does not match with %s elements parameter list size.", numOfParam,
      element.GetName().c_str());
    throw std::logic_error(err_buf_);
  }

  // Node should have childs, but did not found any
  if (isBaseLessNode && element.GetChilds().size() > 0) {
    std::sprintf(
      err_buf_, "%s node should have chields but did not found any",
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
        auto err_buff_it = err_buf_ + i;
        i += node_name.PrintString(err_buff_it, ERROR_BUFFER_SIZE - i);
        std::sprintf(
          err_buf_ + i, " parameter into the %s elements parameter list",
          element.GetName().c_str());
        throw std::logic_error(err_buf_);
      }
      if (*buffer_it != '<') {
        auto i = sprintf(err_buf_, "In \"");
        auto err_buff_it = err_buf_ + i;
        i += node_name.PrintString(err_buff_it, ERROR_BUFFER_SIZE - i);
        std::sprintf(err_buf_ + i, "\" param syntax error found");
        throw std::logic_error(err_buf_);
      }
    } else {
      // node base has childs
      for (auto && child : element.Childs()) {
        decodeNode(child, buffer, buffer_it, buffer_size);
      }
    }
    if (*(buffer_it) != '<') {
      std::sprintf(
        err_buf_,
        "Syntax error at the end of %s node",
        element.GetName().c_str());
      throw std::logic_error(err_buf_);
    }
    buffer_it++;
    if (*(buffer_it) != '/') {
      std::sprintf(
        err_buf_,
        "Syntax error at the end of %s node",
        element.GetName().c_str());
      throw std::logic_error(err_buf_);
    }
    buffer_it++;
    if (!element.IsNameValid(node_name, buffer_it)) {
      auto i = std::sprintf(err_buf_, "The detected name (");
      auto err_buff_it = err_buf_ + i;
      i += node_name.PrintString(err_buff_it, ERROR_BUFFER_SIZE - i);
      std::sprintf(
        err_buf_ + i, ") does not match the elements name: %s",
        element.GetName().c_str());
      throw std::logic_error(err_buf_);
    }
    if (*buffer_it != '>') {
      std::sprintf(
        err_buf_,
        "Syntax error at the end of %s node",
        element.GetName().c_str());
      throw std::logic_error(err_buf_);
    }
    buffer_it++;
  }
}

void RSICommandHandler::encodeNode(
  xml::XMLElement & element, char * & buffer_it, int & size_left)
{
  auto idx = snprintf(buffer_it, size_left, "<%s", element.GetName().c_str());
  if (idx < 0 || idx > size_left) {
    throw std::range_error("Out of the buffers range");
  } else {
    buffer_it += idx;
    size_left -= idx;
  }
  bool isBaseLessNode = false;
  if (element.GetChilds().size() == 0) {
    isBaseLessNode = true;
  }
  for (auto && param : element.Params()) {
    if (element.GetName() == param.first) {
      isBaseLessNode = false;
    } else {
      idx = snprintf(buffer_it, size_left, " %s=\"", param.first.c_str());
      if (idx < 0 || idx > size_left) {
        throw std::range_error("Out of the buffers range");
      } else {
        buffer_it += idx;
        size_left -= idx;
      }
      param.second.PrintParam(buffer_it, size_left);

      idx = snprintf(buffer_it, size_left, "\"");
      if (idx < 0 || idx > size_left) {
        throw std::range_error("Out of the buffers range");
      } else {
        buffer_it += idx;
        size_left -= idx;
      }
    }
  }
  if (isBaseLessNode) {
    idx = snprintf(buffer_it, size_left, " />");
    if (idx < 0 || idx > size_left) {
      throw std::range_error("Out of the buffers range");
    } else {
      buffer_it += idx;
      size_left -= idx;
    }
  } else {
    idx = snprintf(buffer_it, size_left, ">");
    if (idx < 0 || idx > size_left) {
      throw std::range_error("Out of the buffers range");
    } else {
      buffer_it += idx;
      size_left -= idx;
    }
    if (element.GetChilds().size() > 0) {
      // Add childs
      for (auto && child : element.Childs()) {
        encodeNode(child, buffer_it, size_left);
      }
    } else {
      // Add data
      element.Params().find(element.GetName())->second.PrintParam(
        buffer_it, size_left);
    }
    // Add end bracket
    idx = snprintf(buffer_it, size_left, "</%s>", element.GetName().c_str());
    if (idx < 0 || idx > size_left) {
      throw std::range_error("Out of the buffers range");
    } else {
      buffer_it += idx;
      size_left -= idx;
    }
  }
}
}
