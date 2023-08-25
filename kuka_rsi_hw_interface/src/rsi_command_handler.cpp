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
#include <clocale>

#include "kuka_rsi_hw_interface/rsi_command_handler.hpp"
#include "rclcpp/rclcpp.hpp"

namespace kuka_rsi_hw_interface
{
RSICommandHandler::RSICommandHandler()
: command_data_structure_("Sen"), state_data_structure_("Rob")
{
  // save previous locale to restore later;
  prev_locale_ = std::setlocale(LC_NUMERIC, nullptr);
  // State structure
  // TODO(Komaromi): Later this should be defined by the rsi xml
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
bool RSICommandHandler::SetLocale()
{
  return std::setlocale(LC_NUMERIC, "C");
}

bool RSICommandHandler::ResetLocale()
{
  return std::setlocale(LC_NUMERIC, prev_locale_.c_str());
}

bool RSICommandHandler::Decode(char * const buffer, const size_t buffer_size)
{
  auto buffer_it = buffer;
  try {
    decodeNodes(this->state_data_structure_, buffer, buffer_it, buffer_size);
    return true;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("CommandHandler"), "%s", e.what());
    ResetLocale();
    return false;
  }
}

int RSICommandHandler::Encode(char * & buffer, const size_t buffer_size)
{
  auto buffer_it = buffer;
  int size_left = buffer_size;
  try {
    encodeNodes(this->command_data_structure_, buffer_it, size_left);
    if (static_cast<int>(buffer_size) - size_left != static_cast<int>(buffer_it - buffer)) {
      throw std::range_error("Range error occured");
    }
    // +1 is for the \0 character
    return buffer_size - size_left + 1;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("CommandHandler"), "%s", e.what());
    ResetLocale();
    return -1;
  }
}

void RSICommandHandler::decodeNodes(
  xml::XMLElement & element, char * const buffer, char * & buffer_it,
  const size_t buffer_size)
{
  xml::XMLString node_name(buffer, 0);
  if (*buffer_it != '<') {
    std::stringstream err_ss;
    err_ss << "Syntax error at the start of " << element.GetName() << " node.";
    throw std::logic_error(err_ss.str());
  }
  buffer_it++;
  // Validate node name
  if (!element.IsNameValid(node_name, buffer_it)) {
    std::stringstream err_ss;
    err_ss << "The detected start node name \"" << node_name <<
      "\" does not match the elements name " <<
      element.GetName() << ".";
    throw std::logic_error(err_ss.str());
  }

  // Validate node parameters
  size_t num_of_param = 0;
  bool is_param_only_node = false;
  bool is_no_more_param = false;
  while (!is_no_more_param && static_cast<size_t>(buffer_it - buffer) < buffer_size) {
    // Go to the next non-space character
    decodeSkipSpaces(buffer_it, element, buffer, buffer_size);

    // Check for the start node's closing tag
    if (*buffer_it == '/' && *(buffer_it + 1) == '>') {
      is_param_only_node = true;
      is_no_more_param = true;
      buffer_it += 2;
    } else if (*buffer_it == '>') {
      is_no_more_param = true;
      buffer_it++;
    } else {
      // Decode parameter
      decodeParam(element, buffer_it);
      num_of_param++;
    }
  }
  if (static_cast<size_t>(buffer_it - buffer) >= buffer_size) {
    std::stringstream err_ss;
    err_ss << "Out of range error in " << element.GetName() << " node.";
    throw std::range_error(err_ss.str());
  }

  // Could not find all parameters, or found too much
  //   Checks if it is a parameter-only node, because the normal leaf node's data is stored within
  //   the element's parameters. So in case of a normal leaf node the num_of_param and the
  //   Params().size() would not match even for a valid xml
  if (is_param_only_node && num_of_param != element.Params().size()) {
    std::stringstream err_ss;
    err_ss << num_of_param << " parameter found. It does not match with " << element.GetName() <<
      "elements parameter list size.";
    throw std::logic_error(err_ss.str());
  }

  // Node should have childs, but did not found any
  if (is_param_only_node && element.GetChilds().size() > 0) {
    std::stringstream err_ss;
    err_ss << element.GetName() << " node should have chields but did not found any.";
    throw std::logic_error(err_ss.str());
  }
  // Go to the next node or the end of the node
  decodeSkipSpaces(buffer_it, element, buffer, buffer_size);

  // If a node is not an only-parameter node, than it has data or childs and needs an end node
  if (!is_param_only_node) {
    if (*buffer_it != '<') {
      // Node is a leaf node, checking its data
      decodeLeafNodeParamData(element, buffer_it, node_name);

    } else {
      // Node has childs, not a leaf node
      for (auto && child : element.Childs()) {
        decodeNodes(child, buffer, buffer_it, buffer_size);
      }
    }
    // Check for the end tag (</tag>)
    decodeNodeEnd(element, buffer_it);
  }
}

void RSICommandHandler::decodeParam(xml::XMLElement & element, char * & buffer_it)
{
  xml::XMLString node_param;
  // Validate parameter name
  if (!element.IsParamNameValid(node_param, buffer_it)) {
    std::stringstream err_ss;
    err_ss << "The detected parameter \"" << node_param <<
      "\" does not match any of the " << element.GetName() << " elements parameters.";
    throw std::logic_error(err_ss.str());
  }
  // Check xml syntax (parameter name must be followed by '=')
  if (*buffer_it != '=' && *(buffer_it + 1) != '"') {
    std::stringstream err_ss;
    err_ss << "In \"" << node_param << "\" param syntax error found.";
    throw std::logic_error(err_ss.str());
  }
  buffer_it += 2;
  // Cast parameter data
  if (!element.CastParamData(node_param, buffer_it)) {
    std::stringstream err_ss;
    err_ss << "Could not cast the \"" << node_param << "\" param into the " <<
      element.GetName() << " elements parameter list.";
    throw std::logic_error(err_ss.str());
  }
  // Check xml syntax (Parameter value must be inside quotes)
  if (*buffer_it != '"') {
    std::stringstream err_ss;
    err_ss << "In \"" << node_param << "\" param syntax error found.";
    throw std::logic_error(err_ss.str());
  }
  buffer_it++;
}

void RSICommandHandler::decodeLeafNodeParamData(
  xml::XMLElement & element, char * & buffer_it,
  xml::XMLString & param_name)
{
  if (!element.CastParamData(param_name, buffer_it)) {
    std::stringstream err_ss;
    err_ss << "Could not cast the \"" << param_name << "\" parameter into the " <<
      element.GetName() << " elements parameter list.";
    throw std::logic_error(err_ss.str());
  }
  if (*buffer_it != '<') {
    std::stringstream err_ss;
    err_ss << "In \"" << param_name << "\" param syntax error found.";
    throw std::logic_error(err_ss.str());
  }
}

void RSICommandHandler::decodeNodeEnd(xml::XMLElement & element, char * & buffer_it)
{
  xml::XMLString node_name;
  if (*buffer_it != '<' && *(buffer_it + 1) != '/') {
    std::stringstream err_ss;
    err_ss << "Syntax error at the end of " << element.GetName() << " node.";
    throw std::logic_error(err_ss.str());
  }
  buffer_it += 2;
  if (!element.IsNameValid(node_name, buffer_it)) {
    std::stringstream err_ss;
    err_ss << "The detected name \"" << node_name << "\" does not match the elements name: " <<
      element.GetName() << ".";
    throw std::logic_error(err_ss.str());
  }
  if (*buffer_it != '>') {
    std::stringstream err_ss;
    err_ss << "Syntax error at the end of " << element.GetName() << " node.";
    throw std::logic_error(err_ss.str());
  }
  buffer_it++;
}

void RSICommandHandler::decodeSkipSpaces(
  char * & buffer_it, const xml::XMLElement & element, char * const buffer,
  const size_t & buffer_size)
{
  for (; *buffer_it == ' '; buffer_it++) {
  }
  if (static_cast<size_t>(buffer_it - buffer) >= buffer_size) {
    std::stringstream err_ss;
    err_ss << "Out of range error in " << element.GetName() << " node.";
    throw std::range_error(err_ss.str());
  }
}

void RSICommandHandler::encodeNodes(
  xml::XMLElement & element, char * & buffer_it, int & size_left)
{
  // Start the node with its name
  auto idx = snprintf(buffer_it, size_left, "<%s", element.GetName().c_str());
  update_iterators(buffer_it, size_left, element, idx);

  bool is_param_only_node = false;
  // If the node does not have childs it is a parameter-only node
  if (element.GetChilds().size() == 0) {
    is_param_only_node = true;
  }
  for (auto && param : element.Params()) {
    // In the current implementation it is not supported, that a parameter has the same name as
    //  it's base node
    // Therefore if the element's name is equal to one of it's parameters, it can't be a
    //  parameter-only node
    if (element.GetName() == param.first) {
      is_param_only_node = false;
    } else {
      // Creating parameters
      // Add param name
      idx = snprintf(buffer_it, size_left, " %s=\"", param.first.c_str());
      update_iterators(buffer_it, size_left, element, idx);

      // Add param data
      param.second.PrintParam(buffer_it, size_left);
      idx = snprintf(buffer_it, size_left, "\"");
      update_iterators(buffer_it, size_left, element, idx);
    }
  }
  // Add start node end bracket
  if (is_param_only_node) {
    idx = snprintf(buffer_it, size_left, " />");

    update_iterators(buffer_it, size_left, element, idx);
  } else {
    idx = snprintf(buffer_it, size_left, ">");

    update_iterators(buffer_it, size_left, element, idx);
    if (element.GetChilds().size() > 0) {
      // Add childs
      for (auto && child : element.Childs()) {
        encodeNodes(child, buffer_it, size_left);
      }
    } else {
      // Add data
      element.Params().find(element.GetName())->second.PrintParam(
        buffer_it, size_left);
    }
    // Add node end tag
    idx = snprintf(buffer_it, size_left, "</%s>", element.GetName().c_str());
    update_iterators(buffer_it, size_left, element, idx);
  }
}

void RSICommandHandler::update_iterators(
  char * & buffer_it, int & buf_size_left, const xml::XMLElement & element,
  const int & buf_idx)
{
  if (buf_idx < 0 || buf_idx > buf_size_left) {
    std::stringstream err_ss;
    err_ss << "Out of range error in " << element.GetName() << " node.";
    throw std::range_error(err_ss.str());
  } else {
    buffer_it += buf_idx;
    buf_size_left -= buf_idx;
  }
}
}  // namespace kuka_rsi_hw_interface
