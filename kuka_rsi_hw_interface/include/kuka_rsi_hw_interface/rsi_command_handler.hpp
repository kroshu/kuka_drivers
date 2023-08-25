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

#ifndef  KUKA_RSI_HW_INTERFACE__RSI_COMMAND_HANDLER_HPP_
#define  KUKA_RSI_HW_INTERFACE__RSI_COMMAND_HANDLER_HPP_

#include <string>

#include "xml_handler/xml_element.hpp"

namespace kuka_rsi_hw_interface
{

class RSICommandHandler
{
public:
  RSICommandHandler();
  ~RSICommandHandler() = default;


  inline const xml::XMLElement & GetState() const {return state_data_structure_;}
  inline const xml::XMLElement & GetCommand() const {return command_data_structure_;}

  template<typename T>
  bool SetCommandParam(
    const std::string & elementName, const std::string & paramName,
    const T & param)
  {
    try {
      return command_data_structure_.Element(elementName)->SetParam<T>(paramName, param);
    } catch (const std::exception & e) {
      std::cerr << e.what() << '\n';
      return false;
    }
  }

  bool Decode(char * const buffer, const size_t buffer_size);
  int Encode(char * & buffer, const size_t buffer_size);
  bool SetLocale();
  bool ResetLocale();

private:
  xml::XMLElement command_data_structure_;
  xml::XMLElement state_data_structure_;

  std::string prev_locale_;

  // node decoder functions

  void decodeNodes(
    xml::XMLElement & element, char * const buffer, char * & buffer_it,
    const size_t buffer_size);

  void decodeParam(xml::XMLElement & element, char * & buffer_it);

  void decodeLeafNodeParamData(
    xml::XMLElement & element, char * & buffer_it,
    xml::XMLString & param_name);

  void decodeNodeEnd(xml::XMLElement & element, char * & buffer_it);

  void decodeSkipSpaces(
    char * & buffer_it, const xml::XMLElement & element, char * const buffer,
    const size_t & buffer_size);

  // node encoder functins

  void encodeNodes(
    xml::XMLElement & element, char * & buffer_it, int & size_left);

  void update_iterators(
    char * & buffer_it, int & buf_size_left, const xml::XMLElement & element,
    const int & buf_idx);
};
}  // namespace kuka_rsi_hw_interface


#endif  //  KUKA_RSI_HW_INTERFACE__RSI_COMMAND_HANDLER_HPP_
