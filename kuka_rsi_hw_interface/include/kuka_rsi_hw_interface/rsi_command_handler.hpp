/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2014 Norwegian University of Science and Technology
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Norwegian University of Science and
*     Technology, nor the names of its contributors may be used to
*     endorse or promote products derived from this software without
*     specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/*
 * Author: Komaromi Sandor
 */

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

private:
  xml::XMLElement command_data_structure_;
  xml::XMLElement state_data_structure_;

  void decodeNodes(
    xml::XMLElement & element, char * const buffer, char * & buffer_it,
    const size_t buffer_size);

  void encodeNodes(
    xml::XMLElement & element, char * & buffer_it, int & size_left);
};
}  // namespace kuka_rsi_hw_interface


#endif  //  KUKA_RSI_HW_INTERFACE__RSI_COMMAND_HANDLER_HPP_
