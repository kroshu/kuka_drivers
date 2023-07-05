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

#ifndef  XML__XML_ELEMENT_H_
#define  XML__XML_ELEMENT_H_

#include <map>
#include <string>
#include <vector>
#include <variant>
#include <cstring>

namespace xml
{

enum class XMLType : size_t
{
  BOOL = 0,
  LONG = 1,
  DOUBLE = 2,
  STRING = 3
};

struct XMLString
{
  char * data_ptr_;
  size_t length_;
  XMLString(char * data_ptr = nullptr, size_t length = 0)
  : data_ptr_(data_ptr), length_(length) {}
};

struct XMLParam
{
  XMLType param_type_;
  std::variant<bool, long, double, XMLString> param_;
  XMLParam(XMLType type)
  : param_type_(type) {}
};

// struct XMLStringComparator
// {
//   bool operator()(const char * a, const char * b) const
//   {
//     return a.length() == b.length_ && memcmp(a.data(), b.data_ptr_, b.length_) > 0;
//   }
// };

class XMLElement
{
public:
  std::map<std::string, XMLParam> params_;
  std::string name_;
  std::vector<XMLElement> childs_;

  XMLElement(std::string name)
  : name_(name) {}
  XMLElement() = default;
  ~XMLElement() = default;

  // TODO (Komaromi): When cpp20 is in use, use requires so only the types we set can be used
  template<typename T>
  bool GetParam(std::string key, T & param)
  {
    auto param_it = params_.find(key);
    if (param_it != params_.end()) {
      if (std::is_same<T, bool>() || std::is_same<T, long>() ||
        std::is_same<T, double>() || std::is_same<T, XMLString>)
      {
        param = std::get<T>(param_it->second.param_);
        return true;
      }
    }
    return false;
  }

  // TODO (Komaromi): When cpp20 is in use, use requires so only the types we set can be used
  template<typename T>
  bool SetParam(std::string key, const T & param)
  {
    auto param_it = params_.find(key);
    if (param_it != params_.end()) {
      if (std::is_same<T, bool>() || std::is_same<T, long>() ||
        std::is_same<T, double>() || std::is_same<T, XMLString>)
      {
        param_it->second.param_ = param;
        return true;
      }
    }
    return false;
  }

  void CastParam(XMLString key, const char * str_start_ptr, char ** end_ptr)
  {
    char key_c_str[key.length_];
    strncpy(key_c_str, key.data_ptr_, key.length_);

    auto param_it = params_.find(key_c_str);
    if (param_it != params_.end()) {
      switch (param_it->second.param_type_) {
        case XMLType::BOOL:
        case XMLType::LONG:
          param_it->second.param_ = strtol(str_start_ptr, end_ptr, 0);
          break;
        case XMLType::DOUBLE:
          param_it->second.param_ = strtod(str_start_ptr, end_ptr);
          break;
        case XMLType::STRING:
          // TODO (Komaromi): write a cast func for casting XMLString
          //param_it->second.param_ = XMLString...
          break;
        default:
          break;
      }
    }

  }

};
}

#endif  // XML__XML_ELEMENT_H_
