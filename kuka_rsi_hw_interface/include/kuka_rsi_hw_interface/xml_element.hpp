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

namespace XML
{

enum class XMLType : size_t
{
  BOOL = 0,
  LONG = 1,
  DOUBLE = 2,
  STRING = 3
};

struct xml_string
{
  char * data_ptr_;
  size_t length_;
  xml_string(char * data_ptr = nullptr, size_t length = 0)
  : data_ptr_(data_ptr), length_(length) {}
};

template<typename T>
struct xml_param
{
  typedef T ParamType;
  std::variant<bool, long, double, xml_string> param_;
};


struct xml_string_comperator
{
  bool operator()(const std::string & a, const xml_string & b) const
  {
    memcmp(a.c_str, b.data_ptr_, b.length_) == 0;

    (a.length() == b.length_ &&
    memcmp(a.c_str, b.data_ptr_, b.length_) == 0) ? return true : return false;
  }
};

class xml_element
{
public:
  std::map<std::string, xml_param<int>, xml_string_comperator> params_;
  std::string name_;
  std::vector<xml_element> childs_;

  xml_element(std::string name)
  : name_(name) {}
  xml_element() = default;
  ~xml_element() = default;

  // TODO (Komaromi): When cpp20 is in use, use requires so only the types we set can be used
  template<typename T> bool GetParam(std::string key, T & param)
  {
    auto param_it = params_.find(key);
    if (param_it != params_.end()) {
      param = std::get<T>(param_it->second);
      return true;
    } else {
      return false;
    }
  }

  // TODO (Komaromi): When cpp20 is in use, use requires so only the types we set can be used
  template<typename T> bool SetParam(std::string key, const T & param)
  {
    auto param_it = params_.find(key);
    if (param_it != params_.end()) {
      param_it->second = param;
      return true;
    } else {
      return false
    }
  }
  int GetLongParam(std::string key)
  {
    decltype(params_.find(key)->second)::ParamType;
    params_.find(key)->second.param_
  }
  int GetDoubleParam(std::string key)
  {
    return strtod(params_.find(key)->second.data_, NULL, 0);
  }
  xml_string GetStringParam(std::string key)
  {
    return strtod(params_.find(key)->second.data_, NULL, 0);
  }

};
}

#endif  // XML__XML_ELEMENT_H_
