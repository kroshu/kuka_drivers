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

#include <map>
#include <vector>
#include <variant>
#include <string>
#include <cstring>
#include <cstdlib>
#include <iostream>
#include <variant>

#include "xml_handler/xml_element.hpp"

namespace xml
{
bool XMLElement::CastParam(const XMLString & key, char * & str_ptr)
{
  auto param_it = params_.find(key);
  auto ret_val = param_it != params_.end();
  if (ret_val) {
    errno = 0;
    switch (param_it->second.param_type_) {
      case XMLType::BOOL:
        {
          char * end = str_ptr;
          auto res = (bool)strtol(str_ptr, &end, 0);
          if (res == 0 && (errno != 0 || end == str_ptr)) {
            return false;
          }
          str_ptr = end;
          param_it->second.param_ = res;
          break;
        }
      case XMLType::LONG:
        {
          char * end = str_ptr;
          auto res = strtol(str_ptr, &end, 0);
          if (res == 0 && (errno != 0 || end == str_ptr)) {
            return false;
          }
          str_ptr = end;
          param_it->second.param_ = res;
          break;
        }
      case XMLType::DOUBLE:
        {
          char * end = str_ptr;
          auto res = strtod(str_ptr, &end);
          if (res == 0 && (errno != 0 || end == str_ptr)) {
            return false;
          }
          str_ptr = end;
          param_it->second.param_ = res;
          break;
        }
      case XMLType::STRING:
        param_it->second.param_ = castXMLString(str_ptr);
        break;
      default:
        return false;
    }
  }
  return ret_val;
}

bool XMLElement::IsParamNameValid(XMLString & key, char * & str_ptr)
{
  key = castXMLString(str_ptr);
  std::cout << "detected param: " << key << std::endl;
  auto param_it = params_.find(key);
  if (param_it != params_.end()) {
    std::cout << "stored param:   " << param_it->first << std::endl;
    return true;
  }
  return false;
  // return params_.find(key) != params_.end();
}


bool XMLElement::IsNameValid(XMLString & key, char * & str_ptr)
{
  key = castXMLString(str_ptr);
  std::cout << "detected name: " << key << std::endl;
  std::cout << "stored name:   " << name_ << std::endl;
  if (name_.length() != key.length_) {
    return false;
  }
  return strncmp(name_.c_str(), key.data_ptr_, key.length_) == 0;
}

XMLElement & XMLElement::Element(const std::string & elementName, int depth)
{
  if (elementName == name_) {
    return *this;
  }
  for (auto && child : childs_) {
    child.Element(elementName, depth + 1);
  }
  if (depth == 0) {
    char err_buf[1000];
    sprintf(err_buf, "%s element not found", elementName.c_str());
    throw std::logic_error(err_buf);
  }
}

const XMLElement & XMLElement::GetElement(const std::string & elementName, int depth) const
{
  if (elementName == name_) {
    return *this;
  }
  for (auto && child : childs_) {
    child.GetElement(elementName);
  }
  if (depth == 0) {
    char err_buf[1000];
    sprintf(err_buf, "%s element not found", elementName.c_str());
    throw std::logic_error(err_buf);
  }
}


std::ostream & XMLElement::operator<<(std::ostream & out) const
{
  out << "Element name: " << this->name_ << "\n";
  for (auto && param : params_) {
    out << "  - " << param.first << ": ";
    switch (param.second.param_type_) {
      case XMLType::BOOL:
        out << std::get<0>(param.second.param_) << "\n";
        break;
      case XMLType::LONG:
        out << std::get<1>(param.second.param_) << "\n";
        break;
      case XMLType::DOUBLE:
        out << std::get<2>(param.second.param_) << "\n";
        break;
      case XMLType::STRING:
        {
          char str[std::get<3>(param.second.param_).length_ + 1];
          std::snprintf(
            str, std::get<3>(param.second.param_).length_ + 1, "%s",
            std::get<3>(param.second.param_).data_ptr_);
          out << str << "\n";
        }
        break;

      default:
        break;
    }
  }
  for (auto && child : childs_) {
    child.operator<<(out) << "\n";
  }
  return out;
}

XMLString XMLElement::castXMLString(char * & str_ptr)
{
  auto start_ref = str_ptr;
  for (;
    *str_ptr != '\0' && *str_ptr != '"' && *str_ptr != ' ' &&
    *str_ptr != '=' && *str_ptr != '>' && *str_ptr != '/' && *str_ptr != '<'; str_ptr++)
  {
  }
  auto data = XMLString(
    start_ref, (size_t)(str_ptr - start_ref));
  return data;
}
}
