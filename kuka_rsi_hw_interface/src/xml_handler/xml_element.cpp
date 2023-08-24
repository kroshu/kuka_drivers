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
#include <clocale>
#include <sstream>

#include "xml_handler/xml_element.hpp"

namespace xml
{
bool XMLElement::CastParamData(const XMLString & key, char * & str_ptr)
{
  auto param_it = params_.find(key);
  auto ret_val = param_it != params_.end();
  if (ret_val) {
    errno = 0;
    char * prev_locale_ptr = prev_locale_;
    prev_locale_ptr = std::setlocale(LC_NUMERIC, nullptr);
    if (!std::setlocale(LC_NUMERIC, "C")) {
      throw std::logic_error("Locale set failed");
    }
    switch (param_it->second.param_type_) {
      case XMLType::BOOL:
        {
          char * end = str_ptr;
          auto res = static_cast<bool>(strtol(str_ptr, &end, 0));
          if (res == 0 && (errno != 0 || end == str_ptr)) {
            if (!std::setlocale(LC_NUMERIC, prev_locale_ptr)) {
              throw std::logic_error("Locale set failed");
            }
            return false;
          }
          str_ptr = end;
          param_it->second.param_value_ = res;
          break;
        }
      case XMLType::LONG:
        {
          char * end = str_ptr;
          auto res = strtol(str_ptr, &end, 0);
          if (res == 0 && (errno != 0 || end == str_ptr)) {
            if (!std::setlocale(LC_NUMERIC, prev_locale_ptr)) {
              throw std::logic_error("Locale set failed");
            }
            return false;
          }
          str_ptr = end;
          param_it->second.param_value_ = res;
          break;
        }
      case XMLType::DOUBLE:
        {
          char * end = str_ptr;
          auto res = strtod(str_ptr, &end);
          if (res == 0 && (errno != 0 || end == str_ptr)) {
            if (!std::setlocale(LC_NUMERIC, prev_locale_ptr)) {
              throw std::logic_error("Locale set failed");
            }
            return false;
          }
          str_ptr = end;
          param_it->second.param_value_ = res;
          break;
        }
      case XMLType::STRING:
        param_it->second.param_value_ = castXMLString(str_ptr);
        break;
      default:
        std::setlocale(LC_NUMERIC, prev_locale_ptr);
        return false;
    }
    if (!std::setlocale(LC_NUMERIC, prev_locale_ptr)) {
      throw std::logic_error("Locale set failed");
    }
  }
  return ret_val;
}

bool XMLElement::IsParamNameValid(XMLString & key, char * & str_ptr)
{
  key = castXMLString(str_ptr);
  auto param_it = params_.find(key);
  if (param_it != params_.end()) {
    return true;
  }
  return false;
}


bool XMLElement::IsNameValid(XMLString & key, char * & str_ptr)
{
  key = castXMLString(str_ptr);
  if (name_.length() != key.length_) {
    return false;
  }
  return strncmp(name_.c_str(), key.data_ptr_, key.length_) == 0;
}

XMLElement * XMLElement::element(const std::string & elementName, int depth)
{
  if (elementName == name_) {
    return this;
  }
  for (auto && child : childs_) {
    auto ret_val = child.element(elementName, depth + 1);
    if (ret_val != nullptr) {
      return ret_val;
    }
  }
  if (depth == 0) {
    std::stringstream err_ss;
    err_ss << elementName << " element not found.";
    throw std::logic_error(err_ss.str());
  }
  return nullptr;
}

const XMLElement * XMLElement::getElement(
  const std::string & elementName,
  int depth) const
{
  if (elementName == name_) {
    return this;
  }
  for (auto && child : childs_) {
    auto ret_val = child.getElement(elementName, depth + 1);
    if (ret_val != nullptr) {
      return ret_val;
    }
  }
  if (depth == 0) {
    std::stringstream err_ss;
    err_ss << elementName << " element not found.";
    throw std::logic_error(err_ss.str());
  }
  return nullptr;
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
    start_ref, static_cast<size_t>(str_ptr - start_ref));
  return data;
}
}  // namespace xml
