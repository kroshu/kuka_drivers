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
#include <cstring>
#include <cstdlib>

#include "xml_element.hpp"

namespace xml
{
bool XMLElement::CastParam(XMLString key, char ** str_ptr)
{
  char key_c_str[key.length_ + 1] = {0};
  strncpy(key_c_str, key.data_ptr_, key.length_);

  auto param_it = params_.find(key_c_str);
  auto ret_val = param_it != params_.end();
  if (ret_val) {
    switch (param_it->second.param_type_) {
      case XMLType::BOOL:
      case XMLType::LONG:
        param_it->second.param_ = strtol(*str_ptr, str_ptr, 0);
        break;
      case XMLType::DOUBLE:
        param_it->second.param_ = strtod(*str_ptr, str_ptr);
        break;
      case XMLType::STRING:
        param_it->second.param_ = castXMLString(str_ptr);
        break;
      default:
        return false;
    }
  }
  return ret_val;
}

bool XMLElement::IsParamNameValid(XMLString & key, char ** str_ptr)
{
  key = castXMLString(str_ptr);
  char key_c_str[key.length_ + 1] = {0};
  strncpy(key_c_str, key.data_ptr_, key.length_);
  return params_.find(key_c_str) != params_.end();
}


bool XMLElement::IsNameValid(XMLString & key, char ** str_ptr)
{
  key = castXMLString(str_ptr);
  if (name_.length() != key.length_) {
    return false;
  }
  return strncmp(name_.c_str(), key.data_ptr_, key.length_);
}

XMLString XMLElement::castXMLString(char ** str_ptr)
{
  char * start_ptr = *str_ptr;
  for (;
    **str_ptr != '\0' && **str_ptr != '"' && **str_ptr != ' ' && **str_ptr != '=' &&
    **str_ptr != '>' && **str_ptr != '/'; *str_ptr++)
  {
  }
  return XMLString(start_ptr, (size_t)(*str_ptr - start_ptr));
}
}
