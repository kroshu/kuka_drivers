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

#ifndef  XML__XML_ELEMENT_H_
#define  XML__XML_ELEMENT_H_

#include <map>
#include <vector>
#include <variant>

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
private:
  static XMLString castXMLString(char ** str_ptr);

public:
  std::string name_;
  std::map<std::string, XMLParam> params_;
  std::vector<XMLElement> childs_;

  XMLElement(std::string name)
  : name_(name) {}
  XMLElement() = default;
  ~XMLElement() = default;

  bool CastParam(XMLString key, char ** str_ptr);

  bool IsParamNameValid(XMLString & key, char ** str_ptr);
  bool IsNameValid(XMLString & key, char ** str_ptr);

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
};
}

#endif  // XML__XML_ELEMENT_H_
