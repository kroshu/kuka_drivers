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
#include <string>
#include <cstring>
#include <cstdlib>
#include <iostream>
#include <functional>


namespace xml
{

enum class XMLType : size_t
{
  BOOL = 0,
  LONG = 1,
  DOUBLE = 2,
  STRING = 3
};

class XMLString
{
public:
  const char * data_ptr_;
  size_t length_;
  XMLString(const char * data_ptr = nullptr, size_t length = 0)
  : data_ptr_(data_ptr), length_(length) {}
  XMLString(const std::string & str)
  : data_ptr_(str.c_str()), length_(str.length()) {}
  bool operator==(const XMLString & rhs);
  bool operator==(const std::string & rhs);
  bool operator==(const char * & rhs);
};

struct XMLParam
{
  XMLType param_type_;
  std::variant<bool, long, double, XMLString> param_;
  XMLParam() = default;
  XMLParam(XMLType type)
  : param_type_(type) {}
};


inline bool operator<(const XMLString & a, const std::string & b)
{
  return a.length_ == b.length() && strncmp(a.data_ptr_, b.c_str(), a.length_) < 0;
}

inline bool operator<(const std::string & b, const XMLString & a)
{
  return a.length_ == b.length() && strncmp(b.c_str(), a.data_ptr_, a.length_) < 0;
}

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
  static XMLString castXMLString(char * & str_ptr);

public:
  std::string name_;
  std::map<std::string, XMLParam, std::less<>> params_;
  std::vector<XMLElement> childs_;

  XMLElement(const std::string & name)
  : name_(name) {}
  XMLElement() = default;
  ~XMLElement() = default;

  bool CastParam(const XMLString & key, char * & str_ptr);
  bool IsParamNameValid(XMLString & key, char * & str_ptr);
  bool IsNameValid(XMLString & key, char * & str_ptr);

  XMLElement & GetElement(const std::string & elementName);
  const XMLElement & GetElement(const std::string & elementName) const;

  std::ostream & operator<<(std::ostream & out) const;

  // TODO (Komaromi): When cpp20 is in use, use requires so only the types we set can be used
  template<typename T>
  bool GetParam(const std::string & key, T & param) const
  {
    auto param_it = params_.find(key);
    if (param_it != params_.end()) {
      if constexpr (std::is_same<T, bool>::value || std::is_same<T, long>::value ||
        std::is_same<T, double>::value || std::is_same<T, XMLString>::value)
      {
        param = std::get<T>(param_it->second.param_);
        return true;
      }
    }
    return false;
  }

  // TODO (Komaromi): When cpp20 is in use, use requires so only the types we set can be used
  template<typename T>
  bool SetParam(const std::string & key, const T & param)
  {
    auto param_it = params_.find(key);
    if (param_it != params_.end()) {
      if constexpr (std::is_same<T, bool>::value || std::is_same<T, long>::value ||
        std::is_same<T, double>::value || std::is_same<T, XMLString>::value)
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
