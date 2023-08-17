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

#ifndef  XML__XML_STRING_H_
#define  XML__XML_STRING_H_

#include <string>

namespace xml
{
class XMLString
{
public:
  const char * data_ptr_;
  size_t length_;
  XMLString(const char * data_ptr, size_t length)
  : data_ptr_(data_ptr), length_(length) {}
  XMLString()
  : data_ptr_(nullptr), length_(0) {}
  XMLString(const char * data_ptr)
  : data_ptr_(data_ptr), length_(strlen(data_ptr)) {}
  XMLString(const std::string & str)
  : data_ptr_(str.c_str()), length_(str.length()) {}

  int PrintString(char * & buffer_it, const int & size_left);

  bool operator==(const XMLString & rhs);
  bool operator==(const std::string & rhs);
  bool operator==(const char * & rhs);
  friend std::ostream & operator<<(std::ostream & out, XMLString & xml_str);
};
}  // namespace xml

#endif  // XML__XML_STRING_H_
