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

#ifndef  XML_HANDLER__XML_ELEMENT_HPP_
#define  XML_HANDLER__XML_ELEMENT_HPP_

#include <map>
#include <vector>
#include <string>
#include <cstring>
#include <cstdlib>
#include <iostream>
#include <functional>

#include "xml_param.hpp"

namespace xml
{
inline bool operator<(const XMLString & a, const std::string & b)
{
  if (a.length_ < b.length()) {
    return true;
  } else if (a.length_ > b.length()) {
    return false;
  } else {
    return strncmp(a.data_ptr_, b.c_str(), a.length_) < 0;
  }
}

inline bool operator<(const std::string & b, const XMLString & a)
{
  if (b.length() < a.length_) {
    return true;
  } else if (b.length() > a.length_) {
    return false;
  } else {
    return strncmp(b.c_str(), a.data_ptr_, a.length_) < 0;
  }
}

/**
 * @brief The XMLElement class is a tree based representation of a XML structure.
 * It has name, parameters and childs. It's name is a unique key, which identifies the XMLElement object.
 * The parameters contain the data and attributes together.
 * The childs contains the XMLElement's every child (of type XMLElement).
 */
class XMLElement
{
public:
  /**
   * @brief Construct a new XMLElement object
   *
   * @param name: Name of the XMLElement object
   */
  explicit XMLElement(const std::string & name)
  : name_(name) {}
  ~XMLElement() = default;

  /**
   * @brief Get the name of the XMLElement object
   *
   * @return const std::string & - The name of the XMLElement object
   */
  inline const std::string & GetName() const {return name_;}

  /**
   * @brief Get the Childs of the XMLElement object
   *
   * @return const std::vector<XMLElement> & - A constant vector of the XMLElements childs
   */
  inline const std::vector<XMLElement> & GetChilds() const {return childs_;}

  /**
   * @brief Returns the XMLElement object with the given name.
   *
   * @param elementName Name of the XMLElement object that the functon is looking for
   * in the XMLElement tree
   * @return const XMLElement * - The found XMLElment object.
   * Return nullptr if the given key was not found.
   */
  inline const XMLElement * GetElement(
    const std::string & elementName) const {return getElement(elementName);}

  /**
   * @brief Returns the value of the parameter of the XMLElement object with the given key.
   *
   * @tparam T Can only be bool, int64_t, double and XMLString.
   * @param key The parameter's name that the function is looking for.
   * @return T - The parameter value if it is found
   * @exception Range_error if key not found in the parameter list.
   */
  template<typename T>
  T GetParam(const std::string & key) const
  {
    auto param_it = params_.find(key);
    if (param_it != params_.end()) {
      return param_it->second.GetParamValue<T>();
    }
    throw std::range_error("Could not find key in parameter list");
  }

  /**
   * @brief Set the Name of the XMLElement object
   *
   * @param name: Name of the XMLElement object
   */
  inline void SetName(const std::string & name) {name_ = name;}

  /**
 * @brief Set the value of the parameter with the given name
 *
 * @tparam T Can only be bool, int64_t, double and XMLString.
 * @param key The name of the parameter to be changed.
 * @param param The data witch will be inserted.
 * @return true if the key is found in the parameter list and the parameter has proper type,
 * @return false otherwise
 */
  template<typename T>
  bool SetParam(const std::string & key, const T & param)
  {
    auto param_it = params_.find(key);
    if (param_it != params_.end()) {
      if constexpr (std::is_same<T, bool>::value || std::is_same<T, int64_t>::value ||
        std::is_same<T, double>::value || std::is_same<T, XMLString>::value)
      {
        param_it->second.param_value_ = param;
        return true;
      }
    }
    return false;
  }

  /**
   * @brief Return every child in a std::vector that the XMLElement object has
   *
   * @return A reference to a vector of the XMLElement object's childs
   */
  inline std::vector<XMLElement> & Childs() {return childs_;}

  /**
   * @brief Returns every parameter in a std::map that the XMLElement object has
   *
   * @return A reference to the XMLElement object's parameter map
   */
  inline std::map<std::string, XMLParam, std::less<>> & Params() {return params_;}

  /**
   * @brief Returns the XMLElement object with the given name
   *
   * @param elementName Name of the XMLElement object that the functon is looking for
   * in the XMLElement tree
   * @return XMLElement* - The found XMLElment object.
   * Return nullptr if the given key was not found.
   */
  inline XMLElement * Element(const std::string & elementName) {return element(elementName);}

  /**
   * @brief Converts the given string to the specified parameter of the element
   * and updates the parameter map
   *
   * @param key The name of the parameter that the function casts into.
   * @param str_ptr Pointer reference to the string the function casts from.
   * When the function returns, the pointer points to the character after the cast data.
   * @return true if the cast was succesful and the parameter was found in the parameter list,
   * @return false otherwise
   */
  bool CastParamData(const XMLString & key, char * & str_ptr);

  /**
   * @brief Checks if the given string matches one of parameter names of the XMLElement
   *
   * @param [out] key name of the parameter as an XMLString.
   * @param str_ptr Pointer reference to the string the function checks.
   * When the function returns, the pointer points to the character after the checked data.
   * @return true if the name check was successfull,
   * @return false otherwise.
   */
  bool IsParamNameValid(XMLString & key, char * & str_ptr);

  /**
   * @brief Checks if the given string matches the XMLElement's name
   *
   * @param [out] key name of the XMLElement object as an XMLString
   * @param str_ptr Pointer reference to the string the function checks.
   * When the function returns, the pointer points to the character after the checked data.
   * @return true if the name check was successfull,
   * @return false otherwise.
   */
  bool IsNameValid(XMLString & key, char * & str_ptr);

private:
  static XMLString castXMLString(char * & str_ptr);
  XMLElement * element(const std::string & elementName, int depth = 0);
  const XMLElement * getElement(
    const std::string & elementName,
    int depth = 0) const;

  std::map<std::string, XMLParam, std::less<>> params_;
  std::vector<XMLElement> childs_;
  std::string name_;
};
}  // namespace xml

#endif  // XML_HANDLER__XML_ELEMENT_HPP_
