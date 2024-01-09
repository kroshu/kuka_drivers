// Copyright 2022 Lars Tingelstad
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

#ifndef KUKA_KSS_RSI_DRIVER__RSI_STATE_HPP_
#define KUKA_KSS_RSI_DRIVER__RSI_STATE_HPP_

#include <tinyxml.h>
#include <string>
#include <vector>

namespace kuka_kss_rsi_driver
{
class RSIState
{
private:
  std::string xml_doc_;

public:
  RSIState() { xml_doc_.resize(1024); }

  explicit RSIState(std::string xml_doc) : xml_doc_(xml_doc)
  {
    // Parse message from robot
    TiXmlDocument bufferdoc;
    bufferdoc.Parse(xml_doc_.c_str());
    // Get the Rob node:
    TiXmlElement * rob = bufferdoc.FirstChildElement("Rob");
    // Extract axis specific actual position
    TiXmlElement * AIPos_el = rob->FirstChildElement("AIPos");
    AIPos_el->Attribute("A1", &positions[0]);
    AIPos_el->Attribute("A2", &positions[1]);
    AIPos_el->Attribute("A3", &positions[2]);
    AIPos_el->Attribute("A4", &positions[3]);
    AIPos_el->Attribute("A5", &positions[4]);
    AIPos_el->Attribute("A6", &positions[5]);
    // Extract axis specific setpoint position
    TiXmlElement * ASPos_el = rob->FirstChildElement("ASPos");
    ASPos_el->Attribute("A1", &initial_positions[0]);
    ASPos_el->Attribute("A2", &initial_positions[1]);
    ASPos_el->Attribute("A3", &initial_positions[2]);
    ASPos_el->Attribute("A4", &initial_positions[3]);
    ASPos_el->Attribute("A5", &initial_positions[4]);
    ASPos_el->Attribute("A6", &initial_positions[5]);
    // Extract cartesian actual position
    TiXmlElement * RIst_el = rob->FirstChildElement("RIst");
    RIst_el->Attribute("X", &cart_position[0]);
    RIst_el->Attribute("Y", &cart_position[1]);
    RIst_el->Attribute("Z", &cart_position[2]);
    RIst_el->Attribute("A", &cart_position[3]);
    RIst_el->Attribute("B", &cart_position[4]);
    RIst_el->Attribute("C", &cart_position[5]);
    // Extract cartesian actual position
    TiXmlElement * RSol_el = rob->FirstChildElement("RSol");
    RSol_el->Attribute("X", &initial_cart_position[0]);
    RSol_el->Attribute("Y", &initial_cart_position[1]);
    RSol_el->Attribute("Z", &initial_cart_position[2]);
    RSol_el->Attribute("A", &initial_cart_position[3]);
    RSol_el->Attribute("B", &initial_cart_position[4]);
    RSol_el->Attribute("C", &initial_cart_position[5]);
    // Get the IPOC timestamp
    TiXmlElement * ipoc_el = rob->FirstChildElement("IPOC");
    ipoc = std::stoull(ipoc_el->FirstChild()->Value());
  }

  std::vector<double> positions = std::vector<double>(6, 0.0);
  std::vector<double> initial_positions = std::vector<double>(6, 0.0);
  std::vector<double> cart_position = std::vector<double>(6, 0.0);
  std::vector<double> initial_cart_position = std::vector<double>(6, 0.0);
  uint64_t ipoc = 0;
};
}  // namespace kuka_kss_rsi_driver

#endif  // KUKA_KSS_RSI_DRIVER__RSI_STATE_HPP_
