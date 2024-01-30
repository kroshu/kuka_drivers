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

#ifndef KUKA_KSS_RSI_DRIVER__RSI_COMMAND_HPP_
#define KUKA_KSS_RSI_DRIVER__RSI_COMMAND_HPP_

#include <tinyxml.h>
#include <string>
#include <vector>

namespace kuka_kss_rsi_driver
{
class RSICommand
{
public:
  RSICommand() = default;
  RSICommand(std::vector<double> joint_position_correction, uint64_t ipoc, bool stop = false)
  {
    TiXmlDocument doc;
    TiXmlElement * root = new TiXmlElement("Sen");
    root->SetAttribute("Type", "KROSHU");
    TiXmlElement * el = new TiXmlElement("AK");
    // Add string attribute
    el->SetAttribute("A1", std::to_string(joint_position_correction[0]));
    el->SetAttribute("A2", std::to_string(joint_position_correction[1]));
    el->SetAttribute("A3", std::to_string(joint_position_correction[2]));
    el->SetAttribute("A4", std::to_string(joint_position_correction[3]));
    el->SetAttribute("A5", std::to_string(joint_position_correction[4]));
    el->SetAttribute("A6", std::to_string(joint_position_correction[5]));
    root->LinkEndChild(el);

    el = new TiXmlElement("Stop");
    el->LinkEndChild(new TiXmlText(std::to_string(static_cast<int>(stop))));
    root->LinkEndChild(el);

    el = new TiXmlElement("IPOC");
    el->LinkEndChild(new TiXmlText(std::to_string(ipoc)));
    root->LinkEndChild(el);
    doc.LinkEndChild(root);
    TiXmlPrinter printer;
    printer.SetStreamPrinting();
    doc.Accept(&printer);

    xml_doc = printer.Str();
  }
  std::string xml_doc;
};
}  // namespace kuka_kss_rsi_driver

#endif  // KUKA_KSS_RSI_DRIVER__RSI_COMMAND_HPP_
