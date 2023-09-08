// Copyright 2023 Áron Svastits
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

#include "encode_test.h"  // NOLINT

#include <iostream>

#include "kuka_rsi_hw_interface/rsi_command_handler.hpp"
#include "kuka_rsi_hw_interface/pugi_command_handler.hpp"

TEST_F(EncodeTest, EncodeSuccessful) {
  constexpr size_t UDP_BUFFER_SIZE = 1024;
  kuka_rsi_hw_interface::RSICommandHandler rsi_command_handler;
  char out_buffer_[UDP_BUFFER_SIZE] = {0};
  bool ret_val = true;


  std::vector<double> cmd_pos {0, 1, 2, 3, 4, 5};
  bool stop_flag = false;
  int64_t ipoc = 100;

  // Initial command pos data
  ret_val &= rsi_command_handler.SetCommandParam<double>("AK", "A1", cmd_pos[0]);
  ret_val &= rsi_command_handler.SetCommandParam<double>("AK", "A2", cmd_pos[1]);
  ret_val &= rsi_command_handler.SetCommandParam<double>("AK", "A3", cmd_pos[2]);
  ret_val &= rsi_command_handler.SetCommandParam<double>("AK", "A4", cmd_pos[3]);
  ret_val &= rsi_command_handler.SetCommandParam<double>("AK", "A5", cmd_pos[4]);
  ret_val &= rsi_command_handler.SetCommandParam<double>("AK", "A6", cmd_pos[5]);
  ret_val &= rsi_command_handler.SetCommandParam<bool>("Stop", "Stop", stop_flag);
  ret_val &=
    rsi_command_handler.SetCommandParam<int64_t>("IPOC", "IPOC", static_cast<int64_t>(ipoc));

  auto out_buffer_it = out_buffer_;
  ASSERT_TRUE(ret_val);
  ASSERT_TRUE(rsi_command_handler.Encode(out_buffer_it, 1024));
}
TEST_F(EncodeTest, EncodedWell) {
  constexpr size_t UDP_BUFFER_SIZE = 1024;
  kuka_rsi_hw_interface::RSICommandHandler rsi_command_handler;
  char out_buffer_[UDP_BUFFER_SIZE] = {0};
  bool ret_val = true;


  std::vector<double> cmd_pos {0, 1, 2, 3, 4, 5};
  bool stop_flag = false;
  int64_t ipoc = 100;

  // Initial command pos data
  ret_val &= rsi_command_handler.SetCommandParam<double>("AK", "A1", cmd_pos[0]);
  ret_val &= rsi_command_handler.SetCommandParam<double>("AK", "A2", cmd_pos[1]);
  ret_val &= rsi_command_handler.SetCommandParam<double>("AK", "A3", cmd_pos[2]);
  ret_val &= rsi_command_handler.SetCommandParam<double>("AK", "A4", cmd_pos[3]);
  ret_val &= rsi_command_handler.SetCommandParam<double>("AK", "A5", cmd_pos[4]);
  ret_val &= rsi_command_handler.SetCommandParam<double>("AK", "A6", cmd_pos[5]);
  ret_val &= rsi_command_handler.SetCommandParam<bool>("Stop", "Stop", stop_flag);
  ret_val &=
    rsi_command_handler.SetCommandParam<int64_t>("IPOC", "IPOC", static_cast<int64_t>(ipoc));

  auto out_buffer_it = out_buffer_;
  ASSERT_TRUE(ret_val);
  ASSERT_TRUE(rsi_command_handler.Encode(out_buffer_it, 1024));
  ASSERT_TRUE(
    strcmp(
      out_buffer_,
      "<Sen Type=\"KROSHU\"><AK A1=\"0.000000\" A2=\"1.000000\" A3=\"2.000000\" A4=\"3.000000\" A5=\"4.000000\" A6=\"5.000000\" /><Stop>0</Stop><IPOC>100</IPOC></Sen>") == 0
  );
}

TEST_F(PugiEncodeTest, EncodeSuccessful) {
  kuka_rsi_hw_interface::PugiCommandHandler pugi_command_handler(1024);
  char out_buffer_[1024] = {0};
  bool ret_val = true;

  std::vector<double> cmd_pos {0, 1, 2, 3, 4, 5};
  bool stop_flag = false;
  int64_t ipoc = 100;

  // Add structure
  ret_val &=
    pugi_command_handler.command_doc.append_child("Sen").append_attribute("Type").set_value(
    "KROSHU");
  pugi::xml_node Sen = pugi_command_handler.command_doc.child("Sen");
  Sen.append_child("AK");
  pugi::xml_node AK = Sen.child("AK");
  AK.append_attribute("A1");
  AK.append_attribute("A2");
  AK.append_attribute("A3");
  AK.append_attribute("A4");
  AK.append_attribute("A5");
  AK.append_attribute("A6");
  Sen.append_child("Stop");
  Sen.append_child("IPOC");

  // Fill Data
  // This Should be real time
  auto cmd_pos_it = cmd_pos.begin();
  for (auto it = AK.attributes_begin(); it != AK.attributes_end() && cmd_pos_it != cmd_pos.end();
    ++it)
  {
    ret_val &= it->set_value(&cmd_pos_it);
    ++cmd_pos_it;
  }
  ret_val &= Sen.child("Stop").text().set(stop_flag);
  ret_val &= Sen.child("IPOC").text().set(ipoc);

  ASSERT_TRUE(ret_val);
  ASSERT_TRUE((pugi_command_handler.Encode(out_buffer_, 1024) != nullptr));
}
TEST_F(PugiEncodeTest, EncodedWell) {
  kuka_rsi_hw_interface::PugiCommandHandler pugi_command_handler(1024);
  char out_buffer_[1024] = {0};
  bool ret_val = true;

  std::vector<double> cmd_pos {0, 1, 2, 3, 4, 5};
  bool stop_flag = false;
  int64_t ipoc = 100;

  // Add structure
  ret_val &=
    pugi_command_handler.command_doc.append_child("Sen").append_attribute("Type").set_value(
    "KROSHU");
  pugi::xml_node Sen = pugi_command_handler.command_doc.child("Sen");
  Sen.append_child("AK");
  pugi::xml_node AK = Sen.child("AK");
  AK.append_attribute("A1");
  AK.append_attribute("A2");
  AK.append_attribute("A3");
  AK.append_attribute("A4");
  AK.append_attribute("A5");
  AK.append_attribute("A6");
  Sen.append_child("Stop");
  Sen.append_child("IPOC");

  // Fill Data
  // This Should be real time
  size_t i = 0;
  for (auto it = AK.attributes_begin(); it != AK.attributes_end(); ++it) {
    ret_val &= it->set_value(cmd_pos[i]);
    ++i;
  }
  ret_val &= Sen.child("Stop").text().set(static_cast<uint8_t>(stop_flag));
  ret_val &= Sen.child("IPOC").text().set(ipoc);

  ASSERT_TRUE(ret_val);
  ASSERT_TRUE((pugi_command_handler.Encode(out_buffer_, 1024) != nullptr));
  ASSERT_TRUE(
    strcmp(
      out_buffer_,
      "<Sen Type=\"KROSHU\"><AK A1=\"0\" A2=\"1\" A3=\"2\" A4=\"3\" A5=\"4\" A6=\"5\"/><Stop>0</Stop><IPOC>100</IPOC></Sen>") == 0
  );
}
