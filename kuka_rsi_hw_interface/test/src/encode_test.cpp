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

#include "kuka_rsi_hw_interface/rsi_command_handler.hpp"

TEST_F(EncodeTest, EncodeSuccessful) {
  constexpr size_t UDP_BUFFER_SIZE = 1024;
  kuka_rsi_hw_interface::RSICommandHandler rsi_command_handler;
  char out_buffer_[UDP_BUFFER_SIZE] = {0};


  std::vector<double> cmd_pos {0, 1, 2, 3, 4, 5};
  bool stop_flag = false;
  int64_t ipoc = 100;

  // Initial command pos data
  rsi_command_handler.SetCommandParam<double>("AK", "A1", cmd_pos[0]);
  rsi_command_handler.SetCommandParam<double>("AK", "A2", cmd_pos[1]);
  rsi_command_handler.SetCommandParam<double>("AK", "A3", cmd_pos[2]);
  rsi_command_handler.SetCommandParam<double>("AK", "A4", cmd_pos[3]);
  rsi_command_handler.SetCommandParam<double>("AK", "A5", cmd_pos[4]);
  rsi_command_handler.SetCommandParam<double>("AK", "A6", cmd_pos[5]);
  rsi_command_handler.SetCommandParam<bool>("Stop", "Stop", stop_flag);
  rsi_command_handler.SetCommandParam<int64_t>("IPOC", "IPOC", static_cast<int64_t>(ipoc));

  auto out_buffer_it = out_buffer_;
  ASSERT_TRUE(rsi_command_handler.Encode(out_buffer_it, 1024));
}
