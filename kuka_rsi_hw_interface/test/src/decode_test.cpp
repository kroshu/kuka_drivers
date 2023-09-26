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

#include "decode_test.h"  // NOLINT

#include <chrono>
#include <fstream>

#include "kuka_rsi_hw_interface/rsi_command_handler.hpp"
#include "kuka_rsi_hw_interface/pugi_command_handler.hpp"

TEST_F(DecodeTest, DecodeSuccessful) {
  kuka_rsi_hw_interface::RSICommandHandler rsi_command_handler;

  char input_string[] =
    "<Rob TYPE=\"KUKA\"><RIst X=\"0.0\" Y=\"0.0\" Z=\"0.0\" A=\"0.0\" B=\"0.0\" C=\"0.0\"" \
    " /><RSol X=\"0.0\" Y=\"0.0\" Z=\"0.0\" A=\"0.0\" B=\"0.0\" C=\"0.0\" /><AIPos A1=\"0.0\"" \
    " A2=\"-90.0\" A3=\"90.0\" A4=\"0.0\" A5=\"90.0\" A6=\"0.0\" /><ASPos A1=\"0.0\" A2=\"-90.0\"" \
    " A3=\"90.0\" A4=\"0.0\" A5=\"90.0\" A6=\"0.0\" /><Delay D=\"0\" /><IPOC>0</IPOC></Rob>";
  ASSERT_TRUE(rsi_command_handler.Decode(input_string, 1024));
}
TEST_F(DecodeTest, DecodeParamValid) {
  kuka_rsi_hw_interface::RSICommandHandler rsi_command_handler;

  char input_string[] =
    "<Rob TYPE=\"KUKA\"><RIst X=\"0.0\" Y=\"0.0\" Z=\"0.0\" A=\"0.0\" B=\"0.0\" C=\"0.0\"" \
    " /><RSol X=\"0.0\" Y=\"0.0\" Z=\"0.0\" A=\"0.0\" B=\"0.0\" C=\"0.0\" /><AIPos A1=\"0.0\"" \
    " A2=\"-90.0\" A3=\"90.0\" A4=\"0.0\" A5=\"90.0\" A6=\"0.0\" /><ASPos A1=\"0.0\" A2=\"-90.0\"" \
    " A3=\"90.0\" A4=\"0.0\" A5=\"90.0\" A6=\"0.0\" /><Delay D=\"0\" /><IPOC>0</IPOC></Rob>";
  ASSERT_TRUE(rsi_command_handler.Decode(input_string, 1024));
  ASSERT_TRUE(rsi_command_handler.GetState().GetElement("AIPos")->GetParam<double>("A5") == 90.0);
}
TEST_F(DecodeTest, DecodeTimeMeasurement) {
  kuka_rsi_hw_interface::RSICommandHandler rsi_command_handler;

  char input_string[] =
    "<Rob TYPE=\"KUKA\"><RIst X=\"0.0\" Y=\"0.0\" Z=\"0.0\" A=\"0.0\" B=\"0.0\" C=\"0.0\"" \
    " /><RSol X=\"0.0\" Y=\"0.0\" Z=\"0.0\" A=\"0.0\" B=\"0.0\" C=\"0.0\" /><AIPos A1=\"0.0\"" \
    " A2=\"-90.0\" A3=\"90.0\" A4=\"0.0\" A5=\"90.0\" A6=\"0.0\" /><ASPos A1=\"0.0\" A2=\"-90.0\"" \
    " A3=\"90.0\" A4=\"0.0\" A5=\"90.0\" A6=\"0.0\" /><Delay D=\"0\" /><IPOC>0</IPOC></Rob>";
  bool ret_val = true;
  std::ofstream timer_data;

  timer_data.open("DecodeTest_DecodeTimeMeasurement.txt");

  struct sched_param param;
  param.sched_priority = 98;
  ret_val &= (sched_setscheduler(0, SCHED_FIFO, &param) != -1);

  for (size_t i = 0; i < 4000; i++) {
    auto start = std::chrono::high_resolution_clock::now();
    ret_val &= rsi_command_handler.Decode(input_string, 1024);
    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(stop - start);
    timer_data << i << " " << duration.count() << std::endl;
  }
  ASSERT_TRUE(ret_val);
}


TEST_F(PugiDecodeTest, DecodeSuccessful) {
  kuka_rsi_hw_interface::PugiCommandHandler pugi_command_handler(1024);

  char input_string[] =
    "<Rob TYPE=\"KUKA\"><RIst X=\"0.0\" Y=\"0.0\" Z=\"0.0\" A=\"0.0\" B=\"0.0\" C=\"0.0\"" \
    " /><RSol X=\"0.0\" Y=\"0.0\" Z=\"0.0\" A=\"0.0\" B=\"0.0\" C=\"0.0\" /><AIPos A1=\"0.0\"" \
    " A2=\"-90.0\" A3=\"90.0\" A4=\"0.0\" A5=\"90.0\" A6=\"0.0\" /><ASPos A1=\"0.0\" A2=\"-90.0\"" \
    " A3=\"90.0\" A4=\"0.0\" A5=\"90.0\" A6=\"0.0\" /><Delay D=\"0\" /><IPOC>0</IPOC></Rob>";
  ASSERT_TRUE(pugi_command_handler.Decode(input_string, 1024));
}
TEST_F(PugiDecodeTest, DecodeParamValid) {
  kuka_rsi_hw_interface::PugiCommandHandler pugi_command_handler(1024);

  char input_string[] =
    "<Rob TYPE=\"KUKA\"><RIst X=\"0.0\" Y=\"0.0\" Z=\"0.0\" A=\"0.0\" B=\"0.0\" C=\"0.0\"" \
    " /><RSol X=\"0.0\" Y=\"0.0\" Z=\"0.0\" A=\"0.0\" B=\"0.0\" C=\"0.0\" /><AIPos A1=\"0.0\"" \
    " A2=\"-90.0\" A3=\"90.0\" A4=\"0.0\" A5=\"90.0\" A6=\"0.0\" /><ASPos A1=\"0.0\" A2=\"-90.0\"" \
    " A3=\"90.0\" A4=\"0.0\" A5=\"90.0\" A6=\"0.0\" /><Delay D=\"0\" /><IPOC>0</IPOC></Rob>";
  ASSERT_TRUE(pugi_command_handler.Decode(input_string, 1024));
  ASSERT_TRUE(
    pugi_command_handler.state_doc.child("Rob").child("AIPos").attribute(
      "A5").as_double() == 90.0);
  ASSERT_TRUE(pugi_command_handler.state_doc.child("Rob").child("IPOC").text().as_llong() == 0);
}
TEST_F(PugiDecodeTest, DecodeTimeMeasurement) {
  // Memory manager
  // Set default resource
  kuka_rsi_hw_interface::print_resource default_resource("Rogue PMR Allocation!",
    std::pmr::null_memory_resource());
  std::pmr::set_default_resource(&default_resource);

  // create memory resource
  kuka_rsi_hw_interface::print_resource out_of_memory("Out of Memory",
    std::pmr::null_memory_resource());

  std::array<std::byte, 600000> buffer{};
  std::pmr::monotonic_buffer_resource underlying_bytes{buffer.data(), buffer.size(),
    &out_of_memory};

  //kuka_rsi_hw_interface::print_resource monotonic("Monotonic Array", &underlying_bytes);

  // Create pool resource
  std::pmr::pool_options pool_options;
  pool_options.largest_required_pool_block = 35000;
  std::pmr::synchronized_pool_resource sync_pool{pool_options, &underlying_bytes};
  //kuka_rsi_hw_interface::print_resource memory_pool("Pool", &sync_pool);

  // Create memory handler
  kuka_rsi_hw_interface::MemoryManager memory_manager(&sync_pool);
  kuka_rsi_hw_interface::memory_manager_handler = &memory_manager;

  pugi::set_memory_management_functions(
    kuka_rsi_hw_interface::custom_allocate,
    kuka_rsi_hw_interface::custom_deallocate);

  kuka_rsi_hw_interface::PugiCommandHandler pugi_command_handler(1024);

  char input_string[] =
    "<Rob TYPE=\"KUKA\"><RIst X=\"0.0\" Y=\"0.0\" Z=\"0.0\" A=\"0.0\" B=\"0.0\" C=\"0.0\"" \
    " /><RSol X=\"0.0\" Y=\"0.0\" Z=\"0.0\" A=\"0.0\" B=\"0.0\" C=\"0.0\" /><AIPos A1=\"0.0\"" \
    " A2=\"-90.0\" A3=\"90.0\" A4=\"0.0\" A5=\"90.0\" A6=\"0.0\" /><ASPos A1=\"0.0\" A2=\"-90.0\"" \
    " A3=\"90.0\" A4=\"0.0\" A5=\"90.0\" A6=\"0.0\" /><Delay D=\"0\" /><IPOC>0</IPOC></Rob>";
  bool ret_val = true;
  std::ofstream timer_data;

  timer_data.open("PugiDecodeTest_DecodeTimeMeasurement.txt");

  struct sched_param param;
  param.sched_priority = 98;
  ret_val &= (sched_setscheduler(0, SCHED_FIFO, &param) != -1);


  for (size_t i = 0; i < 4000; i++) {
    //std::cout << "[Debug] Decode started" << std::endl;
    auto start = std::chrono::high_resolution_clock::now();
    ret_val &= pugi_command_handler.Decode(input_string, 1024);
    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(stop - start);
    timer_data << i << " " << duration.count() << std::endl;
  }
  ASSERT_TRUE(ret_val);
}
