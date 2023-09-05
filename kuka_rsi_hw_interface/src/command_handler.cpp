// Copyright 2023 Komaromi Sándor
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

#include "kuka_rsi_hw_interface/command_handler.hpp"

namespace kuka_rsi
{
command_handler::command_handler(const size_t buffer_size)
{
  state_buffer = new char[buffer_size];
}

void command_handler::Decode(const char * const buffer, const size_t buffer_size)
{

}
}  // namespace kuka_rsi
