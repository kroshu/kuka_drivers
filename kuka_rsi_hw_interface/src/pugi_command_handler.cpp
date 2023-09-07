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
#include <cstring>

#include "kuka_rsi_hw_interface/pugi_command_handler.hpp"

namespace kuka_rsi_hw_interface
{
PugiCommandHandler::PugiCommandHandler(const size_t buffer_size)
{
  state_buffer = new char[buffer_size];
}

PugiCommandHandler::~PugiCommandHandler()
{
  delete[] state_buffer;
}

bool PugiCommandHandler::Decode(const char * buffer, const size_t buffer_size)
{
  std::memcpy(state_buffer, buffer, buffer_size);
  return state_doc.load_buffer_inplace(state_buffer, buffer_size);
}

char * PugiCommandHandler::Encode(char * buffer, const size_t buffer_size)
{
  if (buffer_size == 0) {
    return nullptr;
  }

  // leave one character for null terminator
  xml_memory_writer writer(buffer, buffer_size - 1);

  command_doc.save(writer, "\t", pugi::format_raw | pugi::format_no_declaration);

  // null terminate
  buffer[writer.written_size()] = 0;

  return buffer;
}
}  // namespace kuka_rsi_hw_interface
