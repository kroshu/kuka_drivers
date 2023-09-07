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

#ifndef KUKA_RSI_HW_INTERFACE__PUGI_COMMAND_HANDLER_HPP_
#define KUKA_RSI_HW_INTERFACE__PUGI_COMMAND_HANDLER_HPP_

#include <pugixml.hpp>
#include <cstring>

namespace kuka_rsi_hw_interface
{
struct xml_memory_writer : pugi::xml_writer
{
  char * buffer_;
  size_t capacity_;
  size_t result_;

  xml_memory_writer()
  : buffer_(0), capacity_(0), result_(0) {}
  xml_memory_writer(char * buffer, size_t capacity)
  : buffer_(buffer), capacity_(capacity), result_(0) {}

  size_t written_size() const
  {
    return result_ < capacity_ ? result_ : capacity_;
  }

  virtual void write(const void * data, size_t size)
  {
    if (result_ < capacity_) {
      size_t chunk = (capacity_ - result_ < size) ? capacity_ - result_ : size;
      memcpy(buffer_ + result_, data, chunk);
    }
    result_ += size;
  }
};

class PugiCommandHandler
{
private:
  char * state_buffer;

public:
  pugi::xml_document state_doc;
  pugi::xml_document command_doc;

  PugiCommandHandler(const size_t buffer_size);
  ~PugiCommandHandler();

  bool Decode(const char * buffer, const size_t buffer_size);
  char * Encode(char * buffer, const size_t buffer_size);
};
}  // namespace kuka_rsi_hw_interface


#endif  // KUKA_RSI_HW_INTERFACE__PUGI_COMMAND_HANDLER_HPP_
