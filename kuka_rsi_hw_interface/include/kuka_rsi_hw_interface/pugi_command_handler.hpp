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
#include <memory_resource>
#include <iostream>
#include <cassert>
#include <map>
#include <functional>

namespace kuka_rsi_hw_interface
{

void * custom_allocate(size_t size);
void custom_deallocate(void * ptr);

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

class print_resource : public std::pmr::memory_resource
{
public:
  print_resource(std::string name, std::pmr::memory_resource * upstream)
  : name_(name), upstream_(upstream)
  {
    assert(upstream);
  }

private:
  std::string name_;
  std::pmr::memory_resource * upstream_;

  void * do_allocate(std::size_t bytes, std::size_t alignment) override
  {
    std::cout << "[" << name_ << " (alloc)] Size: " << bytes << " Alignment: " << alignment <<
      " ..." << std::endl;
    auto result = upstream_->allocate(bytes, alignment);
    std::cout << "[" << name_ << " (alloc)] ... Address: " << result << std::endl;
    return result;
  }

  void do_deallocate(void * ptr, std::size_t bytes, std::size_t alignment) override
  {
    std::cout << "[" << name_ << " (dealloc)] Address: " << ptr << "Size: " << bytes <<
      " Alignment: " << alignment << std::endl;
    upstream_->deallocate(ptr, bytes, alignment);
  }

  bool do_is_equal(const std::pmr::memory_resource & other) const noexcept override
  {
    return this == &other;
  }
};

class MemoryManager
{
public:
  MemoryManager(std::pmr::memory_resource * upstream);
  void * Allocate(size_t size);
  void Deallocate(void * ptr);

private:
  //std::pmr::polymorphic_allocator<std::byte> allocator_;
  std::pmr::memory_resource * memory_pool_;
  std::pmr::map<void *, size_t> memory_pool_sizes_;
};

class PugiCommandHandler
{
public:
  pugi::xml_document state_doc;
  pugi::xml_document command_doc;

  PugiCommandHandler(const size_t buffer_size);
  ~PugiCommandHandler();

  bool Decode(const char * buffer, const size_t buffer_size);
  char * Encode(char * buffer, const size_t buffer_size);

private:
  char * state_buffer_;
};

extern MemoryManager * memory_manager_handler;
}  // namespace kuka_rsi_hw_interface


#endif  // KUKA_RSI_HW_INTERFACE__PUGI_COMMAND_HANDLER_HPP_
