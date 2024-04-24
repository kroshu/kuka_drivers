// Copyright 2020 Zoltán Rési
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

#ifndef KUKA_SUNRISE_FRI_DRIVER__SERIALIZATION_HPP_
#define KUKA_SUNRISE_FRI_DRIVER__SERIALIZATION_HPP_

#include <string.h>
#include <algorithm>
#include <cstdint>
#include <string>
#include <vector>

namespace kuka_sunrise_fri_driver
{

int serializeNext(int integer_in, std::vector<std::uint8_t> & serialized_out)
{
  std::uint8_t * bytes = reinterpret_cast<std::uint8_t *>(&integer_in);
  auto it = serialized_out.end();
  serialized_out.insert(it, bytes, bytes + sizeof(int));

  // Redefine iterator because of possible invalidation
  auto from_it = std::prev(serialized_out.end(), sizeof(int));
  std::reverse(from_it, serialized_out.end());
  // TODO(resizoltan): assert that int is 4 bytes long
  // TODO(resizoltan): check endianness
  return sizeof(int);
}

int deserializeNext(const std::vector<std::uint8_t> & serialized_in, int & integer_out)
{
  if (serialized_in.size() < sizeof(int))
  {
    // TODO(resizoltan): error
  }
  std::vector<std::uint8_t> serialized_copy = serialized_in;
  integer_out = *(reinterpret_cast<int *>(serialized_copy.data()));
  return sizeof(int);
}

int serializeNext(double double_in, std::vector<std::uint8_t> & serialized_out)
{
  std::uint8_t * bytes = reinterpret_cast<std::uint8_t *>(&double_in);
  serialized_out.insert(serialized_out.end(), bytes, bytes + sizeof(double));

  // Redefine iterator because of possible invalidation
  auto from_it = std::prev(serialized_out.end(), sizeof(double));
  std::reverse(from_it, serialized_out.end());
  // TODO(resizoltan): assert that int is 4 bytes long
  // TODO(resizoltan): check endianness
  return sizeof(int);
}

int deserializeNext(const std::vector<std::uint8_t> & serialized_in, double & double_out)
{
  if (serialized_in.size() < sizeof(double))
  {
    // TODO(resizoltan): error
  }
  std::vector<std::uint8_t> serialized_copy = serialized_in;
  double_out = *(reinterpret_cast<int *>(serialized_copy.data()));
  return sizeof(int);
}
}  // namespace kuka_sunrise_fri_driver

#endif  // KUKA_SUNRISE_FRI_DRIVER__SERIALIZATION_HPP_
