/*
 * serializer.hpp
 *
 *  Created on: Nov 5, 2019
 *      Author: rosdeveloper
 */

#ifndef INCLUDE_KUKA_SUNRISE_SERIALIZATION_HPP_
#define INCLUDE_KUKA_SUNRISE_SERIALIZATION_HPP_

#include <vector>
#include <cstdint>
#include <algorithm>

namespace kuka_sunrise
{

int serializeNext(int integer_in, std::vector<std::uint8_t> &serialized_out)
{
  std::uint8_t *bytes = reinterpret_cast<std::uint8_t*>(&integer_in);
  auto it = serialized_out.end();
  serialized_out.insert(it, bytes, bytes + sizeof(int));
  std::reverse(it, serialized_out.end());
  //TODO: assert that int is 4 bytes long
  //TODO: check endiannes
  return sizeof(int);
}

int deserializeNext(const std::vector<std::uint8_t> &serialized_in, int &integer_out)
{
  if (serialized_in.size() < sizeof(int))
  {
    //TODO: error
  }
  std::vector<std::uint8_t> serialized_copy = serialized_in;
  integer_out = *(reinterpret_cast<int*>(serialized_copy.data()));
  return sizeof(int);
}

int serializeNext(double double_in, std::vector<std::uint8_t> &serialized_out)
{
  std::uint8_t *bytes = reinterpret_cast<std::uint8_t*>(&double_in);
  auto it = serialized_out.end();
  serialized_out.insert(serialized_out.end(), bytes, bytes + sizeof(double));
  std::reverse(it, serialized_out.end());
  //TODO: assert that int is 4 bytes long
  //TODO: check endiannes
  return sizeof(int);
}

int deserializeNext(const std::vector<std::uint8_t> &serialized_in, double &double_out)
{
  if (serialized_in.size() < sizeof(double))
  {
    //TODO: error
  }
  std::vector<std::uint8_t> serialized_copy = serialized_in;
  double_out = *(reinterpret_cast<int*>(serialized_copy.data()));
  return sizeof(int);
}

}

#endif /* INCLUDE_KUKA_SUNRISE_SERIALIZATION_HPP_ */
