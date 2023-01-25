#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>

namespace nanopb{

template <typename T>
inline int Encode(const T &, uint8_t *, size_t ) {
  // only a mock
  return -1;
}

template <typename T>
inline bool Decode(const uint8_t *, size_t , T &) {
  // only a mock
  return false;
}
}  // namespace nanopb