// Copyright 2024 KUKA Hungaria Kft.
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

#ifndef KUKA_DRIVERS_CORE__HARDWARE_INTERFACE_UTILS_HPP_
#define KUKA_DRIVERS_CORE__HARDWARE_INTERFACE_UTILS_HPP_

#include <chrono>
#include <cstdint>
#include <thread>

namespace kuka_drivers_core
{
namespace hardware_interface_utils
{
/**
 * @brief Wait for interpolation count to match expected value with retry logic for async hardware.
 *
 * Async hardware components may lag one cycle behind controller updates.
 * This function retries up to 1 ms if the current count is exactly one behind expected.
 *
 * @param expected_count The expected interpolation count value.
 * @param current_count The current interpolation count value (will be updated during retries).
 * @param is_async_hardware Whether the hardware is running asynchronously.
 * @param get_current_count Callback to get the current interpolation count.
 * @return The final interpolation count value after waiting.
 */
template <typename GetCountFunc>
inline uint32_t WaitForInterpolationCount(
  uint32_t expected_count, uint32_t current_count, bool is_async_hardware,
  GetCountFunc get_current_count)
{
  if (!is_async_hardware || current_count != expected_count - 1)
  {
    return current_count;
  }

  const auto retry_deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(1);
  const auto retry_step =
    std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::microseconds(200));

  // Async components may lag one cycle behind controller updates; retry up to 1 ms
  while (current_count == expected_count - 1)
  {
    const auto now = std::chrono::steady_clock::now();
    if (now >= retry_deadline)
    {
      break;
    }

    auto sleep_time = retry_step;
    const auto remaining = retry_deadline - now;
    if (remaining < sleep_time)
    {
      sleep_time = remaining;
    }

    std::this_thread::sleep_for(sleep_time);
    current_count = get_current_count();
  }

  return current_count;
}

}  // namespace hardware_interface_utils
}  // namespace kuka_drivers_core

#endif  // KUKA_DRIVERS_CORE__HARDWARE_INTERFACE_UTILS_HPP_
