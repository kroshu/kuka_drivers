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

#ifndef COMMUNICATION_HELPERS__SERVICE_TOOLS_HPP_
#define COMMUNICATION_HELPERS__SERVICE_TOOLS_HPP_

#include <future>
#include <memory>

#include "rclcpp/rclcpp.hpp"

namespace kuka_drivers_core
{
template <typename FutureT, typename WaitTimeT>
std::future_status wait_for_result(FutureT & future, WaitTimeT time_to_wait)
{
  auto end = std::chrono::steady_clock::now() + time_to_wait;
  std::chrono::milliseconds wait_period(100);
  std::future_status status = std::future_status::timeout;
  do
  {
    auto now = std::chrono::steady_clock::now();
    auto time_left = end - now;
    if (time_left <= std::chrono::seconds(0))
    {
      break;
    }
    status = future.wait_for((time_left < wait_period) ? time_left : wait_period);
  } while (rclcpp::ok() && status != std::future_status::ready);
  return status;
}

template <typename ResponseT, typename RequestT, typename ClientT>
std::shared_ptr<ResponseT> sendRequest(
  ClientT client, RequestT request, const uint32_t & service_timeout_ms = 2000,
  const uint32_t & response_timeout_ms = 100)
{
  if (
    service_timeout_ms && !client->wait_for_service(std::chrono::milliseconds(service_timeout_ms)))
  {
    printf("Wait for service failed\n");
    return nullptr;
  }
  auto future_result = client->async_send_request(request);
  auto future_status =
    wait_for_result(future_result, std::chrono::milliseconds(response_timeout_ms));
  if (future_status != std::future_status::ready)
  {
    printf("Request timed out\n");
    return nullptr;
  }
  return future_result.get();
}
}  // namespace kuka_drivers_core

#endif  // COMMUNICATION_HELPERS__SERVICE_TOOLS_HPP_
