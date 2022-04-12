/*
 * service_tools.hpp
 *
 *  Created on: Nov 19, 2019
 *      Author: rosdeveloper
 */

#ifndef INCLUDE_KUKA_SUNRISE_INTERNAL_SERVICE_TOOLS_HPP_
#define INCLUDE_KUKA_SUNRISE_INTERNAL_SERVICE_TOOLS_HPP_

#include <future>
#include <rclcpp/rclcpp.hpp>

namespace teleop_guided_robot
{

template<typename FutureT, typename WaitTimeT>
std::future_status wait_for_result(FutureT &future, WaitTimeT time_to_wait)
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
}

#endif /* INCLUDE_KUKA_SUNRISE_INTERNAL_SERVICE_TOOLS_HPP_ */
