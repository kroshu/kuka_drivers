// Copyright 2022 Lars Tingelstad
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

#ifndef KUKA_KSS_RSI_DRIVER__UDP_SERVER_HPP_
#define KUKA_KSS_RSI_DRIVER__UDP_SERVER_HPP_

// Select includes
#include <sys/time.h>

#include <arpa/inet.h>
#include <netdb.h>
#include <netinet/in.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#include <iostream>
#include <stdexcept>
#include <string>

#include "rclcpp/rclcpp.hpp"

class UDPServer
{
public:
  UDPServer(std::string host, unsigned short port)
  : local_host_(host), local_port_(port), timeout_(false)
  {
    RCLCPP_INFO(rclcpp::get_logger("UDPServer"), "%s: %i", local_host_.c_str(), local_port_);
    sockfd_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd_ < 0)
    {
      throw std::runtime_error("Error opening socket: " + std::string(strerror(errno)));
    }
    optval = 1;
    setsockopt(sockfd_, SOL_SOCKET, SO_REUSEADDR, (const void *)&optval, sizeof(int));
    memset(&serveraddr_, 0, sizeof(serveraddr_));
    serveraddr_.sin_family = AF_INET;
    serveraddr_.sin_addr.s_addr = inet_addr(local_host_.c_str());
    serveraddr_.sin_port = htons(local_port_);
    if (bind(sockfd_, (struct sockaddr *)&serveraddr_, sizeof(serveraddr_)) < 0)
    {
      throw std::runtime_error("Error binding socket: " + std::string(strerror(errno)));
    }
    clientlen_ = sizeof(clientaddr_);
  }

  ~UDPServer() { close(sockfd_); }

  UDPServer(UDPServer & other) = delete;
  UDPServer & operator=(const UDPServer & other) = delete;

  bool set_timeout(int millisecs)
  {
    if (millisecs != 0)
    {
      tv_.tv_sec = millisecs / 1000;
      tv_.tv_usec = (millisecs % 1000) * 1000;
      timeout_ = true;
      return timeout_;
    }
    else
    {
      return timeout_;
    }
  }

  ssize_t send(std::string & buffer)
  {
    ssize_t bytes = 0;
    bytes = sendto(
      sockfd_, buffer.c_str(), buffer.size(), 0, (struct sockaddr *)&clientaddr_, clientlen_);
    if (bytes < 0)
    {
      RCLCPP_ERROR(rclcpp::get_logger("UDPServer"), "Error in send");
    }

    return bytes;
  }

  ssize_t recv(std::string & buffer)
  {
    ssize_t bytes = 0;

    if (timeout_)
    {
      fd_set read_fds;
      FD_ZERO(&read_fds);
      FD_SET(sockfd_, &read_fds);

      struct timeval tv;
      tv.tv_sec = tv_.tv_sec;
      tv.tv_usec = tv_.tv_usec;

      if (select(sockfd_ + 1, &read_fds, nullptr, nullptr, &tv) < 0)
      {
        return 0;
      }

      if (FD_ISSET(sockfd_, &read_fds))
      {
        memset(buffer_, 0, BUFSIZE);
        bytes =
          recvfrom(sockfd_, buffer_, BUFSIZE, 0, (struct sockaddr *)&clientaddr_, &clientlen_);
        if (bytes < 0)
        {
          RCLCPP_ERROR(rclcpp::get_logger("UDPServer"), "Error in receive");
        }
      }
      else
      {
        return 0;
      }
    }
    else
    {
      memset(buffer_, 0, BUFSIZE);
      bytes = recvfrom(sockfd_, buffer_, BUFSIZE, 0, (struct sockaddr *)&clientaddr_, &clientlen_);
      if (bytes < 0)
      {
        RCLCPP_ERROR(rclcpp::get_logger("UDPServer"), "Error in receive");
      }
    }

    buffer = std::string(buffer_);

    return bytes;
  }

private:
  static const int BUFSIZE = 1024;
  std::string local_host_;
  uint16_t local_port_;
  bool timeout_;
  struct timeval tv_;

  int sockfd_;
  socklen_t clientlen_;
  struct sockaddr_in serveraddr_;
  struct sockaddr_in clientaddr_;
  char buffer_[BUFSIZE];
  int optval;
};

#endif  // KUKA_KSS_RSI_DRIVER__UDP_SERVER_HPP_
