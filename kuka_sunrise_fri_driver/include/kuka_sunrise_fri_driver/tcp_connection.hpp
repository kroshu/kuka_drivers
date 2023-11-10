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

#ifndef KUKA_SUNRISE_FRI_DRIVER__TCP_CONNECTION_HPP_
#define KUKA_SUNRISE_FRI_DRIVER__TCP_CONNECTION_HPP_

#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <pthread.h>

#include <stdexcept>
#include <string>
#include <functional>
#include <vector>
#include <atomic>

namespace kuka_sunrise_fri_driver
{

class TCPConnection
{
public:
  TCPConnection(
    const char * server_addr, const int server_port,
    std::function<void(const std::vector<std::uint8_t> &)> data_received_callback,
    std::function<void(const char * server_addr, const int server_port)> connection_lost_callback);

  bool sendByte(std::uint8_t data);
  bool sendBytes(const std::vector<std::uint8_t> & data);
  void closeConnection();

  ~TCPConnection();
  TCPConnection(const TCPConnection &) = delete;
  TCPConnection & operator=(const TCPConnection &) = delete;
  TCPConnection & operator=(TCPConnection && from);

private:
  static void * listen_helper(void * tcpConnection);
  void listen();
  std::function<void(const std::vector<std::uint8_t> &)> dataReceivedCallback_;
  std::function<void(const char *, int)> connectionLostCallback_;

  int socket_desc_;
  struct sockaddr_in server_;
  pthread_t read_thread_;
  std::atomic_bool cancelled_;
};

}  // namespace kuka_sunrise_fri_driver

#endif  // KUKA_SUNRISE_FRI_DRIVER__TCP_CONNECTION_HPP_
