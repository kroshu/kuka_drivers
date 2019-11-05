/*
 * tcp_connection.hpp
 *
 *  Created on: Nov 4, 2019
 *      Author: rosdeveloper
 */

#ifndef INCLUDE_IIWA_INTERFACE_TCP_CONNECTION_HPP_
#define INCLUDE_IIWA_INTERFACE_TCP_CONNECTION_HPP_

#include <sys/socket.h>
#include <arpa/inet.h>
#include <stdexcept>
#include <string>
#include <unistd.h>
#include <pthread.h>
#include <functional>
#include <vector>
#include <atomic>

namespace iiwa_interface{

class TCPConnection{

public:
  TCPConnection(const char* server_addr, const int server_port,
                std::function<void(const std::vector<char>&)> data_received_callback, std::function<void(void)> connection_lost_callback);

  bool sendByte(char data);
  bool sendBytes(const std::vector<char>& data);
  void closeConnection();

  ~TCPConnection();

private:
  TCPConnection(const TCPConnection&) = delete;
  TCPConnection& operator=(const TCPConnection&) = delete;

  static void* listen_helper(void *tcpConnection);
  void listen();
  std::function<void(const std::vector<char>&)> dataReceivedCallback_;
  std::function<void(void)> connectionLostCallback_;

  int socket_desc_;
  struct sockaddr_in server_;
  pthread_t read_thread_;
  std::atomic_bool cancelled_;
};


}
#endif /* INCLUDE_IIWA_INTERFACE_TCP_CONNECTION_HPP_ */
