// This material is the exclusive property of KUKA Deutschland GmbH.
// Except as expressly permitted by separate agreement, this material may only
// be used by members of the development department of KUKA Deutschland GmbH for
// internal development purposes of KUKA Deutschland GmbH.
//
// Copyright (C)
// KUKA Deutschland GmbH, Germany. All Rights Reserved.

#ifndef UDP_SOCKET_H
#define UDP_SOCKET_H

#include <netinet/in.h>

#include <chrono>
#include <optional>
#include <string>

namespace os::core::udp::communication {

class SocketAddress {
 public:
  SocketAddress();
  SocketAddress(const std::string &ip, int port);
  SocketAddress(const std::string &ip);
  SocketAddress(int port);
  SocketAddress(const struct sockaddr_in *raw_address);

 public:
  struct sockaddr *RawAddr();
  const struct sockaddr *RawAddr() const;
  struct sockaddr_in *RawInetAddr();
  const struct sockaddr_in *RawInetAddr() const;
  size_t Size() const;
  const std::string Ip() const;
  uint16_t Port() const;

 public:
  static const std::string kAnyAddress;
};

class UDPSocket {
 public:
  enum ErrorCode {
    kSuccess = 0,
    kSocketError = -1,
    kNotActive = -2,
    kAlreadyActive = -2,
    kNotBound = -3,
    kNotConnected = -4,
    kTimeout = -5,
    kError = -6
  };

 public:
  virtual ~UDPSocket();

 public:

  int Map(int flags = 0);

  int SetSocketOption(int level, int optname, const void *optval, socklen_t optlen);

  int SetSendTimeout(const std::chrono::microseconds &timeout);

  int SetReceiveTimeout(const std::chrono::microseconds &timeout);

  int JoinMulticastGroup(const SocketAddress &multicast_address);

  int LeaveMulticastGroup(const SocketAddress &multicast_address);

  int SetTTLForMulticast(int ttl = 1);

  int SetTTLForUnicast(int ttl = 1);

  int SetReuseAddress(int flag = 1);

  int SetReceiveBufferSize(int size);

  int Bind(const SocketAddress &local_address);

  int Connect(const SocketAddress &remote_address);

  int Select(std::chrono::microseconds timeout, bool read = true);

  int Send(const unsigned char *raw_data, int raw_data_size, int flags = 0);

  int SendTo(const SocketAddress &remote_address, const unsigned char *raw_data, int raw_data_size,
             int flags = 0);

  int Receive(unsigned char *buffer, int buffer_size, int flags = 0);

  int ReceiveOrTimeout(const std::chrono::microseconds &timeout, unsigned char *buffer,
                       int buffer_size, int flags = 0);

  int ReceiveFrom(SocketAddress &incoming_remote_address, unsigned char *buffer, int buffer_size,
                  int flags = 0);

  int ReceiveFromOrTimeout(const std::chrono::microseconds &timeout,
                           SocketAddress &incoming_remote_address, unsigned char *buffer,
                           int buffer_size, int flags = 0);

  int Close();

 public:
  int GetSocketFd() const { return socket_fd_; }
  bool IsActive() const { return socket_fd_ >= 0; }
  int GetErrorCode() const;
  std::string GetErrorText() const;
  bool IsReadable() const;
};

}  // namespace os::core::udp::communication

#endif  // UDP_SOCKET_H
