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

 protected:
  struct sockaddr_in sockaddr_;
  char ip_[INET_ADDRSTRLEN];
  uint16_t port_;
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
  /* creates a new socket with the given `flags`, and returns the underlaying
   * socket identifer */
  int Map(int flags = 0);

  /*sets generic socket option*/
  int SetSocketOption(int level, int optname, const void *optval, socklen_t optlen);

  /* sets a given `timeout` (in ms) for *send* operation to wait if there was
   * no packet send */
  int SetSendTimeout(const std::chrono::microseconds &timeout);

  /* sets a given `timeout` (in ms) for *receive* operation to wait if there
   * was no packet receive*/
  int SetReceiveTimeout(const std::chrono::microseconds &timeout);

  /* join a multicast group with the given `multicast_address` to able receive
   * the packets*/
  int JoinMulticastGroup(const SocketAddress &multicast_address);

  /* leave a multicast group with the given `multicast_address`*/
  int LeaveMulticastGroup(const SocketAddress &multicast_address);

  /* sets the given `ttl`for multicast sending (IP packet property) */
  int SetTTLForMulticast(int ttl = 1);

  /* sets the given `ttl`for unicast sending (IP packet property) */
  int SetTTLForUnicast(int ttl = 1);

  /* sets the to socket to be able to reuse address */
  int SetReuseAddress(int flag = 1);

  /* sets the to sockets rx buffer size */
  int SetReceiveBufferSize(int size);

  /* binds the socket to the given `local_address` */
  int Bind(const SocketAddress &local_address);

  /* connects the socket to the given `remote_address` */
  int Connect(const SocketAddress &remote_address);

  /* wait for event(read or write) on the socket until given time*/
  int Select(std::chrono::microseconds timeout, bool read = true);

  /* sends the given `raw_data` to the previously connected remote address */
  int Send(const unsigned char *raw_data, int raw_data_size, int flags = 0);

  /* sends the given `raw_data` to the given `remote_address` */
  int SendTo(const SocketAddress &remote_address, const unsigned char *raw_data, int raw_data_size,
             int flags = 0);

  /* receives a packet on the previously bind address into the given `buffer`
   */
  int Receive(unsigned char *buffer, int buffer_size, int flags = 0);

  /* receives a packet on the previously bind address into the given `buffer`
   * or timeout after the given `timeout` */
  int ReceiveOrTimeout(const std::chrono::microseconds &timeout, unsigned char *buffer,
                       int buffer_size, int flags = 0);

  /* receives a packet on the previously bind address into the given `buffer`,
   * extracting the remote address to `incoming_remote_address` */
  int ReceiveFrom(SocketAddress &incoming_remote_address, unsigned char *buffer, int buffer_size,
                  int flags = 0);

  /* receives a packet on the previously bind address into the given `buffer`,
   * extracting the remote address to `incoming_remote_address` or timeout
   * after the given `timeout` */
  int ReceiveFromOrTimeout(const std::chrono::microseconds &timeout,
                           SocketAddress &incoming_remote_address, unsigned char *buffer,
                           int buffer_size, int flags = 0);

  /* freeing up the socket */
  int Close();

 public:
  int GetSocketFd() const { return socket_fd_; }
  bool IsActive() const { return socket_fd_ >= 0; }
  int GetErrorCode() const;
  std::string GetErrorText() const;
  bool IsReadable() const;

 protected:
  int socket_fd_ = -1;
  std::optional<SocketAddress> local_address_;
  std::optional<SocketAddress> remote_address_;

  static constexpr uint32_t kMicroToSec = 1000000;
};

}  // namespace os::core::udp::communication

#endif  // UDP_SOCKET_H
