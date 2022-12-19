// This material is the exclusive property of KUKA Deutschland GmbH.
// Except as expressly permitted by separate agreement, this material may only
// be used by members of the development department of KUKA Deutschland GmbH for
// internal development purposes of KUKA Deutschland GmbH.
//
// Copyright (C)
// KUKA Deutschland GmbH, Germany. All Rights Reserved.

#ifndef UDP_REPLIER_H
#define UDP_REPLIER_H

#include <chrono>

#include "os-core-udp-communication/udp_socket.h"

namespace os::core::udp::communication {

class UDPReplier {
 public:  //<ctor>
  UDPReplier(const SocketAddress& local_address);
  virtual ~UDPReplier() = default;
  UDPSocket::ErrorCode Setup();
  void Reset();

 public:  //<operations>
  UDPSocket::ErrorCode ReceiveRequest();
  UDPSocket::ErrorCode ReceiveRequestOrTimeout(std::chrono::microseconds recv_timeout);
  UDPSocket::ErrorCode SendReply(uint8_t* reply_msg_data, size_t reply_msg_size);

 public:  //<properties>
  std::pair<const uint8_t*, size_t> GetRequestMessage() const;

 protected:
  static constexpr uint16_t kMaxBufferSize = 1500;
  uint8_t server_buffer_[kMaxBufferSize];

  UDPSocket socket_;
  SocketAddress local_address_;

  bool active_request_ = false;
  SocketAddress last_remote_address_;
  int last_request_size_ = 0;
};
}  // namespace os::core::udp::communication

#endif  // UDP_REPLIER_H