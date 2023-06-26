// This material is the exclusive property of KUKA Deutschland GmbH.
// Except as expressly permitted by separate agreement, this material may only
// be used by members of the development department of KUKA Deutschland GmbH for
// internal development purposes of KUKA Deutschland GmbH.
//
// Copyright (C)
// KUKA Deutschland GmbH, Germany. All Rights Reserved.

#ifndef REPLIER_H
#define REPLIER_H

#include <chrono>

#include "os-core-udp-communication/socket.h"

namespace os::core::udp::communication
{

class Replier
{
public:
  //<ctor>
  Replier(const SocketAddress & local_address);
  virtual ~Replier() = default;
  Socket::ErrorCode Setup();
  void Reset();

public:
  //<operations>
  Socket::ErrorCode ReceiveRequest();
  Socket::ErrorCode ReceiveRequestOrTimeout(std::chrono::microseconds recv_timeout);
  Socket::ErrorCode SendReply(uint8_t * reply_msg_data, size_t reply_msg_size);

public:
  //<properties>
  std::pair<const uint8_t *, size_t> GetRequestMessage() const;

protected:
  static constexpr uint16_t kMaxBufferSize = 1500;
  uint8_t server_buffer_[kMaxBufferSize];

  Socket socket_;
  SocketAddress local_address_;

  bool active_request_ = false;
  SocketAddress last_remote_address_;
  int last_request_size_ = 0;
};
}  // namespace os::core::udp::communication

#endif  // REPLIER_H
