// This material is the exclusive property of KUKA Deutschland GmbH.
// Except as expressly permitted by separate agreement, this material may only
// be used by members of the development department of KUKA Deutschland GmbH for
// internal development purposes of KUKA Deutschland GmbH.
//
// Copyright (C)
// KUKA Deutschland GmbH, Germany. All Rights Reserved.

#include "os-core-udp-communication/replier.h"

namespace os::core::udp::communication
{
Replier::Replier(const SocketAddress & local_address)
{
  // only a mock
}
Socket::ErrorCode Replier::Setup()
{
  // only a mock
}
void Replier::Reset()
{
  // only a mock
}

Socket::ErrorCode Replier::ReceiveRequest()
{
  // only a mock
  return Socket::ErrorCode::kSuccess;
}
Socket::ErrorCode Replier::ReceiveRequestOrTimeout(std::chrono::microseconds recv_timeout)
{
  // only a mock
  return Socket::ErrorCode::kSuccess;
}
Socket::ErrorCode Replier::SendReply(uint8_t * reply_msg_data, size_t reply_msg_size)
{
  // only a mock
  return Socket::ErrorCode::kSuccess;
}

std::pair<const uint8_t *, size_t> Replier::GetRequestMessage() const
{
  // only a mock
  return {nullptr, 0};
}
}  // namespace os::core::udp::communication
