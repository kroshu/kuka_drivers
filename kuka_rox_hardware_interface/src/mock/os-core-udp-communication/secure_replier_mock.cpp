// This material is the exclusive property of KUKA Deutschland GmbH.
// Except as expressly permitted by separate agreement, this material may only
// be used by members of the development department of KUKA Deutschland GmbH for
// internal development purposes of KUKA Deutschland GmbH.
//
// Copyright (C)
// KUKA Deutschland GmbH, Germany. All Rights Reserved.

#include "os-core-udp-communication/secure_replier.h"

namespace os::core::udp::communication
{

SecureReplier::SecureReplier(const std::string& certificate_path, const std::string& private_key_path, const SocketAddress & local_address)
: local_address_(local_address) {}

Socket::ErrorCode SecureReplier::Setup()
{
  // only a mock
  return Socket::ErrorCode::kSuccess;
}

void SecureReplier::Reset()
{
  // only a mock
}

Socket::ErrorCode SecureReplier::ReceiveRequest()
{
  // only a mock
  return Socket::ErrorCode::kSuccess;
}

Socket::ErrorCode SecureReplier::ReceiveRequestOrTimeout(std::chrono::microseconds recv_timeout)
{
  // only a mock
  return Socket::ErrorCode::kSuccess;
}

Socket::ErrorCode SecureReplier::SendReply(uint8_t * reply_msg_data, size_t reply_msg_size)
{
  // only a mock
  return Socket::ErrorCode::kSuccess;
}

std::pair<const uint8_t *, size_t> SecureReplier::GetRequestMessage() const
{
  return {nullptr, 0};
}

}  // namespace os::core::udp::communication
