// This material is the exclusive property of KUKA Deutschland GmbH.
// Except as expressly permitted by separate agreement, this material may only
// be used by members of the development department of KUKA Deutschland GmbH for
// internal development purposes of KUKA Deutschland GmbH.
//
// Copyright (C)
// KUKA Deutschland GmbH, Germany. All Rights Reserved.

#include "os-core-udp-communication/udp_replier.h"

namespace os::core::udp::communication {

UDPReplier::UDPReplier(const SocketAddress& local_address) : local_address_(local_address) {}

UDPSocket::ErrorCode UDPReplier::Setup() {
    // only a mock
    return UDPSocket::ErrorCode::kSuccess;
}

void UDPReplier::Reset() {
    // only a mock
}

UDPSocket::ErrorCode UDPReplier::ReceiveRequest() {
    // only a mock
    return UDPSocket::ErrorCode::kSuccess;
}

UDPSocket::ErrorCode UDPReplier::ReceiveRequestOrTimeout(std::chrono::microseconds recv_timeout) {
    // only a mock
    return UDPSocket::ErrorCode::kSuccess;
}

UDPSocket::ErrorCode UDPReplier::SendReply(uint8_t* reply_msg_data, size_t reply_msg_size) {
    // only a mock
    return UDPSocket::ErrorCode::kSuccess;
}

std::pair<const uint8_t*, size_t> UDPReplier::GetRequestMessage() const {
    return {nullptr, 0};
}

}  // namespace os::core::udp::communication
