// This material is the exclusive property of KUKA Deutschland GmbH.
// Except as expressly permitted by separate agreement, this material may only
// be used by members of the development department of KUKA Deutschland GmbH for
// internal development purposes of KUKA Deutschland GmbH.
//
// Copyright (C)
// KUKA Deutschland GmbH, Germany. All Rights Reserved.

#include "os-core-udp-communication/udp_socket.h"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#include <cstring>
#include <string>

namespace os::core::udp::communication {

SocketAddress::SocketAddress() {
  // only a mock
}

SocketAddress::SocketAddress(const std::string& ip, int port) {
  // only a mock
}

SocketAddress::SocketAddress(const std::string& ip) {
  // only a mock
}

SocketAddress::SocketAddress(int port) {
  // only a mock
}

SocketAddress::SocketAddress(const struct sockaddr_in* raw_address) {
  // only a mock
}

struct sockaddr* SocketAddress::RawAddr() {
  return nullptr;
}

const struct sockaddr* SocketAddress::RawAddr() const {
  return nullptr;
}

struct sockaddr_in* SocketAddress::RawInetAddr() {
  return nullptr;
}

const struct sockaddr_in* SocketAddress::RawInetAddr() const { return nullptr; }

size_t SocketAddress::Size() const { return 0; }
const std::string SocketAddress::Ip() const { return ""; }
uint16_t SocketAddress::Port() const { return 0; }

UDPSocket::~UDPSocket() {
  // only a mock
}

int UDPSocket::Map(int flags) {
  return 0;
}

int UDPSocket::SetSocketOption(int level, int optname, const void* optval, socklen_t optlen) {
  return 0;
}

int UDPSocket::SetReuseAddress(int flag) {
  return 0;
}

int UDPSocket::SetReceiveBufferSize(int size) {
  return 0;
}

int UDPSocket::SetSendTimeout(const std::chrono::microseconds& timeout) {
  return 0;
}

int UDPSocket::SetReceiveTimeout(const std::chrono::microseconds& timeout) {
  return 0;
}

int UDPSocket::JoinMulticastGroup(const SocketAddress& multicast_address) {
  return 0;
}

int UDPSocket::LeaveMulticastGroup(const SocketAddress& multicast_address) {
  return 0;
}

int UDPSocket::SetTTLForMulticast(int ttl) {
  return 0;
}

int UDPSocket::SetTTLForUnicast(int ttl) {
  return 0;
}

int UDPSocket::Bind(const SocketAddress& local_address) {
  return 0;
}

int UDPSocket::Connect(const SocketAddress& remote_address) {
  return 0;
}

int UDPSocket::Select(std::chrono::microseconds timeout, bool read) {
  // TODO(kovacsge): check
  return 1;
}

int UDPSocket::Send(const unsigned char* raw_data, int raw_data_size, int flags) {
  return 0;
}

int UDPSocket::SendTo(const SocketAddress& remote_address, const unsigned char* raw_data,
                      int raw_data_size, int flags) {
  return 0;
}

int UDPSocket::Receive(unsigned char* buffer, int buffer_size, int flags) {
  return 0;
}

int UDPSocket::ReceiveOrTimeout(const std::chrono::microseconds& timeout, unsigned char* buffer,
                                int buffer_size, int flags) {
  return 0;
}

int UDPSocket::ReceiveFrom(SocketAddress& incoming_remote_address, unsigned char* buffer,
                           int buffer_size, int flags) {
  return 0;
}

int UDPSocket::ReceiveFromOrTimeout(const std::chrono::microseconds& timeout,
                                    SocketAddress& incoming_remote_address, unsigned char* buffer,
                                    int buffer_size, int flags) {
  return 0;
}

int UDPSocket::Close() {
  return ErrorCode::kSuccess;
}

int UDPSocket::GetErrorCode() const { return errno; }
std::string UDPSocket::GetErrorText() const { return ""; }

bool UDPSocket::IsReadable() const {
  // TODO(kovacsge): is there any packet - MSG_PEEK
  throw "NOT IMPLEMENTED";
}

}  // namespace os::core::udp::communication
