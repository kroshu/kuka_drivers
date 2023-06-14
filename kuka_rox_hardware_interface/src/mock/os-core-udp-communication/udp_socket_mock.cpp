// This material is the exclusive property of KUKA Deutschland GmbH.
// Except as expressly permitted by separate agreement, this material may only
// be used by members of the development department of KUKA Deutschland GmbH for
// internal development purposes of KUKA Deutschland GmbH.
//
// Copyright (C)
// KUKA Deutschland GmbH, Germany. All Rights Reserved.


#include <arpa/inet.h>
#include <netinet/in.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#include <cstring>
#include <string>

#include "os-core-udp-communication/socket.h"

namespace os::core::udp::communication
{

SocketAddress::SocketAddress()
{
  // only a mock
}

SocketAddress::SocketAddress(const std::string & ip, int port)
{
  // only a mock
}

SocketAddress::SocketAddress(const std::string & ip)
{
  // only a mock
}

SocketAddress::SocketAddress(int port)
{
  // only a mock
}

SocketAddress::SocketAddress(const struct sockaddr_in * raw_address)
{
  // only a mock
}

struct sockaddr * SocketAddress::RawAddr()
{
  return nullptr;
}

const struct sockaddr * SocketAddress::RawAddr() const
{
  return nullptr;
}

struct sockaddr_in * SocketAddress::RawInetAddr()
{
  return nullptr;
}

const struct sockaddr_in * SocketAddress::RawInetAddr() const {return nullptr;}

size_t SocketAddress::Size() const {return 0;}
const std::string SocketAddress::Ip() const {return "";}
uint16_t SocketAddress::Port() const {return 0;}

Socket::~Socket()
{
  // only a mock
}

int Socket::Map(int flags)
{
  return 0;
}

int Socket::SetSocketOption(int level, int optname, const void * optval, socklen_t optlen)
{
  return 0;
}

int Socket::SetReuseAddress(int flag)
{
  return 0;
}

int Socket::SetReceiveBufferSize(int size)
{
  return 0;
}

int Socket::SetSendTimeout(const std::chrono::microseconds & timeout)
{
  return 0;
}

int Socket::SetReceiveTimeout(const std::chrono::microseconds & timeout)
{
  return 0;
}

int Socket::JoinMulticastGroup(const SocketAddress & multicast_address)
{
  return 0;
}

int Socket::LeaveMulticastGroup(const SocketAddress & multicast_address)
{
  return 0;
}

int Socket::SetTTLForMulticast(int ttl)
{
  return 0;
}

int Socket::SetTTLForUnicast(int ttl)
{
  return 0;
}

int Socket::Bind(const SocketAddress & local_address)
{
  return 0;
}

int Socket::Connect(const SocketAddress & remote_address)
{
  return 0;
}

int Socket::Select(std::chrono::microseconds timeout, bool read)
{
  return 1;
}

int Socket::Send(const unsigned char * raw_data, int raw_data_size, int flags)
{
  return 0;
}

int Socket::SendTo(
  const SocketAddress & remote_address, const unsigned char * raw_data,
  int raw_data_size, int flags)
{
  return 0;
}

int Socket::Receive(unsigned char * buffer, int buffer_size, int flags)
{
  return 0;
}

int Socket::ReceiveOrTimeout(
  const std::chrono::microseconds & timeout, unsigned char * buffer,
  int buffer_size, int flags)
{
  return 0;
}

int Socket::ReceiveFrom(
  SocketAddress & incoming_remote_address, unsigned char * buffer,
  int buffer_size, int flags)
{
  return 0;
}

int Socket::ReceiveFromOrTimeout(
  const std::chrono::microseconds & timeout,
  SocketAddress & incoming_remote_address, unsigned char * buffer,
  int buffer_size, int flags)
{
  return 0;
}

int Socket::Close()
{
  return ErrorCode::kSuccess;
}

int Socket::GetErrorCode() const {return errno;}
std::string Socket::GetErrorText() const {return "";}

bool Socket::IsReadable() const
{
  return false;
}

}  // namespace os::core::udp::communication
