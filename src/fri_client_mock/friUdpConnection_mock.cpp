// Copyright 2020 Zoltán Rési
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <fri_client/friUdpConnection.h>

namespace KUKA
{
namespace FRI
{

UdpConnection::UdpConnection(unsigned int receiveTimeout):
    _udpSock(0),
    _receiveTimeout(receiveTimeout)
{

}

UdpConnection::~UdpConnection(){}

bool UdpConnection::open(int port, const char *controllerAddress)
{
  (void)port;
  (void)controllerAddress;
  return false;
}

void UdpConnection::close()
{
  return;
}

bool UdpConnection::isOpen() const
{
  return false;
}

int UdpConnection::receive(char *buffer, int maxSize)
{
  (void)buffer;
  (void)maxSize;
  return 0;
}

bool UdpConnection::send(const char *buffer, int size)
{
  (void)buffer;
  (void)size;
  return false;
}

}
}


