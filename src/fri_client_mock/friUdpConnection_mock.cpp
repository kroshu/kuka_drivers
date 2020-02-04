/*
 * friUdpConnection_mock.cpp
 *
 *  Created on: Feb 4, 2020
 *      Author: rosdeveloper
 */

#include "fri_client/friUdpConnection.h"

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


