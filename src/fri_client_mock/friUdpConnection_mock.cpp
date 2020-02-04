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

UdpConnection::UdpConnection(unsigned int receiveTimeout = 0):
    _udpSock(0),
    _receiveTimeout(0)
{

}

UdpConnection::~UdpConnection(){}

bool UdpConnection::open(int port, const char *controllerAddress = NULL)
{
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
  return 0;
}

bool UdpConnection::send(const char *buffer, int size)
{
  return false;
}

}
}


