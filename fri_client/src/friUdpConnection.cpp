/**

The following license terms and conditions apply, unless a redistribution
agreement or other license is obtained by KUKA Roboter GmbH, Augsburg, Germany.

SCOPE

The software "KUKA Sunrise.Connectivity FRI Client SDK" is targeted to work in
conjunction with the "KUKA Sunrise.Connectivity FastRobotInterface" toolkit.
In the following, the term "software" refers to all material directly
belonging to the provided SDK "Software development kit", particularly source
code, libraries, binaries, manuals and technical documentation.

COPYRIGHT

All Rights Reserved
Copyright (C)  2014-2019 
KUKA Roboter GmbH
Augsburg, Germany

LICENSE 

Redistribution and use of the software in source and binary forms, with or
without modification, are permitted provided that the following conditions are
met:
a) The software is used in conjunction with KUKA products only. 
b) Redistributions of source code must retain the above copyright notice, this
list of conditions and the disclaimer.
c) Redistributions in binary form must reproduce the above copyright notice,
this list of conditions and the disclaimer in the documentation and/or other
materials provided with the distribution. Altered source code of the
redistribution must be made available upon request with the distribution.
d) Modification and contributions to the original software provided by KUKA
must be clearly marked and the authorship must be stated.
e) Neither the name of KUKA nor the trademarks owned by KUKA may be used to
endorse or promote products derived from this software without specific prior
written permission.

DISCLAIMER OF WARRANTY

The Software is provided "AS IS" and "WITH ALL FAULTS," without warranty of
any kind, including without limitation the warranties of merchantability,
fitness for a particular purpose and non-infringement. 
KUKA makes no warranty that the Software is free of defects or is suitable for
any particular purpose. In no event shall KUKA be responsible for loss or
damages arising from the installation or use of the Software, including but
not limited to any indirect, punitive, special, incidental or consequential
damages of any character including, without limitation, damages for loss of
goodwill, work stoppage, computer failure or malfunction, or any and all other
commercial damages or losses. 
The entire risk to the quality and performance of the Software is not borne by
KUKA. Should the Software prove defective, KUKA is not liable for the entire
cost of any service and repair.



\file
\version {1.15}
*/
#include <cstring>
#include <cstdio>
#ifndef _MSC_VER
#include <unistd.h>
#endif

#include <fri/friUdpConnection.h>


#ifdef WIN32
#include <winsock2.h>
#include <Ws2tcpip.h>
#ifdef _MSC_VER
#pragma comment(lib, "ws2_32.lib")
#endif
#endif

using namespace KUKA::FRI;

//******************************************************************************
UdpConnection::UdpConnection(unsigned int receiveTimeout) : 
      _udpSock(-1),
      _receiveTimeout(receiveTimeout)
{
#ifdef WIN32
   WSADATA WSAData;
   WSAStartup(MAKEWORD(2,0), &WSAData);
#endif
}
   
//******************************************************************************
UdpConnection::~UdpConnection()
{
   close();
#ifdef WIN32
   WSACleanup();
#endif
}
   
//******************************************************************************
bool UdpConnection::open(int port, const char *controllerAddress)
{
   struct sockaddr_in servAddr;
   memset(&servAddr, 0, sizeof(servAddr));
   memset(&_controllerAddr, 0, sizeof(_controllerAddr));

   // socket creation
   _udpSock = socket(PF_INET, SOCK_DGRAM, 0);
   if (_udpSock < 0)
   {
      printf("opening socket failed!\n");
      return false;
   }

   // use local server port
   servAddr.sin_family = AF_INET;
   servAddr.sin_port = htons(port);
   servAddr.sin_addr.s_addr = htonl(INADDR_ANY);
   
   if (bind(_udpSock, (struct sockaddr *)&servAddr, sizeof(servAddr)) < 0)
   {
      printf("binding port number %d failed!\n", port);
      close();
      return false;
   }
   // initialize the socket properly
   _controllerAddr.sin_family = AF_INET;
   _controllerAddr.sin_port = htons(port);
   if (controllerAddress)
   {
#ifndef __MINGW32__
      inet_pton(AF_INET, controllerAddress, &_controllerAddr.sin_addr);
#else
      _controllerAddr.sin_addr.s_addr = inet_addr(controllerAddress);
#endif
      if ( connect(_udpSock, (struct sockaddr *)&_controllerAddr, sizeof(_controllerAddr)) < 0)
      {
         printf("connecting to controller with address %s failed !\n", controllerAddress);
         close();
         return false;
      }
   }
   else
   {
       _controllerAddr.sin_addr.s_addr = htonl(INADDR_ANY);
   }
   return true;
}
   
//******************************************************************************
void UdpConnection::close()
{
   if (isOpen())
   {
#ifdef WIN32
      closesocket(_udpSock);
#else
      ::close(_udpSock);
#endif
   }
   _udpSock = -1;
}

//******************************************************************************
bool UdpConnection::isOpen() const
{
   return (_udpSock >= 0);
}
   
//******************************************************************************
int UdpConnection::receive(char *buffer, int maxSize)
{
   if (isOpen())
   {
      /** HAVE_SOCKLEN_T
       Yes - unbelievable: There are differences in standard calling parameters (types) to recvfrom 
       Windows winsock, VxWorks and QNX use int
       newer Posix (most Linuxes) use socklen_t
       */
#ifdef HAVE_SOCKLEN_T
      socklen_t sockAddrSize;
#else
      int sockAddrSize;
#endif
      sockAddrSize = sizeof(struct sockaddr_in);
      /** check for timeout
       Because SO_RCVTIMEO is in Windows not correctly implemented, select is used for the receive time out.
       If a timeout greater than 0 is given, wait until the timeout is reached or a message was received.
       If t, abort the function with an error. 
       */
      if(_receiveTimeout > 0)
      {
          
          // Set up struct timeval
          struct timeval tv;
          tv.tv_sec = _receiveTimeout / 1000;
          tv.tv_usec = (_receiveTimeout % 1000) * 1000;

          // initialize file descriptor
          /**
          * Replace FD_ZERO with memset, because bzero is not available for VxWorks 
          * User Space Aplications(RTPs). Therefore the macro FD_ZERO does not compile.
          */
#ifndef VXWORKS
          FD_ZERO(&_filedescriptor);
#else
          memset((char *)(&_filedescriptor), 0, sizeof(*(&_filedescriptor)));
#endif
          FD_SET(_udpSock, &_filedescriptor);
          
          // wait until something was received
          int numberActiveFileDescr = select(_udpSock+1, &_filedescriptor,NULL,NULL,&tv);
          // 0 indicates a timeout
          if(numberActiveFileDescr == 0)
          {
              printf("The connection has timed out. Timeout is %d\n", _receiveTimeout);
              return -1;
          }
          // a negative value indicates an error
          else if(numberActiveFileDescr == -1)
          {
              printf("An error has occured \n");
              return -1;
          }
      }

      return recvfrom(_udpSock, buffer, maxSize, 0, (struct sockaddr *)&_controllerAddr, &sockAddrSize);
   }
   return -1;
}
   
//******************************************************************************
bool UdpConnection::send(const char* buffer, int size)
{
   if ((isOpen()) && (ntohs(_controllerAddr.sin_port) != 0))
   {
      int sent = sendto(_udpSock, const_cast<char*>(buffer), size, 0, (struct sockaddr *)&_controllerAddr, sizeof(_controllerAddr));
      if (sent == size)
      {
         return true;
      }
   }
   return false;
}
