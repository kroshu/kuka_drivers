/**

The following license terms and conditions apply, unless a redistribution
agreement or other license is obtained by KUKA Roboter GmbH, Augsburg, Germany.

SCOPE

The software �KUKA Sunrise.Connectivity FRI Client SDK� is targeted to work in
conjunction with the �KUKA Sunrise.Connectivity FastRobotInterface� toolkit.
In the following, the term �software� refers to all material directly
belonging to the provided SDK �Software development kit�, particularly source
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
#ifndef _KUKA_FRI_CONNECTION_H
#define _KUKA_FRI_CONNECTION_H


/** Kuka namespace */
namespace KUKA
{
namespace FRI
{

/**
    * \brief FRI connection interface.
    *
    * Connections to the KUKA Sunrise controller have to be implemented using
    * this interface.
    */
class IConnection
{

public:
  /** \brief Virtual destructor. */
  virtual ~IConnection() {}

  /**
     * \brief Open a connection to the KUKA Sunrise controller.
     *
     * @param port The port ID
     * @param remoteHost The address of the remote host
     * @return True if connection was established
     */
  virtual bool open(int port, const char * remoteHost) = 0;

  /**
     * \brief Close a connection to the KUKA Sunrise controller.
     */
  virtual void close() = 0;

  /**
     * \brief Checks whether a connection to the KUKA Sunrise controller is established.
     *
     * @return True if connection is established
     */
  virtual bool isOpen() const = 0;

  /**
     * \brief Receive a new FRI monitoring message from the KUKA Sunrise controller.
     *
     * This method blocks until a new message arrives.
     * @param buffer Pointer to the allocated buffer that will hold the FRI message
     * @param maxSize Size in bytes of the allocated buffer
     * @return Number of bytes received (0 when connection was terminated, negative in case of errors)
     */
  virtual int receive(char * buffer, int maxSize) = 0;

  /**
     * \brief Send a new FRI command message to the KUKA Sunrise controller.
     *
     * @param buffer Pointer to the buffer holding the FRI message
     * @param size Size in bytes of the message to be send
     * @return True if successful
     */
  virtual bool send(const char * buffer, int size) = 0;

};

}
}


#endif // _KUKA_FRI_CONNECTION_H
