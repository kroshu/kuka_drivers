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
#ifndef _KUKA_FRI_EXCEPTION_H
#define _KUKA_FRI_EXCEPTION_H

#include <stdio.h>

/** Kuka namespace */
namespace KUKA
{
namespace FRI
{

/**
    * \brief Standard exception for the FRI Client
    *
    * \note For realtime considerations the internal message buffer is static.
    * So don't use this exception in more than one thread per process.
    */
class FRIException
{

public:
  /**
     * \brief FRIException Constructor
     *
     * @param message Error message
     */
  FRIException(const char * message)
  {
    strncpy(_buffer, message, sizeof(_buffer) - 1);
    _buffer[sizeof(_buffer) - 1] = 0;      // ensure string termination
    printf("FRIException: ");
    printf("%s", _buffer);
    printf("\n");
  }

  /**
     * \brief FRIException Constructor
     *
     * @param message Error message which may contain one "%s" parameter
     * @param param1 First format parameter for parameter message.
     */
  FRIException(const char * message, const char * param1)
  {
#ifdef _MSC_VER
    _snprintf(      // visual studio compilers (up to VS 2013) only know this method
#else
    snprintf(
#endif
      _buffer, sizeof(_buffer), message, param1);
    printf("FRIException: ");
    printf("%s", _buffer);
    printf("\n");
  }

  /**
     * \brief FRIException Constructor
     *
     * @param message Error message which may contain two "%s" parameter
     * @param param1 First format parameter for parameter message.
     * @param param2 Second format parameter for parameter message.
     */
  FRIException(const char * message, const char * param1, const char * param2)
  {
#ifdef _MSC_VER
    _snprintf(      // visual studio compilers (up to VS 2013) only know this method
#else
    snprintf(
#endif
      _buffer, sizeof(_buffer), message, param1, param2);
    printf("FRIException: ");
    printf("%s", _buffer);
    printf("\n");
  }

  /**
     * \brief Get error string.
     * @return Error message stored in the exception.
     */
  const char * getErrorMessage() const {return _buffer;}

  /** \brief Virtual destructor. */
  virtual ~FRIException() {}

protected:
  static char _buffer[1024];

};

}
}


#endif // _KUKA_FRI_EXCEPTION_H
