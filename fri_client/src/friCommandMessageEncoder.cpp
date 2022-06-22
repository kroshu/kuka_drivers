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
#include <cstdio>
#include <friCommandMessageEncoder.h>
#include <nanopb/pb_encode.h>

using namespace KUKA::FRI;

//******************************************************************************
CommandMessageEncoder::CommandMessageEncoder(FRICommandMessage* pMessage, int num)
   : m_nNum(num), m_pMessage(pMessage)
{
   initMessage();
}

//******************************************************************************
CommandMessageEncoder::~CommandMessageEncoder()
{

}

//******************************************************************************
void CommandMessageEncoder::initMessage()
{
   m_pMessage->has_commandData = false;
   m_pMessage->has_endOfMessageData = false;
   m_pMessage->commandData.has_jointPosition = false;
   m_pMessage->commandData.has_cartesianWrenchFeedForward = false;
   m_pMessage->commandData.has_jointTorque = false;
   m_pMessage->commandData.commandedTransformations_count = 0;
   m_pMessage->header.messageIdentifier = 0;
   // init with 0. Necessary for creating the correct reflected sequence count in the monitoring msg
   m_pMessage->header.sequenceCounter = 0;
   m_pMessage->header.reflectedSequenceCounter = 0;

   m_pMessage->commandData.writeIORequest_count = 0;

   // allocate and map memory for protobuf repeated structures
   map_repeatedDouble(FRI_MANAGER_NANOPB_ENCODE, m_nNum, 
         &m_pMessage->commandData.jointPosition.value,
         &m_tRecvContainer.jointPosition);
   map_repeatedDouble(FRI_MANAGER_NANOPB_ENCODE, m_nNum, 
         &m_pMessage->commandData.jointTorque.value,
         &m_tRecvContainer.jointTorque);
   
   // nanopb encoding needs to know how many elements the static array contains
   // a Cartesian wrench feed forward vector always contains 6 elements
   m_pMessage->commandData.cartesianWrenchFeedForward.element_count = 6;
}

//******************************************************************************
bool CommandMessageEncoder::encode(char* buffer, int& size)
{
    // generate stream for encoding
    pb_ostream_t stream = pb_ostream_from_buffer((uint8_t*)buffer, FRI_COMMAND_MSG_MAX_SIZE);
    // encode monitoring Message to stream
    bool status = pb_encode(&stream, FRICommandMessage_fields, m_pMessage);
    size = stream.bytes_written;
    if (!status)
    {
        printf("!!encoding error: %s!!\n", PB_GET_ERROR(&stream));
    }
    return status;
}
