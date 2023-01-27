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
#include <fri/friClientApplication.h>
#include <fri/friClientIf.h>
#include <fri/friConnectionIf.h>
#include <friClientData.h>
#include <FRIMessages.pb.h>
#include <fri/friTransformationClient.h>

using namespace KUKA::FRI;

//******************************************************************************
ClientApplication::ClientApplication(IConnection& connection, IClient& client)
   : _connection(connection), _robotClient(&client),_trafoClient(NULL), _data(NULL)
{
   _data = _robotClient->createData();
}

//******************************************************************************
ClientApplication::ClientApplication(IConnection& connection, IClient& client, TransformationClient& trafoClient)
   : _connection(connection), _robotClient(&client),_trafoClient(&trafoClient), _data(NULL)
{
   _data = _robotClient->createData();
   _trafoClient->linkData(_data);
}

//******************************************************************************
ClientApplication::~ClientApplication()
{
   disconnect();
   delete _data;
}

//******************************************************************************
bool ClientApplication::connect(int port, const char *remoteHost)
{
   if (_connection.isOpen()) 
   {
      printf("Warning: client application already connected!\n");
      return true;
   }
   
   return _connection.open(port, remoteHost);
}
   
//******************************************************************************
void ClientApplication::disconnect()
{
   if (_connection.isOpen()) _connection.close();
}

//******************************************************************************
bool ClientApplication::step()
{
   if (!_connection.isOpen())
   {
      printf("Error: client application is not connected!\n");
      return false;
   }

   // **************************************************************************
   // Receive and decode new monitoring message
   // **************************************************************************
   int size = _connection.receive(_data->receiveBuffer, FRI_MONITOR_MSG_MAX_SIZE);
   
   if (size <= 0)
   {  // TODO: size == 0 -> connection closed (maybe go to IDLE instead of stopping?)
      printf("Error: failed while trying to receive monitoring message!\n");
      return false;
   }
   
   if (!_data->decoder.decode(_data->receiveBuffer, size))
   {
      return false;
   }
   
   // check message type (so that our wrappers match)
   if (_data->expectedMonitorMsgID != _data->monitoringMsg.header.messageIdentifier)
   {
      printf("Error: incompatible IDs for received message (got: %d expected %d)!\n",
            (int)_data->monitoringMsg.header.messageIdentifier,
            (int)_data->expectedMonitorMsgID);
      return false;
   }   
   
   // **************************************************************************
   // callbacks
   // **************************************************************************
   // reset commmand message before callbacks
   _data->resetCommandMessage();
   
   // callbacks for robot client
   ESessionState currentState = (ESessionState)_data->monitoringMsg.connectionInfo.sessionState;
   
   if (_data->lastState != currentState)
   {
      _robotClient->onStateChange(_data->lastState, currentState);
      _data->lastState = currentState;
   }
   
   switch (currentState)
   {
      case MONITORING_WAIT:
      case MONITORING_READY:
         _robotClient->monitor();
         break;
      case COMMANDING_WAIT:
         _robotClient->waitForCommand();
         break;
      case COMMANDING_ACTIVE:
         _robotClient->command();
         break;
      case IDLE:
      default:
         return true; // nothing to send back
   }

   // callback for transformation client
   if(_trafoClient != NULL)
   {
      _trafoClient->provide();
   }
   
   // **************************************************************************
   // Encode and send command message
   // **************************************************************************
   
   _data->lastSendCounter++;
   // check if its time to send an answer
   if (_data->lastSendCounter >= _data->monitoringMsg.connectionInfo.receiveMultiplier)
   {
      _data->lastSendCounter = 0;
      
      // set sequence counters
      _data->commandMsg.header.sequenceCounter = _data->sequenceCounter++;
      _data->commandMsg.header.reflectedSequenceCounter = 
            _data->monitoringMsg.header.sequenceCounter;
      
      if (!_data->encoder.encode(_data->sendBuffer, size))
      {
         return false;
      }
      
      if (!_connection.send(_data->sendBuffer, size))
      {
         printf("Error: failed while trying to send command message!\n");
         return false;
      }
   }
   
   return true;
}

