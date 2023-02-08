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
#include <fri/friLBRState.h>
#include <friClientData.h>
#include <pb_frimessages_callbacks.h>

using namespace KUKA::FRI;


LBRState::LBRState():_message(0)
{
    
}
//******************************************************************************
double LBRState::getSampleTime() const
{
   return _message->connectionInfo.sendPeriod * 0.001;
}

//******************************************************************************
ESessionState LBRState::getSessionState() const
{
   return (ESessionState)_message->connectionInfo.sessionState;
}

//******************************************************************************
EConnectionQuality LBRState::getConnectionQuality() const
{
   return (EConnectionQuality)_message->connectionInfo.quality;
}

//******************************************************************************
ESafetyState LBRState::getSafetyState() const
{
   return (ESafetyState)_message->robotInfo.safetyState;
}

//******************************************************************************
EOperationMode LBRState::getOperationMode() const
{
   return (EOperationMode)_message->robotInfo.operationMode;
}

//******************************************************************************
EDriveState LBRState::getDriveState() const
{
   tRepeatedIntArguments *values =
         (tRepeatedIntArguments *)_message->robotInfo.driveState.arg;
   int firstState = (int)values->value[0];
   for (int i=1; i<NUMBER_OF_JOINTS; i++)
   {
      int state = (int)values->value[i];
      if (state != firstState)
      {
         return TRANSITIONING;
      }
   }
   return (EDriveState)firstState;
}


//********************************************************************************
EOverlayType LBRState::getOverlayType() const
{
   return (EOverlayType)_message->ipoData.overlayType;
}

//********************************************************************************
EClientCommandMode LBRState::getClientCommandMode() const
{
   return (EClientCommandMode)_message->ipoData.clientCommandMode;
}


//******************************************************************************
EControlMode LBRState::getControlMode() const
{
   return (EControlMode)_message->robotInfo.controlMode;
}

//******************************************************************************
unsigned int LBRState::getTimestampSec() const
{
   return _message->monitorData.timestamp.sec;
}

//******************************************************************************
unsigned int LBRState::getTimestampNanoSec() const
{
   return _message->monitorData.timestamp.nanosec;
}

//******************************************************************************
const double* LBRState::getMeasuredJointPosition() const
{
   tRepeatedDoubleArguments *values =
         (tRepeatedDoubleArguments*)_message->monitorData.measuredJointPosition.value.arg;
   return (double*)values->value;
}

//******************************************************************************
const double* LBRState::getCommandedJointPosition() const
{
   tRepeatedDoubleArguments *values =
            (tRepeatedDoubleArguments*)_message->monitorData.commandedJointPosition.value.arg;
   return (double*)values->value;
}

//******************************************************************************
const double* LBRState::getMeasuredTorque() const
{
   tRepeatedDoubleArguments *values =
            (tRepeatedDoubleArguments*)_message->monitorData.measuredTorque.value.arg;
   return (double*)values->value;
}

//******************************************************************************
const double* LBRState::getCommandedTorque() const
{
   tRepeatedDoubleArguments *values =
            (tRepeatedDoubleArguments*)_message->monitorData.commandedTorque.value.arg;
   return (double*)values->value;
}

//******************************************************************************
const double* LBRState::getExternalTorque() const
{
   tRepeatedDoubleArguments *values =
            (tRepeatedDoubleArguments*)_message->monitorData.externalTorque.value.arg;
   return (double*)values->value;
}

//******************************************************************************
const double* LBRState::getIpoJointPosition() const
{
   if (!_message->ipoData.has_jointPosition)
   {
      throw FRIException("IPO joint position not available in monitoring mode.");
      return NULL;
   }
   
   tRepeatedDoubleArguments *values =
            (tRepeatedDoubleArguments*)_message->ipoData.jointPosition.value.arg;
   return (double*)values->value;
}

//******************************************************************************
double LBRState::getTrackingPerformance() const
{
   if (!_message->ipoData.has_trackingPerformance) return 0.0;
   
   return _message->ipoData.trackingPerformance;
}

//******************************************************************************
bool LBRState::getBooleanIOValue(const char* name) const
{
   return ClientData::getBooleanIOValue(_message, name).digitalValue != 0;
}

//******************************************************************************
unsigned long long LBRState::getDigitalIOValue(const char* name) const
{
   return ClientData::getDigitalIOValue(_message, name).digitalValue;
}

//******************************************************************************
double LBRState::getAnalogIOValue(const char* name) const
{
   return ClientData::getAnalogIOValue(_message, name).analogValue;
}

//******************************************************************************
/*const std::vector<const char*>& LBRState::getRequestedIO_IDs() const
{
   return _clientData->getRequestedIO_IDs();
}*/
