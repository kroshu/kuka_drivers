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

#include <fri/friTransformationClient.h>
#include <friClientData.h>

#include <FRIMessages.pb.h>
#include <pb_frimessages_callbacks.h>

using namespace KUKA::FRI;

//******************************************************************************
TransformationClient::TransformationClient()
{
}
      
//******************************************************************************
TransformationClient::~TransformationClient()
{
}

//******************************************************************************
const std::vector<const char*>& TransformationClient::getRequestedTransformationIDs() const
{
   unsigned int trafoCount = _data->monitoringMsg.requestedTransformations_count;
   _data->requestedTrafoIDs.resize(trafoCount);
   for (unsigned int i=0; i<trafoCount; i++)
   {
      _data->requestedTrafoIDs[i] = _data->monitoringMsg.requestedTransformations[i].name;
   }   
	return _data->requestedTrafoIDs;
}

//******************************************************************************
const unsigned int TransformationClient::getTimestampSec() const 
{
	return _data->monitoringMsg.monitorData.timestamp.sec;
}

//******************************************************************************
const unsigned int TransformationClient::getTimestampNanoSec() const
{
	return _data->monitoringMsg.monitorData.timestamp.nanosec;
}

//******************************************************************************
void TransformationClient::setTransformation(const char* transformationID, 
      const double transformationMatrix[3][4], unsigned int timeSec, unsigned int timeNanoSec)
{
	_data->commandMsg.has_commandData = true;
	
	unsigned int currentSize = _data->commandMsg.commandData.commandedTransformations_count;
	
   if (currentSize < _data->MAX_REQUESTED_TRANSFORMATIONS)
	{
	   _data->commandMsg.commandData.commandedTransformations_count++;
	   Transformation& dest = _data->commandMsg.commandData.commandedTransformations[currentSize];
      strncpy(dest.name, transformationID, _data->MAX_SIZE_TRANSFORMATION_ID);
      dest.name[_data->MAX_SIZE_TRANSFORMATION_ID - 1] = '\0';
      dest.matrix_count = 12;
	   memcpy(dest.matrix, transformationMatrix, 12 * sizeof(double));
	   dest.has_timestamp = true;
	   dest.timestamp.sec = timeSec;
	   dest.timestamp.nanosec = timeNanoSec;
	}
   else
   {
      throw FRIException("Exceeded maximum number of transformations.");
   }
}

//******************************************************************************
void TransformationClient::linkData(ClientData* clientData)
{
	_data = clientData;
}

//******************************************************************************
double TransformationClient::getSampleTime() const
{
   return _data->monitoringMsg.connectionInfo.sendPeriod * 0.001;
}

//******************************************************************************
EConnectionQuality TransformationClient::getConnectionQuality() const
{
   return (EConnectionQuality)_data->monitoringMsg.connectionInfo.quality;
}


//******************************************************************************
void TransformationClient::setBooleanIOValue(const char* name, const bool value)
{
   ClientData::setBooleanIOValue(&_data->commandMsg, name, value, &_data->monitoringMsg);
}

//******************************************************************************
void TransformationClient::setAnalogIOValue(const char* name, const double value)
{
   ClientData::setAnalogIOValue(&_data->commandMsg, name, value, &_data->monitoringMsg);
}

//******************************************************************************
void TransformationClient::setDigitalIOValue(const char* name, const unsigned long long value)
{
   ClientData::setDigitalIOValue(&_data->commandMsg, name, value, &_data->monitoringMsg);
}

//******************************************************************************
bool TransformationClient::getBooleanIOValue(const char* name) const
{
   return ClientData::getBooleanIOValue(&_data->monitoringMsg, name).digitalValue != 0;
}

//******************************************************************************
unsigned long long TransformationClient::getDigitalIOValue(const char* name) const
{
   return ClientData::getDigitalIOValue(&_data->monitoringMsg, name).digitalValue;
}

//******************************************************************************
double TransformationClient::getAnalogIOValue(const char* name) const
{
   return ClientData::getAnalogIOValue(&_data->monitoringMsg, name).analogValue;
}

//******************************************************************************
/*const std::vector<const char*>& TransformationClient::getRequestedIO_IDs() const
{
   return _data->getRequestedIO_IDs();
}*/
