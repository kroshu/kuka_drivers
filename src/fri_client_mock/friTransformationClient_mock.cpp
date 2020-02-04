/*
 * friTransformationClient_mock.cpp
 *
 *  Created on: Feb 4, 2020
 *      Author: rosdeveloper
 */

#include "fri_client/friTransformationClient.h"
#include "fri_client/friClientIf.h"

namespace KUKA
{
namespace FRI
{

struct ClientData
{

};

TransformationClient::TransformationClient():
    _data()
{

}

double TransformationClient::getSampleTime() const
{
  return 0.0;
}

EConnectionQuality TransformationClient::getConnectionQuality() const
{
  return POOR;
}

const std::vector<const char*>& TransformationClient::getRequestedTransformationIDs() const
{
  return std::vector<const char*>();
}

const unsigned int TransformationClient::getTimestampSec() const
{
  return 0;
}

const unsigned int TransformationClient::getTimestampNanoSec() const
{
  return 0;
}

void TransformationClient::setTransformation(const char* transformationID, const double transformationMatrix[3][4],
            unsigned int timeSec, unsigned int timeNanoSec)
{
  return;
}

void TransformationClient::setBooleanIOValue(const char* name, const bool value)
{
  return;
}

void TransformationClient::setDigitalIOValue(const char* name, const unsigned long long value)
{
  return;
}

void TransformationClient::setAnalogIOValue(const char* name, const double value)
{
  return;
}

bool TransformationClient::getBooleanIOValue(const char* name) const
{
  return false;
}

unsigned long long TransformationClient::getDigitalIOValue(const char* name) const
{
  return 0;
}

double TransformationClient::getAnalogIOValue(const char* name) const
{
  return 0.0;
}

}
}

