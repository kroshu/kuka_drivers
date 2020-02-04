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
  const unsigned int i = 0;
  return i;
}

const unsigned int TransformationClient::getTimestampNanoSec() const
{
  const unsigned int i = 0;
  return i;
}

void TransformationClient::setTransformation(const char* transformationID, const double transformationMatrix[3][4],
            unsigned int timeSec, unsigned int timeNanoSec)
{
  (void)transformationID;
  (void)transformationMatrix;
  (void)timeSec;
  (void)timeNanoSec;
  return;
}

void TransformationClient::setBooleanIOValue(const char* name, const bool value)
{
  (void)name;
  (void)value;
  return;
}

void TransformationClient::setDigitalIOValue(const char* name, const unsigned long long value)
{
  (void)name;
  (void)value;
  return;
}

void TransformationClient::setAnalogIOValue(const char* name, const double value)
{
  (void)name;
  (void)value;
  return;
}

bool TransformationClient::getBooleanIOValue(const char* name) const
{
  (void)name;
  return false;
}

unsigned long long TransformationClient::getDigitalIOValue(const char* name) const
{
  (void)name;
  return 0;
}

double TransformationClient::getAnalogIOValue(const char* name) const
{
  (void)name;
  return 0.0;
}

}
}

