/*
 * friLBRState_mock.cpp
 *
 *  Created on: Feb 4, 2020
 *      Author: rosdeveloper
 */

#include <fri_client/friClientIf.h>
#include <fri_client/friLBRState.h>

namespace KUKA
{
namespace FRI
{

struct _FRIMonitoringMessage{};

LBRState::LBRState():
    _message()
{

}

double LBRState::getSampleTime() const
{
  return 0.0;
}

ESessionState LBRState::getSessionState() const
{
  return IDLE;
}

EConnectionQuality LBRState::getConnectionQuality() const
{
  return POOR;
}

ESafetyState LBRState::getSafetyState() const
{
  return NORMAL_OPERATION;
}


EOperationMode LBRState::getOperationMode() const
{
  return TEST_MODE_1;
}

EDriveState LBRState::getDriveState() const
{
  return OFF;
}


EClientCommandMode LBRState::getClientCommandMode() const
{
  return NO_COMMAND_MODE;
}

EOverlayType LBRState::getOverlayType() const
{
  return NO_OVERLAY;
}

EControlMode LBRState::getControlMode() const
{
  return NO_CONTROL;
}

unsigned int LBRState::getTimestampSec() const
{
  return 0;
}

unsigned int LBRState::getTimestampNanoSec() const
{
  return 0;
}

const double* LBRState::getMeasuredJointPosition() const
{
  return nullptr;
}

const double* LBRState::getCommandedJointPosition() const
{
  return nullptr;
}

const double* LBRState::getMeasuredTorque() const
{
  return nullptr;
}

const double* LBRState::getCommandedTorque() const
{
  return nullptr;
}

const double* LBRState::getExternalTorque() const
{
  return nullptr;
}

const double* LBRState::getIpoJointPosition() const
{
  return nullptr;
}

double LBRState::getTrackingPerformance() const
{
  return 0.0;
}

bool LBRState::getBooleanIOValue(const char* name) const
{
  (void)name;
  return false;
}
unsigned long long LBRState::getDigitalIOValue(const char* name) const
{
  (void)name;
  return 0;
}

double LBRState::getAnalogIOValue(const char* name) const
{
  (void)name;
  return 0.0;
}

}
}
