/*
 * friLBRCommand_mock.cpp
 *
 *  Created on: Feb 4, 2020
 *      Author: rosdeveloper
 */

#include <fri_client/friLBRCommand.h>

namespace KUKA
{
namespace FRI
{

void LBRCommand::setJointPosition(const double* values)
{
  (void)values;
  return;
}

void LBRCommand::setWrench(const double* wrench)
{
  (void)wrench;
  return;
}

void LBRCommand::setTorque(const double* torques)
{
  (void)torques;
  return;
}

void LBRCommand::setBooleanIOValue(const char* name, const bool value)
{
  (void)name;
  (void)value;
  return;
}

void LBRCommand::setDigitalIOValue(const char* name, const unsigned long long value)
{
  (void)name;
  (void)value;
  return;
}

void LBRCommand::setAnalogIOValue(const char* name, const double value)
{
  (void)name;
  (void)value;
  return;
}

}
}
