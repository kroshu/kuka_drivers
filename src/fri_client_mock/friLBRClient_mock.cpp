/*
 * friLBRClient_mock.cpp
 *
 *  Created on: Feb 4, 2020
 *      Author: rosdeveloper
 */

#include "fri_client/friLBRClient.h"
#include "fri_client/friClientIf.h"

namespace KUKA
{
namespace FRI
{

LBRClient::LBRClient()
{

}

LBRClient::~LBRClient()
{

}

void LBRClient::onStateChange(ESessionState oldState, ESessionState newState)
{
  return;
}

void LBRClient::monitor()
{
  return;
}

void LBRClient::waitForCommand()
{
  return;
}

void LBRClient::command()
{
  return;
}

}
}
