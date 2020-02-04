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

struct ClientData{};

LBRClient::LBRClient()
{

}

LBRClient::~LBRClient()
{

}

void LBRClient::onStateChange(ESessionState oldState, ESessionState newState)
{
  (void)oldState;
  (void)newState;
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

ClientData* LBRClient::createData()
{
  return nullptr;
}


}
}
