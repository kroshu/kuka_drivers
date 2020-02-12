// Copyright 2020 Zoltán Rési
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "fri_client/friClientIf.h"
#include "fri_client/friLBRClient.h"

namespace KUKA
{
namespace FRI
{

struct ClientData
{
};

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
}

void LBRClient::monitor()
{
}

void LBRClient::waitForCommand()
{
}

void LBRClient::command()
{
}

ClientData * LBRClient::createData()
{
  return nullptr;
}

}  // namespace FRI

}  // namespace KUKA
