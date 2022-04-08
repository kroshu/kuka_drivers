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

#include "fri_client/friClientApplication.h"

namespace KUKA
{
namespace FRI
{

class IClient
{
};
class TransformationClient
{
};
class IConnection
{
};
struct ClientData
{
};

ClientApplication::ClientApplication(IConnection & connection, IClient & client)
: _connection(connection), _robotClient(), _trafoClient(), _data()
{
  (void)client;
}

ClientApplication::ClientApplication(
  IConnection & connection, IClient & client,
  TransformationClient & trafoClient)
: _connection(connection), _robotClient(), _trafoClient(), _data()
{
  (void)client;
  (void)trafoClient;
}

ClientApplication::~ClientApplication()
{
}

bool ClientApplication::connect(int port, const char * remoteHost)
{
  (void)port;
  (void)remoteHost;
  return false;
}

void ClientApplication::disconnect()
{
  // only a mock
}

bool ClientApplication::step()
{
  return false;
}

}  // namespace FRI

}  // namespace KUKA
