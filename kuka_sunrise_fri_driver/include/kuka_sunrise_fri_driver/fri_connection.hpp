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

#ifndef KUKA_SUNRISE_FRI_DRIVER__FRI_CONNECTION_HPP_
#define KUKA_SUNRISE_FRI_DRIVER__FRI_CONNECTION_HPP_

#include <condition_variable>
#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace kuka_sunrise_fri_driver
{
// forward declaration
class TCPConnection;

enum CommandState : std::uint8_t
{
  ACCEPTED = 1,
  REJECTED = 2,
  UNKNOWN = 3,
  ERROR_CONTROL_ENDED = 4,
  ERROR_FRI_ENDED = 5
};

enum CommandID : std::uint8_t
{
  CONNECT = 1,
  DISCONNECT = 2,
  START_FRI = 3,
  END_FRI = 4,
  ACTIVATE_CONTROL = 5,
  DEACTIVATE_CONTROL = 6,
  GET_FRI_CONFIG = 7,
  SET_FRI_CONFIG = 8,
  GET_CONTROL_MODE = 9,
  SET_CONTROL_MODE = 10,
  GET_COMMAND_MODE = 11,
  SET_COMMAND_MODE = 12
};

enum CommandSuccess : std::uint8_t
{
  SUCCESS = 1,
  NO_SUCCESS = 2
};

enum ControlModeID : std::uint8_t
{
  POSITION_CONTROL_MODE = 1,
  JOINT_IMPEDANCE_CONTROL_MODE = 2
};

enum ClientCommandModeID : std::uint8_t
{
  POSITION_COMMAND_MODE = 1,
  TORQUE_COMMAND_MODE = 3
};

static const std::vector<std::uint8_t> FRI_CONFIG_HEADER = {0xAC, 0xED, 0x00, 0x05, 0x77, 0x10};
static const std::vector<std::uint8_t> CONTROL_MODE_HEADER = {0xAC, 0xED, 0x00, 0x05, 0x77, 0x70};

class FRIConnection
{
public:
  FRIConnection(
    std::function<void(void)> handle_control_ended_error_callback,
    std::function<void(void)> handle_fri_ended_callback);
  ~FRIConnection();
  bool connect(const char * server_addr, int server_port);
  bool disconnect();
  bool startFRI();
  bool endFRI();
  bool activateControl();
  bool deactivateControl();
  bool setPositionControlMode();
  bool setJointImpedanceControlMode(
    const std::vector<double> & joint_stiffness, const std::vector<double> & joint_damping);
  bool setClientCommandMode(ClientCommandModeID client_command_mode);
  // bool getControlMode();
  bool setFRIConfig(
    const std::string & client_ip, int remote_port, int send_period_ms, int receive_multiplier);
  // bool getFRIConfig();

  bool isConnected();

private:
  std::unique_ptr<TCPConnection> tcp_connection_;

  std::function<void(void)> handleControlEndedError_;
  std::function<void(void)> handleFRIEndedError_;

  void handleReceivedTCPData(const std::vector<std::uint8_t> & data);
  void connectionLostCallback(const char * server_addr, int server_port);

  CommandState last_command_state_;
  CommandID last_command_id_;
  CommandSuccess last_command_success_;

  bool answer_wanted_;
  bool answer_received_;
  std::mutex m_;
  std::condition_variable cv_;

  // void wait();
  bool assertLastCommandSuccess(CommandID command_id);
  bool sendCommandAndWait(CommandID command_id);
  bool sendCommandAndWait(CommandID command_id, const std::vector<std::uint8_t> & command_data);
};

}  // namespace kuka_sunrise_fri_driver

#endif  // KUKA_SUNRISE_FRI_DRIVER__FRI_CONNECTION_HPP_
