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

#include <chrono>
#include <memory>
#include <thread>
#include <vector>

#include "rclcpp/logging.hpp"

#include <iostream>

#include "kuka_sunrise_fri_driver/fri_connection.hpp"
#include "kuka_sunrise_fri_driver/serialization.hpp"
#include "kuka_sunrise_fri_driver/tcp_connection.hpp"

namespace kuka_sunrise_fri_driver
{

FRIConnection::FRIConnection(
  std::function<void(void)> handle_control_ended_error_callback,
  std::function<void(void)> handle_fri_ended_callback)
: handleControlEndedError_(handle_control_ended_error_callback),
  handleFRIEndedError_(handle_fri_ended_callback),
  last_command_state_(ACCEPTED),
  last_command_id_(CONNECT),
  last_command_success_(NO_SUCCESS),
  answer_wanted_(false),
  answer_received_(false)
{
}

FRIConnection::~FRIConnection() { disconnect(); }

bool FRIConnection::connect(const char * server_addr, int server_port)
{
  // TODO(resizoltan) check if already connected
  try
  {
    tcp_connection_ = std::make_unique<TCPConnection>(
      server_addr, server_port,
      [this](std::vector<std::uint8_t> data) { this->handleReceivedTCPData(data); },
      [this](const char * server_addr, int server_port)
      { this->connectionLostCallback(server_addr, server_port); });
  }
  catch (...)
  {
    tcp_connection_.reset();
  }

  if (!isConnected())
  {
    return false;
  }
  return sendCommandAndWait(CONNECT);
}

bool FRIConnection::disconnect()
{
  if (tcp_connection_ == nullptr)
  {
    return true;
  }
  if (sendCommandAndWait(DISCONNECT) == true)
  {
    tcp_connection_->closeConnection();
    tcp_connection_.reset();
    return true;
  }
  else
  {
    return false;
  }
}

bool FRIConnection::startFRI() { return sendCommandAndWait(START_FRI); }

bool FRIConnection::endFRI() { return sendCommandAndWait(END_FRI); }

bool FRIConnection::activateControl() { return sendCommandAndWait(ACTIVATE_CONTROL); }

bool FRIConnection::deactivateControl() { return sendCommandAndWait(DEACTIVATE_CONTROL); }

bool FRIConnection::setPositionControlMode()
{
  std::vector<std::uint8_t> command_data = {POSITION_CONTROL_MODE};
  return sendCommandAndWait(SET_CONTROL_MODE, command_data);
}

bool FRIConnection::setJointImpedanceControlMode(
  const std::vector<double> & joint_stiffness, const std::vector<double> & joint_damping)
{
  std::vector<std::uint8_t> serialized;
  serialized.reserve(1 + CONTROL_MODE_HEADER.size() + 2 * 7 * sizeof(double));
  serialized.emplace_back(JOINT_IMPEDANCE_CONTROL_MODE);
  for (std::uint8_t byte : CONTROL_MODE_HEADER)
  {
    serialized.emplace_back(byte);
  }
  for (double js : joint_stiffness)
  {
    serializeNext(js, serialized);
  }
  for (double jd : joint_damping)
  {
    serializeNext(jd, serialized);
  }
  return sendCommandAndWait(SET_CONTROL_MODE, serialized);
}

bool FRIConnection::setClientCommandMode(ClientCommandModeID client_command_mode)
{
  std::vector<std::uint8_t> command_data = {client_command_mode};
  return sendCommandAndWait(SET_COMMAND_MODE, command_data);
}

bool FRIConnection::setFRIConfig(
  const std::string & client_ip, int remote_port, int send_period_ms, int receive_multiplier)
{
  std::vector<std::uint8_t> serialized;
  serialized.reserve(FRI_CONFIG_HEADER.size() + 4 * sizeof(int));
  for (std::uint8_t byte : FRI_CONFIG_HEADER)
  {
    serialized.emplace_back(byte);
  }
  serializeNext(remote_port, serialized);
  serializeNext(send_period_ms, serialized);
  serializeNext(receive_multiplier, serialized);

  int ip = inet_addr(client_ip.c_str());
  serializeNext(ip, serialized);

  return sendCommandAndWait(SET_FRI_CONFIG, serialized);
}

bool FRIConnection::isConnected()
{
  if (tcp_connection_)
  {
    return true;
  }
  else
  {
    return false;
  }
}

bool FRIConnection::assertLastCommandSuccess(CommandID command_id)
{
  // TODO(resizoltan) more sophisticated introspection
  if (
    last_command_state_ == ACCEPTED && last_command_id_ == command_id &&
    last_command_success_ == SUCCESS)
  {
    return true;
  }
  else
  {
    return false;
  }
}

bool FRIConnection::sendCommandAndWait(CommandID command_id)
{
  answer_wanted_ = true;
  tcp_connection_->sendByte(command_id);
  std::unique_lock<std::mutex> lk(m_);
  cv_.wait(lk, [this] { return answer_received_; });
  answer_received_ = false;
  answer_wanted_ = false;
  return assertLastCommandSuccess(command_id);
}

bool FRIConnection::sendCommandAndWait(
  CommandID command_id, const std::vector<std::uint8_t> & command_data)
{
  std::vector<std::uint8_t> msg;
  msg.push_back(command_id);
  msg.insert(msg.end(), command_data.begin(), command_data.end());
  answer_wanted_ = true;
  tcp_connection_->sendBytes(msg);
  std::unique_lock<std::mutex> lk(m_);
  cv_.wait(lk, [this] { return answer_received_; });
  answer_received_ = false;
  answer_wanted_ = false;
  return assertLastCommandSuccess(command_id);
}

void FRIConnection::handleReceivedTCPData(const std::vector<std::uint8_t> & data)
{
  if (data.size() == 0)
  {
    return;
  }
  std::thread handler_thread;
  std::lock_guard<std::mutex> lk(m_);
  // TODO(resizoltan) handle invalid data
  switch ((CommandState)data[0])
  {
    case ACCEPTED:
      if (data.size() < 3)
      {
        // TODO(resizoltan) error
      }
      last_command_state_ = ACCEPTED;
      last_command_id_ = (CommandID)data[1];
      last_command_success_ = (CommandSuccess)data[2];
      answer_received_ = true;
      cv_.notify_one();
      break;
    case REJECTED:
      if (data.size() < 2)
      {
        // TODO(resizoltan) error
      }
      last_command_state_ = REJECTED;
      last_command_id_ = (CommandID)data[1];
      answer_received_ = true;
      cv_.notify_one();
      break;
    case UNKNOWN:
      last_command_state_ = UNKNOWN;
      answer_received_ = true;
      cv_.notify_one();
      break;
    case ERROR_CONTROL_ENDED:
      if (answer_wanted_)
      {
        last_command_state_ = ERROR_CONTROL_ENDED;
        answer_received_ = true;
        cv_.notify_one();
      }
      else
      {
        handler_thread = std::thread([this] { this->handleControlEndedError_(); });
        handler_thread.detach();  // Detach the thread
      }
      break;
    case ERROR_FRI_ENDED:
      if (answer_wanted_)
      {
        last_command_state_ = ERROR_FRI_ENDED;
        answer_received_ = true;
        cv_.notify_one();
      }
      else
      {
        handler_thread = std::thread([this] { this->handleFRIEndedError_(); });
        handler_thread.detach();  // Detach the thread
      }
      break;
    default:
      last_command_state_ = UNKNOWN;
      answer_received_ = true;
      cv_.notify_one();
      break;
  }
}

void FRIConnection::connectionLostCallback(const char * server_addr, int server_port)
{
  RCLCPP_ERROR(rclcpp::get_logger("fri_connection"), "Connection lost, trying to reconnect");
  connect(server_addr, server_port);
}

}  // namespace kuka_sunrise_fri_driver
