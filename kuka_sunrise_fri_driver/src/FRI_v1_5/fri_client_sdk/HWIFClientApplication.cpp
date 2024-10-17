#include <fri_client_sdk/HWIFClientApplication.hpp>
#include <iostream>


namespace KUKA
{
namespace FRI
{

HWIFClientApplication::HWIFClientApplication(IConnection & connection, IClient & client)
: ClientApplication(connection, client) {}

bool HWIFClientApplication::client_app_read()
{
  if (!_connection.isOpen()) {
    std::cout << "Error: client application is not connected!" << std::endl;
    return false;
  }

  // **************************************************************************
  // Receive and decode new monitoring message
  // **************************************************************************
  size_ = _connection.receive(_data->receiveBuffer, FRI_MONITOR_MSG_MAX_SIZE);

  if (size_ <= 0) { // TODO: size_ == 0 -> connection closed (maybe go to IDLE instead of stopping?)
    std::cout << "Error: failed while trying to receive monitoring message!" << std::endl;
    return false;
  }

  if (!_data->decoder.decode(_data->receiveBuffer, size_)) {
    std::cout << "Error: failed to decode message" << std::endl;
    return false;
  }

  // check message type (so that our wrappers match)
  if (_data->expectedMonitorMsgID != _data->monitoringMsg.header.messageIdentifier) {
    std::cout << "Error: incompatible IDs for received message, got: " <<
      _data->monitoringMsg.header.messageIdentifier << " expected: " <<
      _data->expectedMonitorMsgID << std::endl;
    return false;
  }

  return true;
}

void HWIFClientApplication::client_app_update()
{
  // **************************************************************************
  // callbacks
  // **************************************************************************
  // reset command message before callbacks
  _data->resetCommandMessage();

  // callbacks for robot client
  ESessionState currentState = (ESessionState)_data->monitoringMsg.connectionInfo.sessionState;

  if (_data->lastState != currentState) {
    _robotClient->onStateChange(_data->lastState, currentState);
    _data->lastState = currentState;
  }

  switch (currentState) {
    case MONITORING_WAIT:
    case MONITORING_READY:
      _robotClient->monitor();
      break;
    case COMMANDING_WAIT:
      _robotClient->waitForCommand();
      break;
    case COMMANDING_ACTIVE:
      _robotClient->command();
      break;
    case IDLE:
    default:
      return;    // nothing to send back
  }

  // callback for transformation client
  if (_trafoClient != NULL) {
    _trafoClient->provide();
  }
}


bool HWIFClientApplication::client_app_write()
{
  // **************************************************************************
  // Encode and send command message
  // **************************************************************************

  _data->lastSendCounter++;
  // check if its time to send an answer
  if (_data->lastSendCounter >= _data->monitoringMsg.connectionInfo.receiveMultiplier) {
    _data->lastSendCounter = 0;

    // set sequence counters
    _data->commandMsg.header.sequenceCounter = _data->sequenceCounter++;
    _data->commandMsg.header.reflectedSequenceCounter =
      _data->monitoringMsg.header.sequenceCounter;

    if (!_data->encoder.encode(_data->sendBuffer, size_)) {
      return false;
    }

    if (!_connection.isOpen()) {
      std::cout << "Client application connection closed" << std::endl;
      return false;
    }

    if (!_connection.send(_data->sendBuffer, size_)) {
      std::cout << "Error: failed while trying to send command message!" << std::endl;
      return false;
    }
  }

  return true;
}
}
}  // namespace KUKA::FRI
