/*
 * robot_manager.cpp
 *
 *  Created on: Nov 4, 2019
 *      Author: rosdeveloper
 */

#include "robot_manager.hpp"
#include "tcp_connection.hpp"

#include <thread>
#include <chrono>

namespace iiwa_interface{


bool RobotManager::connect(const char* server_addr, int server_port){
  //TODO: check if already connected
  tcp_connection_ = TCPConnection(server_addr, server_port,
                                  [](std::vector<char> data){this->handleReceivedTCPData(data);},
                                  [](){this->connectionLostCallback();});

  return sendCommandAndWait(CONNECT);
}

bool RobotManager::disconnect(){
  if(sendCommandAndWait(DISCONNECT) == true){
    tcp_connection_.closeConnection();
    return true;
  } else {
    return false;
  }
}

bool RobotManager::startFRI(){
  return sendCommandAndWait(START_FRI);
}

bool RobotManager::endFRI(){
  return sendCommandAndWait(END_FRI);
}

bool RobotManager::activateControl(){
  return sendCommandAndWait(ACTIVATE_CONTROL);
}

bool RobotManager::deactivateControl(){
  return sendCommandAndWait(DEACTIVATE_CONTROL);
}

bool RobotManager::setControlMode(ControlModeID control_mode_id){
  sendCommand(SET_CONTROL_MODE);
  tcp_connection_.sendByte(control_mode_id);
  wait();
  return assertLastCommandSuccess(SET_CONTROL_MODE);
}

bool RobotManager::setFRIConfig(){

}

void RobotManager::sendCommand(CommandID command_id){
  tcp_connection_.sendByte(command_id);
}

void RobotManager::wait(){
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
}

bool RobotManager::assertLastCommandSuccess(CommandID command_id){
  //TODO: more sophisticated introspection
  if(last_command_state_ == ACCEPTED &&
      last_command_id_ == command_id &&
      last_command_success_ == SUCCESS)
  {
    return true;
  } else {
    return false;
  }
}

bool RobotManager::sendCommandAndWait(CommandID command_id){
  sendCommand(command_id);
  wait();
  return assertLastCommandSuccess(command_id);
}

void RobotManager::handleReceivedTCPData(const std::vector<char>& data){
  if(data.size() == 0){
    return;
  }

  //TODO handle invalid data
  switch((CommandState)data[0]){
    case ACCEPTED:
      if(data.size() < 3){
        //TODO: error
      }
      last_command_state_ = ACCEPTED;
      last_command_id_ = data[1];
      last_command_success_ = data[2];
      break;
    case REJECTED:
      if(data.size() < 2){
        //TODO: error
      }
      last_command_state_ = REJECTED;
      last_command_id_ = data[1];
      break;
    case UNKNOWN:
      last_command_state_ = UNKNOWN;
      break;
    case ERROR_SIGNAL:
      //TODO handle error
      break;
  }
}


}

