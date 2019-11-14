/*
 * robot_manager.cpp
 *
 *  Created on: Nov 4, 2019
 *      Author: rosdeveloper
 */

#include <kuka_sunrise_interface/robot_manager.hpp>
#include <kuka_sunrise_interface/internal/serialization.hpp>
#include <kuka_sunrise_interface/tcp_connection.hpp>
#include <thread>
#include <chrono>

namespace kuka_sunrise_interface{

RobotManager::RobotManager(std::function<void(void)> handle_control_ended_error_callback,  std::function<void(void)> handle_fri_ended_callback):
    handleControlEndedError_(handle_control_ended_error_callback),
    handleFRIEndedError_(handle_fri_ended_callback),
    last_command_state_(ACCEPTED),
    last_command_id_(CONNECT),
    last_command_success_(NO_SUCCESS)
{

}

RobotManager::~RobotManager(){
  disconnect();
}

bool RobotManager::connect(const char* server_addr, int server_port){
  //TODO: check if already connected
  try{
    tcp_connection_ = std::make_unique<TCPConnection>(server_addr, server_port,
                                    [this](std::vector<char> data){this->handleReceivedTCPData(data);},
                                    [this](const char* server_addr, int server_port){this->connectionLostCallback(server_addr, server_port);});
  } catch(...){
    tcp_connection_.reset();
  }

  if(!isConnected()){
    return false;
  }
  return sendCommandAndWait(CONNECT);

}

bool RobotManager::disconnect(){
  if(sendCommandAndWait(DISCONNECT) == true){
    tcp_connection_->closeConnection();
    tcp_connection_.reset();
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
  std::vector<char> command_data = { control_mode_id };
  return sendCommandAndWait(SET_CONTROL_MODE, command_data);
}

bool RobotManager::setFRIConfig(int remote_port, int send_period_ms, int receive_multiplier){
  std::vector<char> serialized(3*sizeof(int));
  int msg_size = 0;
  msg_size += serializeNext(remote_port, serialized);
  msg_size += serializeNext(send_period_ms, serialized);
  msg_size += serializeNext(receive_multiplier, serialized);
  return sendCommandAndWait(SET_FRI_CONFIG, serialized);
}

bool RobotManager::isConnected(){
  if(tcp_connection_){
    return true;
  } else {
    return false;
  }
}

void RobotManager::wait(){
  std::this_thread::sleep_for(std::chrono::milliseconds(3500));
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
  printf("command sent: %u\n", command_id);
  tcp_connection_->sendByte(command_id);
  wait();
  printf("feedback: \n\t%u\t%u\t%u", last_command_state_, last_command_id_, last_command_success_);
  return assertLastCommandSuccess(command_id);
}

bool RobotManager::sendCommandAndWait(CommandID command_id, const std::vector<char>& command_data){
  std::vector<char> msg(1 + command_data.size());
  msg[0] = command_id;
  msg.insert(msg.end(), command_data.begin(), command_data.end());
  tcp_connection_->sendBytes(msg);
  wait();
  return assertLastCommandSuccess(command_id);
}

void RobotManager::handleReceivedTCPData(const std::vector<char>& data){
  if(data.size() == 0){
    return;
  }
  pthread_t handler_thread;

  //TODO handle invalid data
  switch((CommandState)data[0]){
    case ACCEPTED:
      if(data.size() < 3){
        //TODO: error
      }
      last_command_state_ = ACCEPTED;
      last_command_id_ = (CommandID)data[1];
      last_command_success_ = (CommandSuccess)data[2];
      break;
    case REJECTED:
      if(data.size() < 2){
        //TODO: error
      }
      last_command_state_ = REJECTED;
      last_command_id_ = (CommandID)data[1];
      break;
    case UNKNOWN:
      last_command_state_ = UNKNOWN;
      break;
    case ERROR_CONTROL_ENDED:
      pthread_create(&handler_thread, nullptr, [](void* self)->void*{static_cast<RobotManager*>(self)->handleControlEndedError_(); return nullptr;}, this);
      pthread_detach(handler_thread); //TODO ther might be a better way to do this
      break;
    case ERROR_FRI_ENDED:
      pthread_create(&handler_thread, nullptr, [](void* self)->void*{static_cast<RobotManager*>(self)->handleFRIEndedError_(); return nullptr;}, this);
      pthread_detach(handler_thread); //TODO ther might be a better way to do this
      break;
  }
}

void RobotManager::connectionLostCallback(const char* server_addr, int server_port){
  printf("Connection lost, trying to reconnect");
  connect(server_addr, server_port);
}


}

