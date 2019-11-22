/*
 * tcp_connection.cpp
 *
 *  Created on: Nov 4, 2019
 *      Author: rosdeveloper
 */

#include <kuka_sunrise_interface/tcp_connection.hpp>
#include <stdexcept>

namespace kuka_sunrise_interface{

TCPConnection::TCPConnection(const char* server_addr, int server_port,
                             std::function<void(const std::vector<std::uint8_t>&)> data_received_callback, std::function<void(const char* server_addr, const int server_port)> connection_lost_callback):
    dataReceivedCallback_(data_received_callback),
    connectionLostCallback_(connection_lost_callback),
    socket_desc_(socket(AF_INET, SOCK_STREAM, 0)),
    cancelled_(false)
{
  if(socket_desc_ == -1){
    throw std::runtime_error("Could not create socket");
  }
  if(inet_aton(server_addr, &server_.sin_addr) == 0){
    close(socket_desc_);
    std::string errormsg = std::string("Received invalid server IP address: ") + std::string(server_addr);
    throw std::invalid_argument(errormsg.c_str());
  }
  server_.sin_family = AF_INET;
  server_.sin_port = htons(server_port);
  if(connect(socket_desc_, (struct sockaddr *)&server_, sizeof(server_))){
    throw std::runtime_error("Could not connect to server");
  }
  pthread_create(&read_thread_, NULL, &TCPConnection::listen_helper, this);
}

bool TCPConnection::sendByte(std::uint8_t data){
  int sent_length = write(socket_desc_, &data, 1);
  if(sent_length < 0){
    return false;
  } //TODO: handle other kind of errors?
  return true;
}

bool TCPConnection::sendBytes(const std::vector<std::uint8_t>& data){
  int sent_length = write(socket_desc_, data.data(), data.size());
  if(sent_length < 0){
    return false;
  } //TODO: handle other kind of errors?
  return true;
}

void TCPConnection::closeConnection(){
  //TODO what happens upon multiple calls to close?
  cancelled_.store(true);
  pthread_cancel(read_thread_);
  close(socket_desc_);
  //pthread_join(read_thread_, NULL);
  //TODO: handle errors of close?
}

TCPConnection::~TCPConnection(){
  closeConnection();
  //TODO: handle errors of close?
}

void* TCPConnection::listen_helper(void *tcpConnection){
  ((TCPConnection *)tcpConnection)->listen();
  return NULL;
}

void TCPConnection::listen(){
  std::uint8_t msg_buffer[300];
  while(!cancelled_.load()){
    int length = recv(socket_desc_, msg_buffer, 300, 0);
    if(length < 0){
      if(cancelled_.load()){
        break;
      }
      //TODO: handle error
    } else if(length == 0) { //TODO: is this the way to check for connection loss?
      connectionLostCallback_(inet_ntoa(server_.sin_addr), ntohs(server_.sin_port));
      break;
    } else{
      std::vector<std::uint8_t> server_msg(msg_buffer, msg_buffer + length);
      dataReceivedCallback_(server_msg);
    }
  }
}


}



