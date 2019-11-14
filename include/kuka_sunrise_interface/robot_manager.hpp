/*
 * robot_manager.hpp
 *
 *  Created on: Nov 4, 2019
 *      Author: rosdeveloper
 */

#ifndef INCLUDE_KUKA_SUNRISE_INTERFACE_ROBOT_MANAGER_HPP_
#define INCLUDE_KUKA_SUNRISE_INTERFACE_ROBOT_MANAGER_HPP_

#include <memory>
#include <functional>
#include <vector>

namespace kuka_sunrise_interface{
//forward declaration
class TCPConnection;

enum CommandState: char{
  ACCEPTED = 1,
  REJECTED = 2,
  UNKNOWN = 3,
  ERROR_CONTROL_ENDED = 4,
  ERROR_FRI_ENDED = 5
};

enum CommandID: char{
  CONNECT = 1,
  DISCONNECT = 2,
  START_FRI = 3,
  END_FRI = 4,
  ACTIVATE_CONTROL = 5,
  DEACTIVATE_CONTROL = 6,
  GET_FRI_CONFIG = 7,
  SET_FRI_CONFIG = 8,
  GET_CONTROL_MODE = 9,
  SET_CONTROL_MODE = 10
};

enum CommandSuccess: char{
  SUCCESS = 1,
  NO_SUCCESS = 2
};

enum ControlModeID: char{
  POSITION = 1,
  JOINT_IMPEDANCE = 2
};


class RobotManager{
public:
  RobotManager(std::function<void(void)> handle_control_ended_error_callback,  std::function<void(void)> handle_fri_ended_callback);
  ~RobotManager();
  bool connect(const char* server_addr, int server_port);
  bool disconnect();
  bool startFRI();
  bool endFRI();
  bool activateControl();
  bool deactivateControl();
  bool setControlMode(ControlModeID control_mode_id);
  //bool getControlMode();
  bool setFRIConfig(int remote_port, int send_period_ms, int receive_multiplier);
  //bool getFRIConfig();

  bool isConnected();

private:
  std::unique_ptr<TCPConnection> tcp_connection_;

  std::function<void(void)> handleControlEndedError_;
  std::function<void(void)> handleFRIEndedError_;

  void handleReceivedTCPData(const std::vector<char>& data);
  void connectionLostCallback(const char* server_addr, int server_port);

  CommandState last_command_state_;
  CommandID last_command_id_;
  CommandSuccess last_command_success_;

  void wait();
  bool assertLastCommandSuccess(CommandID command_id);
  bool sendCommandAndWait(CommandID command_id);
  bool sendCommandAndWait(CommandID command_id, const std::vector<char>& command_data);

};









}



#endif /* INCLUDE_KUKA_SUNRISE_INTERFACE_ROBOT_MANAGER_HPP_ */
