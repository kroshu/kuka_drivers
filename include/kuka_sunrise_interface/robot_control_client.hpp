/*
 * RobotControlClient.hpp
 *
 *  Created on: Nov 5, 2019
 *      Author: rosdeveloper
 */

#ifndef INCLUDE_KUKA_SUNRISE_INTERFACE_ROBOT_CONTROL_CLIENT_HPP_
#define INCLUDE_KUKA_SUNRISE_INTERFACE_ROBOT_CONTROL_CLIENT_HPP_

#include "fri_client/friLBRClient.h"

#include <condition_variable>
#include <memory>

namespace kuka_sunrise_interface{

class RobotObserver;
class RobotCommander;

class RobotControlClient: public KUKA::FRI::LBRClient{
public:
  bool activateControl();
  bool deactivateControl();
  bool setControlMode();
  bool selectActiveIOs();

  virtual void onStateChanged(KUKA::FRI::ESessionState oldState, KUKA::FRI::ESessionState newState);
  virtual void monitor();
  virtual void waitForCommand();
  virtual void command();

private:
  std::unique_ptr<RobotObserver> robot_observer_;
  std::unique_ptr<RobotCommander> robot_commander_;

  bool command_ready_;
  std::mutex mutex_;
  std::condition_variable cv_;

};

}



#endif /* INCLUDE_KUKA_SUNRISE_INTERFACE_ROBOT_CONTROL_CLIENT_HPP_ */