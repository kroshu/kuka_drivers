/*
 * robot_control_client.cpp
 *
 *  Created on: Nov 5, 2019
 *      Author: rosdeveloper
 */

#include "kuka_sunrise_interface/robot_control_client.hpp"
#include "kuka_sunrise_interface/robot_commander.hpp"
#include "kuka_sunrise_interface/robot_observer.hpp"

namespace kuka_sunrise_interface{

RobotControlClient::RobotControlClient(rclcpp::Node::SharedPtr robot_control_node):
    robot_observer_(new RobotObserver(robotState(), robot_control_node)),
    robot_commander_(new RobotCommander(robotCommand(), robotState(), robot_control_node)),
    robot_control_node_(robot_control_node)
{

}

bool RobotControlClient::activateControl(){
  return robot_commander_->activate();
}

bool RobotControlClient::deactivateControl(){
  return robot_commander_->deactivate();
}

void RobotControlClient::monitor(){
  rclcpp::Time stamp = ros_clock_.now();
  robot_observer_->publishRobotState(stamp);
}

void RobotControlClient::waitForCommand(){
  rclcpp::Time stamp = ros_clock_.now();
  robot_observer_->publishRobotState(stamp);
  robot_commander_->updateCommand(stamp);
}

void RobotControlClient::command(){
  rclcpp::Time stamp = ros_clock_.now();
  robot_observer_->publishRobotState(stamp);
  robot_commander_->updateCommand(stamp);
}



}


