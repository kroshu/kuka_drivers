/*
 * robot_observer.hpp
 *
 *  Created on: Nov 5, 2019
 *      Author: rosdeveloper
 */

#ifndef INCLUDE_KUKA_SUNRISE_INTERFACE_ROBOT_OBSERVER_HPP_
#define INCLUDE_KUKA_SUNRISE_INTERFACE_ROBOT_OBSERVER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

namespace kuka_sunrise_interface{

class RobotObserver{


private:
  rclcpp::Publisher<sensor_msgs::msg::JointState> joint_state_publisher_;
};


}



#endif /* INCLUDE_KUKA_SUNRISE_INTERFACE_ROBOT_OBSERVER_HPP_ */
