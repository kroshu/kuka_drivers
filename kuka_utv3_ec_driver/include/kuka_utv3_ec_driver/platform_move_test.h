#ifndef PLATFORMMOVETEST_H
#define PLATFORMMOVETEST_H
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_srvs/srv/empty.hpp>
#include <vector>

class PlatformMoveTest : public rclcpp::Node
{
public:
  PlatformMoveTest();

private:
  std::vector<std::vector<float> > speed_commands_;
  std::vector<std::vector<float> > position_commands_;
  int command_change_timeout_;
  size_t current_command_index_;
  rclcpp::CallbackGroup::SharedPtr timer_group_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr velocity_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr position_publisher_;
  std::shared_ptr<std::promise<bool> > move_result_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr platformTestService_;
  void sendSpeedCommand();
  void startPlatformTest(
    const std::shared_ptr<std_srvs::srv::Empty::Request>,
    const std::shared_ptr<std_srvs::srv::Empty::Response>);
};

#endif  // PLATFORMMOVETEST_H
