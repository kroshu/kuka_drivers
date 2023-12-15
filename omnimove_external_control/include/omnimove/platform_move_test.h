#ifndef PLATFORMMOVETEST_H
#define PLATFORMMOVETEST_H
#include <rclcpp/rclcpp.hpp>
#include <vector>
#include<std_msgs/msg/float64_multi_array.hpp>

class PlatformMoveTest: public rclcpp::Node
{
public:
    PlatformMoveTest();
private:
    std::vector<std::vector<float> > speed_commands_;
    std::vector<std::vector<float> > position_commands_;
    int command_change_timeout_;
    size_t current_command_index_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr velocity_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr position_publisher_;
    void sendSpeedCommand();
};

#endif // PLATFORMMOVETEST_H
