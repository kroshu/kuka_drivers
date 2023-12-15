#include "omnimove/platform_move_test.h"
#include <chrono>

using namespace rclcpp;
using namespace std::chrono_literals;


PlatformMoveTest::PlatformMoveTest():Node("PlatformMoveTest")
{
    publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/platform/forward_command_controller/commands", 10);
    timer_ = this->create_wall_timer(10000ms, std::bind(&PlatformMoveTest::sendSpeedCommand, this));
    command_change_timeout_ = 5;
    current_command_index_ = 0;
    speed_commands_.push_back(std::vector<float>{50,0,0});
    speed_commands_.push_back(std::vector<float>{-50,0,0});
    speed_commands_.push_back(std::vector<float>{0,50,0});
    speed_commands_.push_back(std::vector<float>{0,-50,0});
    speed_commands_.push_back(std::vector<float>{50, 50,0});
    speed_commands_.push_back(std::vector<float>{-50, -50,0});
    speed_commands_.push_back(std::vector<float>{50, -50,0});
    speed_commands_.push_back(std::vector<float>{-50, 50,0});
    speed_commands_.push_back(std::vector<float>{0, 0,50});
    speed_commands_.push_back(std::vector<float>{0, 0,-50});
    speed_commands_.push_back(std::vector<float>{0, 0, 0});


}

void PlatformMoveTest::sendSpeedCommand(){
    if (current_command_index_ >= speed_commands_.size()){
        timer_->cancel();
        exit(0);
    }
    std_msgs::msg::Float64MultiArray velocity;
    auto cur_speed = speed_commands_[current_command_index_++];
    for(size_t i = 0; i < cur_speed.size(); ++i){
        velocity.data.push_back(cur_speed[i]);
    }

    publisher_->publish(velocity);
}

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PlatformMoveTest>());
    return 0;
}
