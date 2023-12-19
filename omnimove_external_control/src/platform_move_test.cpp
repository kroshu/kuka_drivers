#include "omnimove/platform_move_test.h"
#include <chrono>

using namespace rclcpp;
using namespace std::chrono_literals;


PlatformMoveTest::PlatformMoveTest():Node("PlatformMoveTest")
{
    this->declare_parameter("agv_type", "omnimove");
    std::string agv_type = this->get_parameter("agv_type").as_string();
    velocity_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/platform/velocity_command_controller/commands", 10);
    position_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/platform/position_command_controller/commands", 10);
    timer_ = this->create_wall_timer(100ms, std::bind(&PlatformMoveTest::sendSpeedCommand, this));
    command_change_timeout_ = 5;
    current_command_index_ = 0;
    if (this->get_parameter("agv_type").as_string() == "caterpillar")
    {
        speed_commands_.push_back(std::vector<float>{50,0});
        position_commands_.push_back(std::vector<float>{0,0,0,0,100});
        speed_commands_.push_back(std::vector<float>{-50,0});
        position_commands_.push_back(std::vector<float>{0,0,0,0,0});
        speed_commands_.push_back(std::vector<float>{0,50});
        position_commands_.push_back(std::vector<float>{0,0,0,0,100});
        speed_commands_.push_back(std::vector<float>{0,-50});
        position_commands_.push_back(std::vector<float>{0,0,0,0,0});
        speed_commands_.push_back(std::vector<float>{0, 0});
        position_commands_.push_back(std::vector<float>{100,100,0,0,0});
        speed_commands_.push_back(std::vector<float>{0, 0});
        position_commands_.push_back(std::vector<float>{0,0,0,0,0});
        speed_commands_.push_back(std::vector<float>{0, 0});
        position_commands_.push_back(std::vector<float>{0,0,100,100,0});
        speed_commands_.push_back(std::vector<float>{0, 0});
        position_commands_.push_back(std::vector<float>{0,0,0,0,0});
    } else
    {
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

}

void PlatformMoveTest::sendSpeedCommand(){
    if (current_command_index_ >= speed_commands_.size()){
        timer_->cancel();
        exit(0);
    }
    std_msgs::msg::Float64MultiArray velocity;
    std_msgs::msg::Float64MultiArray position;

    size_t command_index= current_command_index_;
    static int current_command_counter = 0;
    double epsilon = 0.00001;
    if (position_commands_.size() > current_command_index_){
        auto pos = speed_commands_[current_command_index_];
        if (std::accumulate(pos.begin(), pos.end(),0)==0){
            epsilon = 0;
        }
    }
    if (current_command_counter > 100){
        current_command_counter = 0;
        current_command_index_++;
    }

    auto cur_speed = speed_commands_[command_index];
    for(size_t i = 0; i < cur_speed.size(); ++i){
        velocity.data.push_back(cur_speed[i] + current_command_counter*epsilon);
    }
    if (position_commands_.size() > command_index){
        auto cur_pos = position_commands_[command_index];
        for(size_t i = 0; i < cur_pos.size(); ++i){
            position.data.push_back(cur_pos[i]);
        }
    }

    current_command_counter++;
    velocity_publisher_->publish(velocity);
    position_publisher_->publish(position);
}

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PlatformMoveTest>());
    return 0;
}
