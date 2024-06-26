#include "kuka_utv3_ec_driver/platform_move_test.h"
#include <chrono>
#include <future>
#include <std_srvs/srv/empty.hpp>

using namespace rclcpp;
using namespace std::chrono_literals;

PlatformMoveTest::PlatformMoveTest() : Node("PlatformMoveTest")
{
  this->declare_parameter("agv_type", "omnimove");
  this->declare_parameter("start_service", false);

  std::string agv_type = this->get_parameter("agv_type").as_string();
  bool start_service = this->get_parameter("start_service").as_bool();

  velocity_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
    "/platform/velocity_command_controller/commands", 10);
  position_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
    "/platform/position_command_controller/commands", 10);
  timer_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  if (start_service)
  {
    platformTestService_ = this->create_service<std_srvs::srv::Empty>(
      "start_platform_test",
      std::bind(
        &PlatformMoveTest::startPlatformTest, this, std::placeholders::_1, std::placeholders::_2));
  }
  else
  {
    timer_ = this->create_wall_timer(
      100ms, std::bind(&PlatformMoveTest::sendSpeedCommand, this), timer_group_);
    command_change_timeout_ = 5;
    current_command_index_ = 0;
  }
  if (this->get_parameter("agv_type").as_string() == "caterpillar")
  {
    speed_commands_.push_back(std::vector<float>{50, 0});
    position_commands_.push_back(std::vector<float>{0, 0, 0, 0, 100});
    speed_commands_.push_back(std::vector<float>{-50, 0});
    position_commands_.push_back(std::vector<float>{0, 0, 0, 0, 0});
    speed_commands_.push_back(std::vector<float>{0, 500});
    position_commands_.push_back(std::vector<float>{0, 0, 0, 0, 100});
    speed_commands_.push_back(std::vector<float>{0, -500});
    position_commands_.push_back(std::vector<float>{0, 0, 0, 0, 0});
    speed_commands_.push_back(std::vector<float>{0, 0});
    position_commands_.push_back(std::vector<float>{100, 100, 0, 0, 0});
    speed_commands_.push_back(std::vector<float>{0, 0});
    position_commands_.push_back(std::vector<float>{0, 0, 0, 0, 0});
    speed_commands_.push_back(std::vector<float>{0, 0});
    position_commands_.push_back(std::vector<float>{0, 0, 100, 100, 0});
    speed_commands_.push_back(std::vector<float>{0, 0});
    position_commands_.push_back(std::vector<float>{0, 0, 0, 0, 0});
  }
  else
  {
    speed_commands_.push_back(std::vector<float>{50, 0, 0});
    speed_commands_.push_back(std::vector<float>{-50, 0, 0});
    speed_commands_.push_back(std::vector<float>{0, 50, 0});
    speed_commands_.push_back(std::vector<float>{0, -50, 0});
    speed_commands_.push_back(std::vector<float>{50, 50, 0});
    speed_commands_.push_back(std::vector<float>{-50, -50, 0});
    speed_commands_.push_back(std::vector<float>{50, -50, 0});
    speed_commands_.push_back(std::vector<float>{-50, 50, 0});
    speed_commands_.push_back(std::vector<float>{0, 0, 50});
    speed_commands_.push_back(std::vector<float>{0, 0, -50});
    speed_commands_.push_back(std::vector<float>{0, 0, 0});
  }
}

void PlatformMoveTest::startPlatformTest(
  const std::shared_ptr<std_srvs::srv::Empty::Request>,
  const std::shared_ptr<std_srvs::srv::Empty::Response>)
{
  move_result_ = std::make_shared<std::promise<bool> >();
  timer_ = this->create_wall_timer(
    100ms, std::bind(&PlatformMoveTest::sendSpeedCommand, this), timer_group_);
  command_change_timeout_ = 5;
  current_command_index_ = 0;
  move_result_->get_future().wait();
}

void PlatformMoveTest::sendSpeedCommand()
{
  if (current_command_index_ >= speed_commands_.size())
  {
    timer_->cancel();
    if (move_result_.get() != NULL)
    {
      move_result_->set_value(true);
    }
    else
    {
      exit(0);
    }
  }
  std_msgs::msg::Float64MultiArray velocity;
  std_msgs::msg::Float64MultiArray position;

  size_t command_index = current_command_index_;
  static int current_command_counter = 0;
  double epsilon = 0.00001;
  if (position_commands_.size() > current_command_index_)
  {
    auto pos = speed_commands_[current_command_index_];
  }
  if (current_command_counter > 100)
  {
    current_command_counter = 0;
    current_command_index_++;
  }

  auto cur_speed = speed_commands_[command_index];
  for (size_t i = 0; i < cur_speed.size(); ++i)
  {
    velocity.data.push_back(cur_speed[i] + current_command_counter * epsilon);
  }
  if (position_commands_.size() > command_index)
  {
    auto cur_pos = position_commands_[command_index];
    for (size_t i = 0; i < cur_pos.size(); ++i)
    {
      position.data.push_back(cur_pos[i]);
    }
  }

  current_command_counter++;
  velocity_publisher_->publish(velocity);
  position_publisher_->publish(position);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node =
    std::make_shared<PlatformMoveTest>();  // rclcpp::spin(std::make_shared<PlatformMoveTest>());
  rclcpp::WallRate loop_rate(30);
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  return 0;
}
