#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <fstream>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "sensor_msgs/msg/joint_state.hpp"

using namespace std::chrono_literals;

class FrameListener : public rclcpp::Node
{
public:
  FrameListener()
  : Node("tf2_frame_listener")
  {
    msg_.name.push_back("joint_a1");
    msg_.name.push_back("joint_a2");
    msg_.name.push_back("joint_a3");
    msg_.name.push_back("joint_a4");
    msg_.name.push_back("joint_a5");
    msg_.name.push_back("joint_a6");
    msg_.position.resize(6);

    csv_in_.open("joint_states.csv");

    tf_buffer_ =
      std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ =
      std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);


    // Create turtle2 velocity publisher
    publisher_ =
      this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 1);

    // Call on_timer function every second
    timer_ = this->create_wall_timer(
      250ms, std::bind(&FrameListener::on_timer, this));
  }

private:
  void on_timer()
  {
    // Store frame names in variables that will be used to
    // compute transformations
    std::string fromFrameRel = "base_link";
    std::string toFrameRel = "flange";

    geometry_msgs::msg::TransformStamped t;

    std::string line;
    if (!std::getline(csv_in_, line)) {return;}
    msg_.header.stamp = this->now();

    std::stringstream s(line);
    double double_value;
    std::string value;
    std::vector<double> joint_angles;
    while (std::getline(s, value, ',')) {
      try {
        double_value = std::stod(value);
      } catch (const std::invalid_argument & ia) {
        RCLCPP_ERROR(this->get_logger(), ia.what());
        RCLCPP_ERROR(
          this->get_logger(),
          "Could not convert to double");
        return;
      }
      if (!isnan(double_value)) {
        joint_angles.push_back(double_value);
      }
      msg_.position = joint_angles;

      publisher_->publish(msg_);
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(100));


    // Look up for the transformation between target_frame and turtle2 frames
    // and send velocity commands for turtle2 to reach target_frame
    try {
      t = tf_buffer_->lookupTransform(
        toFrameRel, fromFrameRel,
        tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_INFO(
        this->get_logger(), "Could not transform %s to %s: %s",
        toFrameRel.c_str(), fromFrameRel.c_str(), ex.what());
      return;
    }
    RCLCPP_INFO(
      this->get_logger(), "Transform:; %lf; %lf; %lf", t.transform.translation.x,
      t.transform.translation.y, t.transform.translation.z);
  }

  rclcpp::TimerBase::SharedPtr timer_{nullptr};
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_{nullptr};
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  sensor_msgs::msg::JointState msg_;
  std::ifstream csv_in_;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FrameListener>());
  rclcpp::shutdown();
  return 0;
}
