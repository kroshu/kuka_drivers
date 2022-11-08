#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

#include <math.h>

int main(int argc, char * argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "moveit_circle",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("moveit_circle");

  // Next step goes here
  // Create the MoveIt MoveGroup Interface
using moveit::planning_interface::MoveGroupInterface;
auto move_group_interface = MoveGroupInterface(node, "iisy_arm");

std::vector<geometry_msgs::msg::Pose> waypoints;

  moveit_msgs::msg::RobotTrajectory trajectory;
  geometry_msgs::msg::Pose msg;
  msg.orientation.x = 0.0;
  msg.orientation.y = 0.0;
  msg.orientation.z = 0.0;
  msg.orientation.w = 1.0;
  msg.position.x = 0.6;
  msg.position.y = 0.1;
  msg.position.z = 0.4;
  waypoints.push_back(msg);

  for (int i = 1; i<63; i++)
  {
    msg.position.y = 0.0 + sin(0.1 * i) / 5;
    msg.position.z = 0.2 + cos(0.1 * i) / 5;
    waypoints.push_back(msg);
  }

double fraction = move_group_interface.computeCartesianPath(waypoints, 0.005, 0.0, trajectory);

if (fraction == -1) RCLCPP_ERROR(logger, "Planing failed!");
else move_group_interface.execute(trajectory);

  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}

