// Copyright 2022 Áron Svastits
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.


#include <math.h>

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "moveit/move_group_interface/move_group_interface.h"
#include "moveit/planning_scene_interface/planning_scene_interface.h"
#include "moveit_msgs/msg/collision_object.hpp"
#include "moveit_visual_tools/moveit_visual_tools.h"

#define CIRCLE false


int main(int argc, char * argv[])
{
  // Setup
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "moveit_circle",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread([&executor]() {executor.spin();}).detach();

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("moveit_basic_plan");

  // Create Planning group
  static const std::string PLANNING_GROUP = "lbr_iisy_arm";

  // Next step goes here
  // Create the MoveIt MoveGroup Interface
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, PLANNING_GROUP);

  // Create Planning Scene Interface, witch is for adding collision boxes
  using moveit::planning_interface::PlanningSceneInterface;
  auto planning_scene_interface = PlanningSceneInterface();

  // Collision Objects define
  moveit_msgs::msg::CollisionObject collision_object;
  collision_object.header.frame_id = move_group_interface.getPlanningFrame();
  collision_object.id = "box1";
  shape_msgs::msg::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[primitive.BOX_X] = 0.5;
  primitive.dimensions[primitive.BOX_Y] = 0.5;
  primitive.dimensions[primitive.BOX_Z] = 1.2;

  // Define a pose for the box (specified relative to frame_id).
  geometry_msgs::msg::Pose stand_pose;
  stand_pose.orientation.w = 1.0;
  stand_pose.position.x = 0.0;
  stand_pose.position.y = 0.0;
  stand_pose.position.z = -0.6;

  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(stand_pose);
  collision_object.operation = collision_object.ADD;

  std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
  collision_objects.push_back(collision_object);

  planning_scene_interface.addCollisionObjects(collision_objects);
  // End Collision Objects define

  std::vector<geometry_msgs::msg::Pose> waypoints;
  moveit_msgs::msg::RobotTrajectory trajectory;
  geometry_msgs::msg::Pose msg;
  // Add waypoints for planning
  if (!CIRCLE) {
    // Move to point near candle
    msg.orientation.x = 0.0;
    msg.orientation.y = 0.0;
    msg.orientation.z = 0.0;
    msg.orientation.w = 1;
    msg.position.x = 0.05;
    msg.position.y = 0.0;
    msg.position.z = 0.9;
    waypoints.push_back(msg);
  } else {
    // circle facing forward
    msg.orientation.x = 0.0;
    msg.orientation.y = sqrt(2) / 2;
    msg.orientation.z = 0.0;
    msg.orientation.w = sqrt(2) / 2;
    msg.position.x = 0.4;
    // Move in circle
    for (int i = 0; i < 63; i++) {
      msg.position.y = -0.2 + sin(0.1 * i) * 0.15;
      msg.position.z = 0.4 + cos(0.1 * i) * 0.15;
      waypoints.push_back(msg);
    }
  }

  // Get the current joint values
  auto jv = move_group_interface.getCurrentJointValues();

  RCLCPP_INFO(logger, "Start planning");
  // Planning
  // move_group_interface.setPlannerId("");
  double fraction = move_group_interface.computeCartesianPath(waypoints, 0.005, 0.0, trajectory);
  RCLCPP_INFO(logger, "Planning done!");

  if (fraction < 0.1) {RCLCPP_ERROR(logger, "Planning failed!");} else {
    move_group_interface.execute(trajectory);
  }

  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}
