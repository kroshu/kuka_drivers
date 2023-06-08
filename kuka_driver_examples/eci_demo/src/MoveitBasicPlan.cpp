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

static const rclcpp::Logger LOGGER = rclcpp::get_logger("moveit_basic_plan");

moveit_msgs::msg::RobotTrajectory::SharedPtr planThroughwaypoints(
  moveit::planning_interface::MoveGroupInterface & move_group_interface)
{
  std::vector<geometry_msgs::msg::Pose> waypoints;
  moveit_msgs::msg::RobotTrajectory trajectory;
  geometry_msgs::msg::Pose msg;

  // circle facing forward
  msg.orientation.x = 0.0;
  msg.orientation.y = sqrt(2) / 2;
  msg.orientation.z = 0.0;
  msg.orientation.w = sqrt(2) / 2;
  msg.position.x = 0.4;
  // Define waypoints in a circle
  for (int i = 0; i < 63; i++) {
    msg.position.y = -0.2 + sin(0.1 * i) * 0.15;
    msg.position.z = 0.4 + cos(0.1 * i) * 0.15;
    waypoints.push_back(msg);
  }

  RCLCPP_INFO(LOGGER, "Start planning");
  double fraction = move_group_interface.computeCartesianPath(waypoints, 0.005, 0.0, trajectory);
  RCLCPP_INFO(LOGGER, "Planning done!");

  if (fraction < 1) {
    RCLCPP_ERROR(LOGGER, "Could not compute trajectory through all waypoints!");
    return nullptr;
  } else {
    return std::make_shared<moveit_msgs::msg::RobotTrajectory>(trajectory);
  }
}

moveit_msgs::msg::RobotTrajectory::SharedPtr planToPoint(
  moveit::planning_interface::MoveGroupInterface & move_group_interface)
{
  // Create planning request using pilz industrial motion planner
  Eigen::Isometry3d pose = Eigen::Isometry3d(
    Eigen::Translation3d(0.1, 0.0, 0.8) * Eigen::Quaterniond::Identity());
  move_group_interface.setPlanningPipelineId("pilz_industrial_motion_planner");
  move_group_interface.setPlannerId("PTP");
  move_group_interface.setPoseTarget(pose);


  moveit::planning_interface::MoveGroupInterface::Plan plan;
  RCLCPP_INFO(LOGGER, "Sending planning request");
  if (!move_group_interface.plan(plan)) {
    RCLCPP_INFO(LOGGER, "Planning failed");
    return nullptr;
  } else {
    RCLCPP_INFO(LOGGER, "Planning successful");
    return std::make_shared<moveit_msgs::msg::RobotTrajectory>(plan.trajectory_);
  }
}

std::vector<moveit_msgs::msg::CollisionObject> createCollisionObjects(
  moveit::planning_interface::MoveGroupInterface & move_group_interface)
{
  std::vector<moveit_msgs::msg::CollisionObject> collision_objects;

  moveit_msgs::msg::CollisionObject collision_object;
  collision_object.header.frame_id = move_group_interface.getPlanningFrame();
  collision_object.id = "robot_stand";
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

  collision_objects.push_back(collision_object);
  return collision_objects;
}


int main(int argc, char * argv[])
{
  // Setup
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "moveit_basic_plan",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread([&executor]() {executor.spin();}).detach();

  // Define Planning group
  static const std::string PLANNING_GROUP = "lbr_iisy_arm";

  // Create the MoveIt MoveGroup Interface
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, PLANNING_GROUP);

  // Construct and initialize MoveItVisualTools
  auto moveit_visual_tools = moveit_visual_tools::MoveItVisualTools{
    node, "base_link", rviz_visual_tools::RVIZ_MARKER_TOPIC,
    move_group_interface.getRobotModel()};
  moveit_visual_tools.deleteAllMarkers();
  moveit_visual_tools.loadRemoteControl();

  // Define lambda for visualization
  auto const draw_trajectory_tool_path =
    [&moveit_visual_tools, &move_group_interface](auto const trajectory) {
      moveit_visual_tools.deleteAllMarkers();
      moveit_visual_tools.trigger();
      moveit_visual_tools.publishTrajectoryLine(
        trajectory,
        moveit_visual_tools.getRobotModel()->getJointModelGroup(PLANNING_GROUP));
    };

  // Create Planning Scene Interface, witch is for adding collision boxes
  auto planning_scene_interface = moveit::planning_interface::PlanningSceneInterface();

  planning_scene_interface.addCollisionObjects(createCollisionObjects(move_group_interface));
  // End Collision Objects define

  auto planned_trajectory = planToPoint(move_group_interface);
  if (planned_trajectory != nullptr) {
    draw_trajectory_tool_path(*planned_trajectory);
    moveit_visual_tools.trigger();
    moveit_visual_tools.prompt("Press 'Next' in the RvizVisualToolsGui window to execute");
    move_group_interface.execute(*planned_trajectory);
  }

  planned_trajectory = planThroughwaypoints(move_group_interface);
  if (planned_trajectory != nullptr) {
    draw_trajectory_tool_path(*planned_trajectory);
    moveit_visual_tools.trigger();
    moveit_visual_tools.prompt("Press 'Next' in the RvizVisualToolsGui window to execute");
    move_group_interface.execute(*planned_trajectory);
  }

  // Get the current joint values
  auto jv = move_group_interface.getCurrentJointValues();

  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}
