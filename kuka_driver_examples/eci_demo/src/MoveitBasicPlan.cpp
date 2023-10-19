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
std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_;
rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr planning_scene_diff_publisher_;

moveit_msgs::msg::RobotTrajectory::SharedPtr planThroughwaypoints()
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
  double fraction = move_group_interface_->computeCartesianPath(waypoints, 0.005, 0.0, trajectory);
  RCLCPP_INFO(LOGGER, "Planning done!");

  if (fraction < 1) {
    RCLCPP_ERROR(LOGGER, "Could not compute trajectory through all waypoints!");
    return nullptr;
  } else {
    return std::make_shared<moveit_msgs::msg::RobotTrajectory>(trajectory);
  }
}

moveit_msgs::msg::RobotTrajectory::SharedPtr planToPoint(
  const Eigen::Isometry3d & pose,
  const std::string & planning_pipeline = "pilz_industrial_motion_planner",
  const std::string & planner_id = "PTP")
{
  // Create planning request using pilz industrial motion planner
  move_group_interface_->setPlanningPipelineId(planning_pipeline);
  move_group_interface_->setPlannerId(planner_id);
  move_group_interface_->setPoseTarget(pose);

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  RCLCPP_INFO(LOGGER, "Sending planning request");
  if (!move_group_interface_->plan(plan)) {
    RCLCPP_INFO(LOGGER, "Planning failed");
    return nullptr;
  } else {
    RCLCPP_INFO(LOGGER, "Planning successful");
    return std::make_shared<moveit_msgs::msg::RobotTrajectory>(plan.trajectory_);
  }
}

std::vector<moveit_msgs::msg::CollisionObject> createRobotPlatform()
{
  std::vector<moveit_msgs::msg::CollisionObject> collision_objects;

  moveit_msgs::msg::CollisionObject collision_object;
  collision_object.header.frame_id = move_group_interface_->getPlanningFrame();
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

void AddObject(const moveit_msgs::msg::CollisionObject & object)
{
  moveit_msgs::msg::PlanningScene planning_scene;
  planning_scene.name = "scene";
  planning_scene.world.collision_objects.push_back(object);
  planning_scene.is_diff = true;
  planning_scene_diff_publisher_->publish(planning_scene);
}

std::vector<moveit_msgs::msg::CollisionObject> createPalletObjects()
{
  std::vector<moveit_msgs::msg::CollisionObject> pallet_objects;

  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      moveit_msgs::msg::CollisionObject pallet_object;
      pallet_object.header.frame_id = move_group_interface_->getPlanningFrame();

      pallet_object.id = "pallet_" + std::to_string(4 * i + j);
      shape_msgs::msg::SolidPrimitive primitive;
      primitive.type = primitive.BOX;
      primitive.dimensions.resize(3);
      primitive.dimensions[primitive.BOX_X] = 0.1;
      primitive.dimensions[primitive.BOX_Y] = 0.1;
      primitive.dimensions[primitive.BOX_Z] = 0.1;

      // Define a pose for the box (specified relative to frame_id).
      geometry_msgs::msg::Pose stand_pose;
      stand_pose.orientation.w = 1.0;
      stand_pose.position.x = 0.3 + i * 0.15;
      stand_pose.position.y = j * 0.15;
      stand_pose.position.z = 0.3;

      pallet_object.primitives.push_back(primitive);
      pallet_object.primitive_poses.push_back(stand_pose);
      pallet_object.operation = pallet_object.ADD;

      pallet_objects.push_back(pallet_object);
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
      AddObject(pallet_object);
    }
  }
  return pallet_objects;
}


bool AttachObject(
  const std::vector<moveit_msgs::msg::CollisionObject> objects,
  const std::string & object_id)
{
  moveit_msgs::msg::PlanningScene planning_scene;
  planning_scene.name = "scene";
  moveit_msgs::msg::AttachedCollisionObject attached_object;
  moveit_msgs::msg::CollisionObject remove_object;

  attached_object.link_name = "flange";
  auto it = std::find_if(
    objects.begin(), objects.end(), [object_id](const moveit_msgs::msg::CollisionObject & obj) {
      return obj.id == object_id;
    });
  if (it != objects.end()) {
    attached_object.object = *it;
    remove_object = *it;
  } else {
    RCLCPP_INFO(LOGGER, "Object not found");
    return false;
  }

  // Carry out the REMOVE + ATTACH operation
  RCLCPP_INFO(LOGGER, "Attaching the object to the hand and removing it from the world.");
  remove_object.operation = remove_object.REMOVE;
  planning_scene.world.collision_objects.push_back(remove_object);
  planning_scene.robot_state.attached_collision_objects.push_back(attached_object);
  planning_scene.robot_state.is_diff = true;
  planning_scene.is_diff = true;
  planning_scene_diff_publisher_->publish(planning_scene);

  return true;
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
  move_group_interface_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
    node,
    PLANNING_GROUP);

  // Construct and initialize MoveItVisualTools
  auto moveit_visual_tools = moveit_visual_tools::MoveItVisualTools{
    node, "base_link", rviz_visual_tools::RVIZ_MARKER_TOPIC,
    move_group_interface_->getRobotModel()};
  moveit_visual_tools.deleteAllMarkers();
  moveit_visual_tools.loadRemoteControl();
  moveit_visual_tools.trigger();

  // Define lambda for visualization
  auto const draw_trajectory_tool_path =
    [&moveit_visual_tools](auto const trajectory) {
      moveit_visual_tools.deleteAllMarkers();
      moveit_visual_tools.publishTrajectoryLine(
        trajectory,
        moveit_visual_tools.getRobotModel()->getJointModelGroup(PLANNING_GROUP));
    };

  // Create Planning Scene Interface, which is for adding collision boxes
  auto planning_scene_interface = moveit::planning_interface::PlanningSceneInterface();
  planning_scene_diff_publisher_ = node->create_publisher<moveit_msgs::msg::PlanningScene>(
    "planning_scene", 1);

  // Add robot platform
  planning_scene_interface.addCollisionObjects(createRobotPlatform());
  moveit_visual_tools.trigger();
  moveit_visual_tools.prompt("Press 'Next' in the RvizVisualToolsGui window to execute");

  // Add pallets
  auto pallet_objects = createPalletObjects();
  moveit_visual_tools.trigger();
  moveit_visual_tools.prompt("Press 'Next' in the RvizVisualToolsGui window to execute");

  // Attach 1. pallet to robot flange
  AttachObject(pallet_objects, "pallet_0");
  moveit_visual_tools.trigger();
  moveit_visual_tools.prompt("Press 'Next' in the RvizVisualToolsGui window to execute");


  // Define goal pos and plan there
  Eigen::Isometry3d pose = Eigen::Isometry3d(
    Eigen::Translation3d(0.1, 0.0, 0.8) * Eigen::Quaterniond::Identity());
  auto planned_trajectory = planToPoint(pose);
  if (planned_trajectory != nullptr) {
    draw_trajectory_tool_path(*planned_trajectory);
    moveit_visual_tools.trigger();
    moveit_visual_tools.prompt("Press 'Next' in the RvizVisualToolsGui window to execute");
    move_group_interface_->execute(*planned_trajectory);
  }

  planned_trajectory = planThroughwaypoints();
  if (planned_trajectory != nullptr) {
    draw_trajectory_tool_path(*planned_trajectory);
    moveit_visual_tools.trigger();
    moveit_visual_tools.prompt("Press 'Next' in the RvizVisualToolsGui window to execute");
    move_group_interface_->execute(*planned_trajectory);
  }

  // Get the current joint values
  auto jv = move_group_interface_->getCurrentJointValues();

  // Shutdown ROS
  rclcpp::shutdown();
  move_group_interface_.reset();
  planning_scene_diff_publisher_.reset();
  return 0;
}
