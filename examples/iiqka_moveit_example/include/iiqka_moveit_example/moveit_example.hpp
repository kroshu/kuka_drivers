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

#ifndef IIQKA_MOVEIT_EXAMPLE__MOVEIT_EXAMPLE_HPP_
#define IIQKA_MOVEIT_EXAMPLE__MOVEIT_EXAMPLE_HPP_

#include <math.h>

#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/vector3.hpp"
#include "moveit/move_group_interface/move_group_interface.h"
#include "moveit/planning_scene_interface/planning_scene_interface.h"
#include "moveit_msgs/msg/collision_object.hpp"
#include "moveit_visual_tools/moveit_visual_tools.h"
#include "rclcpp/rclcpp.hpp"

class MoveitExample : public rclcpp::Node
{
public:
  MoveitExample() : rclcpp::Node("moveit_example") {}

  void initialize()
  {
    move_group_interface_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
      shared_from_this(), PLANNING_GROUP);

    moveit_visual_tools_ = std::make_shared<moveit_visual_tools::MoveItVisualTools>(
      shared_from_this(), "world", rviz_visual_tools::RVIZ_MARKER_TOPIC,
      move_group_interface_->getRobotModel());

    moveit_visual_tools_->deleteAllMarkers();
    moveit_visual_tools_->loadRemoteControl();
    moveit_visual_tools_->trigger();

    planning_scene_diff_publisher_ =
      this->create_publisher<moveit_msgs::msg::PlanningScene>("planning_scene", 10);

    move_group_interface_->setMaxVelocityScalingFactor(0.1);
    move_group_interface_->setMaxAccelerationScalingFactor(0.1);
  }

  moveit_msgs::msg::RobotTrajectory::SharedPtr drawCircle()
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
    for (int i = 0; i < 63; i++)
    {
      msg.position.y = -0.2 + sin(0.1 * i) * 0.15;
      msg.position.z = 0.4 + cos(0.1 * i) * 0.15;
      waypoints.push_back(msg);
    }

    RCLCPP_INFO(LOGGER, "Start planning");
    double fraction =
      move_group_interface_->computeCartesianPath(waypoints, 0.005, 0.0, trajectory);
    RCLCPP_INFO(LOGGER, "Planning done!");

    if (fraction < 1)
    {
      RCLCPP_ERROR(LOGGER, "Could not compute trajectory through all waypoints!");
      return nullptr;
    }
    else
    {
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
    if (!move_group_interface_->plan(plan))
    {
      RCLCPP_INFO(LOGGER, "Planning failed");
      return nullptr;
    }
    else
    {
      RCLCPP_INFO(LOGGER, "Planning successful");
      return std::make_shared<moveit_msgs::msg::RobotTrajectory>(plan.trajectory_);
    }
  }

  moveit_msgs::msg::RobotTrajectory::SharedPtr planToPosition(const std::vector<double> & joint_pos)
  {
    move_group_interface_->setJointValueTarget(joint_pos);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    RCLCPP_INFO(LOGGER, "Sending planning request");
    if (!move_group_interface_->plan(plan))
    {
      RCLCPP_INFO(LOGGER, "Planning failed");
      return nullptr;
    }
    else
    {
      RCLCPP_INFO(LOGGER, "Planning successful");
      return std::make_shared<moveit_msgs::msg::RobotTrajectory>(plan.trajectory_);
    }
  }

  moveit_msgs::msg::RobotTrajectory::SharedPtr planToPointUntilSuccess(
    const Eigen::Isometry3d & pose,
    const std::string & planning_pipeline = "pilz_industrial_motion_planner",
    const std::string & planner_id = "PTP")
  {
    // Create planning request using given motion planner
    move_group_interface_->setPlanningPipelineId(planning_pipeline);
    move_group_interface_->setPlannerId(planner_id);
    move_group_interface_->setPoseTarget(pose);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    RCLCPP_INFO(LOGGER, "Sending planning request");
    moveit::core::MoveItErrorCode err_code;
    auto start = std::chrono::high_resolution_clock::now();
    do
    {
      RCLCPP_INFO(LOGGER, "Planning ...");
      err_code = move_group_interface_->plan(plan);
    } while (err_code != moveit::core::MoveItErrorCode::SUCCESS);
    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
    RCLCPP_INFO(LOGGER, "Planning successful after %li ms", duration.count());
    return std::make_shared<moveit_msgs::msg::RobotTrajectory>(plan.trajectory_);
  }

  void AddObject(const moveit_msgs::msg::CollisionObject & object)
  {
    moveit_msgs::msg::PlanningScene planning_scene;
    planning_scene.name = "scene";
    planning_scene.world.collision_objects.push_back(object);
    planning_scene.is_diff = true;
    planning_scene_diff_publisher_->publish(planning_scene);
  }

  void addRobotPlatform()
  {
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
    geometry_msgs::msg::Pose stand_pose1;
    stand_pose1.orientation.w = 1.0;
    stand_pose1.position.x = 0.0;
    stand_pose1.position.y = 0.0;
    stand_pose1.position.z = -0.6;

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(stand_pose1);
    collision_object.operation = collision_object.ADD;

    AddObject(collision_object);
  }

  void addCollisionBox(
    const geometry_msgs::msg::Vector3 & position, const geometry_msgs::msg::Vector3 & size)
  {
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = move_group_interface_->getPlanningFrame();
    collision_object.id = "collision_box";
    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[primitive.BOX_X] = size.x;
    primitive.dimensions[primitive.BOX_Y] = size.y;
    primitive.dimensions[primitive.BOX_Z] = size.z;

    // Define a pose for the box (specified relative to frame_id).
    geometry_msgs::msg::Pose stand_pose;
    stand_pose.orientation.w = 1.0;
    stand_pose.position.x = position.x;
    stand_pose.position.y = position.y;
    stand_pose.position.z = position.z;

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(stand_pose);
    collision_object.operation = collision_object.ADD;

    AddObject(collision_object);
  }

  void addPalletObjects()
  {
    for (int k = 0; k < 3; k++)
    {
      for (int j = 0; j < 3; j++)
      {
        for (int i = 0; i < 3; i++)
        {
          moveit_msgs::msg::CollisionObject pallet_object;
          pallet_object.header.frame_id = move_group_interface_->getPlanningFrame();

          pallet_object.id = "pallet_" + std::to_string(9 * k + 3 * j + i);
          shape_msgs::msg::SolidPrimitive primitive;
          primitive.type = primitive.BOX;
          primitive.dimensions.resize(3);
          primitive.dimensions[primitive.BOX_X] = 0.097;
          primitive.dimensions[primitive.BOX_Y] = 0.097;
          primitive.dimensions[primitive.BOX_Z] = 0.097;

          // Define a pose for the box (specified relative to frame_id).
          geometry_msgs::msg::Pose stand_pose;
          stand_pose.orientation.w = 1.0;
          stand_pose.position.x = 0.3 + i * 0.1;
          stand_pose.position.y = -0.1 + j * 0.1;
          stand_pose.position.z = 0.3 - 0.1 * k;

          pallet_object.primitives.push_back(primitive);
          pallet_object.primitive_poses.push_back(stand_pose);
          pallet_object.operation = pallet_object.ADD;

          AddObject(pallet_object);
          std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
      }
    }
  }

  void AttachObject(const std::string & object_id)
  {
    moveit_msgs::msg::PlanningScene planning_scene;
    planning_scene.name = "scene";
    moveit_msgs::msg::AttachedCollisionObject attached_object;

    attached_object.link_name = "flange";
    attached_object.object.id = object_id;

    // Carry out the REMOVE + ATTACH operation
    RCLCPP_INFO(LOGGER, "Attaching the object to the hand and removing it from the world.");
    planning_scene.robot_state.attached_collision_objects.push_back(attached_object);
    planning_scene.robot_state.is_diff = true;
    planning_scene.is_diff = true;
    planning_scene_diff_publisher_->publish(planning_scene);
  }

  void DetachAndRemoveObject(const std::string & object_id)
  {
    moveit_msgs::msg::PlanningScene planning_scene;
    planning_scene.name = "scene";
    moveit_msgs::msg::AttachedCollisionObject attached_object;

    attached_object.link_name = "flange";
    attached_object.object.id = object_id;
    attached_object.object.operation = attached_object.object.REMOVE;

    // Carry out the DETACH operation
    RCLCPP_INFO(LOGGER, "Detaching the object from the hand");
    planning_scene.robot_state.attached_collision_objects.push_back(attached_object);
    planning_scene.robot_state.is_diff = true;
    planning_scene.world.collision_objects.push_back(attached_object.object);
    planning_scene.is_diff = true;
    planning_scene_diff_publisher_->publish(planning_scene);
  }

  void setOrientationConstraint(const geometry_msgs::msg::Quaternion & orientation)
  {
    moveit_msgs::msg::OrientationConstraint orientation_constraint;
    moveit_msgs::msg::Constraints constraints;
    orientation_constraint.header.frame_id = move_group_interface_->getPlanningFrame();
    orientation_constraint.link_name = move_group_interface_->getEndEffectorLink();
    orientation_constraint.orientation = orientation;
    orientation_constraint.absolute_x_axis_tolerance = 0.2;
    orientation_constraint.absolute_y_axis_tolerance = 0.2;
    orientation_constraint.absolute_z_axis_tolerance = 0.2;
    orientation_constraint.weight = 1.0;

    constraints.orientation_constraints.emplace_back(orientation_constraint);
    move_group_interface_->setPathConstraints(constraints);
  }

  void clearConstraints() { move_group_interface_->clearPathConstraints(); }

  void drawTrajectory(const moveit_msgs::msg::RobotTrajectory & trajectory)
  {
    moveit_visual_tools_->deleteAllMarkers();
    moveit_visual_tools_->publishTrajectoryLine(
      trajectory, moveit_visual_tools_->getRobotModel()->getJointModelGroup(PLANNING_GROUP));
  }

  void drawTitle(const std::string & text)
  {
    auto const text_pose = []
    {
      auto msg = Eigen::Isometry3d::Identity();
      msg.translation().z() = 1.0;
      return msg;
    }();
    moveit_visual_tools_->publishText(
      text_pose, text, rviz_visual_tools::RED, rviz_visual_tools::XXLARGE);
  }

  void addBreakPoint()
  {
    moveit_visual_tools_->trigger();
    moveit_visual_tools_->prompt("Press 'Next' in the RvizVisualToolsGui window to execute");
  }

  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> moveGroupInterface()
  {
    return move_group_interface_;
  }

protected:
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_;
  rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr planning_scene_diff_publisher_;
  std::shared_ptr<moveit_visual_tools::MoveItVisualTools> moveit_visual_tools_;
  const rclcpp::Logger LOGGER = rclcpp::get_logger("moveit_basic_plan");
  const std::string PLANNING_GROUP = "manipulator";
};

#endif  // IIQKA_MOVEIT_EXAMPLE__MOVEIT_EXAMPLE_HPP_
