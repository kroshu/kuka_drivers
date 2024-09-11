// Copyright 2024 Ádám Pető
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

#ifndef KUKA_MOVEIT_TASK_CONSTRUCTOR__KUKA_MOVEIT_TASK_CONSTRUCTOR_HPP_
#define KUKA_MOVEIT_TASK_CONSTRUCTOR__KUKA_MOVEIT_TASK_CONSTRUCTOR_HPP_

#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>
#include <moveit/task_constructor/task.h>

#include <string>

#include <rclcpp/rclcpp.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace mtc = moveit::task_constructor;

class MTCTaskNode
{
public:
  explicit MTCTaskNode(const rclcpp::NodeOptions & options);

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();
  void setupPlanningScene();
  mtc::Task createDepalletizingTask();
  bool doTask(mtc::Task & task);

private:
  rclcpp::Node::SharedPtr node_;
  void addPalletObjects();
  void attachObject(const std::string & object_id);
  void detachAndRemoveObject(const std::string & object_id);
};

#endif  // KUKA_MOVEIT_TASK_CONSTRUCTOR__KUKA_MOVEIT_TASK_CONSTRUCTOR_HPP_
