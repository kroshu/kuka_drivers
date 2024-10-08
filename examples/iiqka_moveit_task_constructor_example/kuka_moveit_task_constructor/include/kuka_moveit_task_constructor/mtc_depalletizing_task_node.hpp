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

#ifndef KUKA_MOVEIT_TASK_CONSTRUCTOR__MTC_DEPALLETIZING_TASK_NODE_HPP_
#define KUKA_MOVEIT_TASK_CONSTRUCTOR__MTC_DEPALLETIZING_TASK_NODE_HPP_

#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>

#include <string>

#include "kuka_moveit_task_constructor/imtc_task.hpp"

#include <rclcpp/rclcpp.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace mtc = moveit::task_constructor;

class MTCDepalletizingTaskNode : public IMTCTask
{
public:
  explicit MTCDepalletizingTaskNode(const rclcpp::NodeOptions & options);

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();
  void attachObject(const std::string & object_id);
  void detachObject(const std::string & object_id);
  void setupPlanningScene() override;
  mtc::Task createTask() override;
  bool doTask(mtc::Task & task) override;

private:
  rclcpp::Node::SharedPtr node_;
  void addPalletObjects();
};

#endif  // KUKA_MOVEIT_TASK_CONSTRUCTOR__MTC_DEPALLETIZING_TASK_NODE_HPP_
