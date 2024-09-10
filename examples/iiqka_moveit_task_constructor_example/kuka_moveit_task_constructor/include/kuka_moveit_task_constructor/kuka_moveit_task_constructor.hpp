#ifndef KUKA_MOVEIT_TASK_CONSTRUCTOR_HPP
#define KUKA_MOVEIT_TASK_CONSTRUCTOR_HPP

#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

namespace mtc = moveit::task_constructor;

class MTCTaskNode
{
public:
  MTCTaskNode(const rclcpp::NodeOptions& options);

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();
  void setupPlanningScene();
  mtc::Task createDepalletizingTask();
  bool doTask(mtc::Task& task);

private:
  rclcpp::Node::SharedPtr node_;
  void addPalletObjects();
  void attachObject(const std::string& object_id);
  void detachAndRemoveObject(const std::string& object_id);
};

#endif // KUKA_MOVEIT_TASK_CONSTRUCTOR_HPP
