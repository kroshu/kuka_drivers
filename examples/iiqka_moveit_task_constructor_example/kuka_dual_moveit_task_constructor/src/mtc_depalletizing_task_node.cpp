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

#include "kuka_dual_moveit_task_constructor/mtc_depalletizing_task_node.hpp"

static const rclcpp::Logger LOGGER = rclcpp::get_logger("mtc_depalletizing_task_node");

double PALLET_SIZE = 0.097;
double PALLET_DISTANCE = 0.1;

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr
MTCDepalletizingTaskNode::getNodeBaseInterface()
{
  return node_->get_node_base_interface();
}

MTCDepalletizingTaskNode::MTCDepalletizingTaskNode(const rclcpp::NodeOptions & options)
: node_{std::make_shared<rclcpp::Node>("mtc_dual_depalletizing_task_node", options)}
{
}

void MTCDepalletizingTaskNode::setupPlanningScene()
{
  addPalletObjects();  // Setup the pallet objects in the planning scene
}

void MTCDepalletizingTaskNode::addPalletObjects()
{
  moveit::planning_interface::PlanningSceneInterface psi;

  for (int k = 0; k < 2; k++)
  {
    for (int j = 0; j < 2; j++)
    {
      for (int i = 0; i < 2; i++)
      {
        moveit_msgs::msg::CollisionObject pallet_object;
        pallet_object.header.frame_id = "world";
        pallet_object.id = "pallet_" + std::to_string(4 * k + 2 * j + i);

        shape_msgs::msg::SolidPrimitive primitive;
        primitive.type = primitive.BOX;
        primitive.dimensions.resize(3);
        primitive.dimensions[primitive.BOX_X] = PALLET_SIZE;
        primitive.dimensions[primitive.BOX_Y] = PALLET_SIZE;
        primitive.dimensions[primitive.BOX_Z] = PALLET_SIZE;

        geometry_msgs::msg::Pose pose;
        pose.orientation.w = 1.0;
        pose.position.x = 0.3 + i * PALLET_DISTANCE + 3.0;
        pose.position.y = -0.1 + j * PALLET_DISTANCE;
        pose.position.z = 0.2 - k * PALLET_DISTANCE;

        pallet_object.primitives.push_back(primitive);
        pallet_object.primitive_poses.push_back(pose);
        pallet_object.operation = pallet_object.ADD;

        psi.applyCollisionObject(pallet_object);
        std::this_thread::sleep_for(std::chrono::milliseconds(200));  // Delay for visualization
      }
    }
  }
}

void MTCDepalletizingTaskNode::attachObject(const std::string & object_id)
{
  std::string frame_id_prefix = (node_->get_namespace() == std::string("/")) ? std::string("") : std::string(node_->get_namespace()) + "/";
  moveit_msgs::msg::AttachedCollisionObject attached_object;
  attached_object.link_name = frame_id_prefix + "flange";
  attached_object.object.id = object_id;
  attached_object.object.operation = attached_object.object.REMOVE;

  moveit::planning_interface::PlanningSceneInterface psi;
  psi.applyAttachedCollisionObject(attached_object);
}

void MTCDepalletizingTaskNode::detachObject(const std::string & object_id)
{
  std::string frame_id_prefix = (node_->get_namespace() == std::string("/")) ? std::string("") : std::string(node_->get_namespace()) + "/";
  moveit_msgs::msg::AttachedCollisionObject attached_object;
  attached_object.link_name = frame_id_prefix + "flange";
  attached_object.object.id = object_id;
  attached_object.object.operation = attached_object.object.REMOVE;

  moveit::planning_interface::PlanningSceneInterface psi;
  psi.applyAttachedCollisionObject(attached_object);
  psi.removeCollisionObjects({object_id});
}

mtc::Task MTCDepalletizingTaskNode::createTask()
{
  mtc::Task task;
  task.stages()->setName("depalletizing task");
  task.loadRobotModel(node_);

  const auto & arm_group_name = "manipulator";
  const auto & eef_frame = "flange";

  task.setProperty("group", arm_group_name);
  task.setProperty("ik_frame", eef_frame);

  auto current_state_stage = std::make_unique<mtc::stages::CurrentState>("current state");
  task.add(std::move(current_state_stage));

  auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_, "ompl", "");
  sampling_planner->setTimeout(30.0);
  sampling_planner->setMaxVelocityScalingFactor(0.2);
  auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();
  auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
  cartesian_planner->setTimeout(30.0);
  cartesian_planner->setMaxVelocityScalingFactor(0.2);
  cartesian_planner->setMaxAccelerationScalingFactor(0.2);
  cartesian_planner->setMinFraction(.02);

  for (int i = 0; i < 20; i++)
  {
    // Define start
    auto start = std::make_unique<mtc::stages::MoveTo>("start_" + std::to_string(i), sampling_planner);
    start->properties().set("marker_ns", "start_" + std::to_string(i));
    start->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});

    // Define the joint positions for the goal
    std::map<std::string, double> start_joint_positions = {{"test2_joint_1", 0.0}, {"test2_joint_2", 0.3},
                                                          {"test2_joint_3", 0.0}, {"test2_joint_4", 0.0},
                                                          {"test2_joint_5", 0.0}, {"test2_joint_6", 0.0}};
    start->setGoal(start_joint_positions);
    task.add(std::move(start));

    // Define end
    auto end = std::make_unique<mtc::stages::MoveTo>("end_" + std::to_string(i), sampling_planner);
    end->properties().set("marker_ns", "end_" + std::to_string(i));
    end->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
    // Define the joint positions for the goal
    std::map<std::string, double> end_joint_positions = {{"test2_joint_1", -2.8}, {"test2_joint_2", 0.0},
                                                          {"test2_joint_3", 0.0}, {"test2_joint_4", 0.0},
                                                          {"test2_joint_5", 0.0}, {"test2_joint_6", 0.0}};
    end->setGoal(end_joint_positions);
    task.add(std::move(end));
  }

  return task;
}

bool MTCDepalletizingTaskNode::doTask(mtc::Task & task)
{
  if (task.stages()->numChildren() == 0)
  {
    RCLCPP_ERROR_STREAM(LOGGER, "No task set.");
    return false;
  }

  try
  {
    task.init();
  }
  catch (mtc::InitStageException & e)
  {
    RCLCPP_ERROR_STREAM(LOGGER, e);
    return false;
  }

  if (!task.plan(1000))
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Task planning failed");
    return false;
  }
  task.introspection().publishSolution(*task.solutions().front());

  auto result = task.execute(*task.solutions().front());
  if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Task execution failed");
    return false;
  }

  return true;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);

  auto mtc_depalletizing_task_node = std::make_shared<MTCDepalletizingTaskNode>(options);
  rclcpp::executors::MultiThreadedExecutor executor;

  auto spin_thread = std::make_unique<std::thread>(
    [&executor, &mtc_depalletizing_task_node]()
    {
      executor.add_node(mtc_depalletizing_task_node->getNodeBaseInterface());
      executor.spin();
      executor.remove_node(mtc_depalletizing_task_node->getNodeBaseInterface());
    });

  mtc_depalletizing_task_node->setupPlanningScene();
  mtc::Task task = mtc_depalletizing_task_node->createTask();
  while (!mtc_depalletizing_task_node->doTask(task))
  {
  }
  spin_thread->join();
  rclcpp::shutdown();
  return 0;
}
