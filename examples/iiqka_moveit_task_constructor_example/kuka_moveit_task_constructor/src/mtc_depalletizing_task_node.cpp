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

#include "kuka_moveit_task_constructor/mtc_depalletizing_task_node.hpp"

static const rclcpp::Logger LOGGER = rclcpp::get_logger("mtc_depalletizing_task_node");

double PALLET_SIZE = 0.097;
double PALLET_DISTANCE = 0.1;

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr
MTCDepalletizingTaskNode::getNodeBaseInterface()
{
  return node_->get_node_base_interface();
}

MTCDepalletizingTaskNode::MTCDepalletizingTaskNode(const rclcpp::NodeOptions & options)
: node_{std::make_shared<rclcpp::Node>("mtc_depalletizing_task_node", options)}
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
        pose.position.x = 0.3 + i * PALLET_DISTANCE;
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
  auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();
  auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
  cartesian_planner->setMaxVelocityScalingFactor(0.2);
  cartesian_planner->setMaxAccelerationScalingFactor(0.2);
  cartesian_planner->setMinFraction(.02);

  // Define start
  auto start = std::make_unique<mtc::stages::MoveTo>("start", interpolation_planner);
  start->properties().set("marker_ns", "start");
  start->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});

  // Define the joint positions for the goal
  std::map<std::string, double> start_joint_positions = {{"test1_joint_1", 0.0}, {"test1_joint_2", -1.57},
                                                         {"test1_joint_3", 0.0}, {"test1_joint_4", 0.0},
                                                         {"test1_joint_5", 0.0}, {"test1_joint_6", 0.0}};
  start->setGoal(start_joint_positions);
  task.add(std::move(start));

  // Loop through every item
  for (int k = 0; k < 2; k++)
  {
    for (int j = 0; j < 2; j++)
    {
      for (int i = 0; i < 2; i++)
      {
        // Define Approach Stage
        auto approach_stage = std::make_unique<mtc::stages::MoveTo>(
          "approach_" + std::to_string(4 * k + 2 * j + i), sampling_planner);
        approach_stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
        approach_stage->properties().set(
          "marker_ns", "approach_" + std::to_string(4 * k + 2 * j + i));
        geometry_msgs::msg::PoseStamped approach_pose;
        approach_pose.header.frame_id = "world";
        approach_pose.pose.position.x = 0.3 + PALLET_DISTANCE * i;
        approach_pose.pose.position.y = -0.1 + PALLET_DISTANCE * j;
        approach_pose.pose.position.z = 0.4;
        approach_pose.pose.orientation.x = 0.0;
        approach_pose.pose.orientation.y = 1.0;
        approach_pose.pose.orientation.z = 0.0;
        approach_pose.pose.orientation.w = 0.0;
        approach_stage->setGoal(approach_pose);
        task.add(std::move(approach_stage));

        // Define Pick Stage
        auto pick_stage = std::make_unique<mtc::stages::MoveTo>(
          "pick_" + std::to_string(4 * k + 2 * j + i), cartesian_planner);
        pick_stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
        pick_stage->properties().set("marker_ns", "pick_" + std::to_string(4 * k + 2 * j + i));
        geometry_msgs::msg::PoseStamped pick_pose;
        pick_pose.header.frame_id = "world";
        pick_pose.pose.position.x = 0.3 + PALLET_DISTANCE * i;
        pick_pose.pose.position.y = -0.1 + PALLET_DISTANCE * j;
        pick_pose.pose.position.z = 0.4 - PALLET_DISTANCE * (k + 1) - 0.02;
        pick_pose.pose.orientation.x = 0.0;
        pick_pose.pose.orientation.y = 1.0;
        pick_pose.pose.orientation.z = 0.0;
        pick_pose.pose.orientation.w = 0.0;
        pick_stage->setGoal(pick_pose);
        task.add(std::move(pick_stage));

        // Attach Object Stage
        auto attach_stage = std::make_unique<mtc::stages::ModifyPlanningScene>(
          "attach object_" + std::to_string(4 * k + 2 * j + i));
        attach_stage->attachObject("pallet_" + std::to_string(4 * k + 2 * j + i), eef_frame);
        task.add(std::move(attach_stage));

        // Define Lift Stage
        auto lift_stage = std::make_unique<mtc::stages::MoveTo>(
          "lift_" + std::to_string(4 * k + 2 * j + i), cartesian_planner);
        lift_stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
        lift_stage->properties().set("marker_ns", "lift_" + std::to_string(4 * k + 2 * j + i));
        geometry_msgs::msg::PoseStamped lift_pose;
        lift_pose.header.frame_id = "world";
        lift_pose.pose.position.x = 0.3 + PALLET_DISTANCE * i;
        lift_pose.pose.position.y = -0.1 + PALLET_DISTANCE * j;
        lift_pose.pose.position.z = 0.4;
        lift_pose.pose.orientation.x = 0.0;
        lift_pose.pose.orientation.y = 1.0;
        lift_pose.pose.orientation.z = 0.0;
        lift_pose.pose.orientation.w = 0.0;
        lift_stage->setGoal(lift_pose);
        task.add(std::move(lift_stage));

        // Define Move Stage
        auto move_stage = std::make_unique<mtc::stages::MoveTo>(
          "move_" + std::to_string(4 * k + 2 * j + i), sampling_planner);
        move_stage->setTimeout(10.0);
        move_stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
        move_stage->properties().set("marker_ns", "move_" + std::to_string(4 * k + 2 * j + i));
        geometry_msgs::msg::PoseStamped move_pose;
        move_pose.header.frame_id = "world";
        move_pose.pose.position.x = -0.3 - PALLET_DISTANCE * i;
        move_pose.pose.position.y = 0.1 - PALLET_DISTANCE * j;
        move_pose.pose.position.z = 0.4;
        move_pose.pose.orientation.x = 1.0;
        move_pose.pose.orientation.y = 0.0;
        move_pose.pose.orientation.z = 0.0;
        move_pose.pose.orientation.w = 0.0;
        move_stage->setGoal(move_pose);
        task.add(std::move(move_stage));

        // Define Place Stage
        auto place_stage = std::make_unique<mtc::stages::MoveRelative>(
          "place_" + std::to_string(4 * k + 2 * j + i), cartesian_planner);
        place_stage->properties().set("marker_ns", "place_" + std::to_string(4 * k + 2 * j + i));
        place_stage->properties().set("link", eef_frame);
        place_stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
        geometry_msgs::msg::Vector3Stamped vec_place;
        vec_place.header.frame_id = "world";
        vec_place.vector.z = -(2 - k) * PALLET_DISTANCE - 0.02;
        place_stage->setDirection(vec_place);
        task.add(std::move(place_stage));

        // Detach Object Stage
        auto detach_stage = std::make_unique<mtc::stages::ModifyPlanningScene>(
          "detach object_" + std::to_string(4 * k + 2 * j + i));
        detach_stage->detachObject("pallet_" + std::to_string(4 * k + 2 * j + i), eef_frame);
        task.add(std::move(detach_stage));

        // Define Lift2 Stage
        auto lift2_stage = std::make_unique<mtc::stages::MoveRelative>(
          "lift2_" + std::to_string(4 * k + 2 * j + i), cartesian_planner);
        lift2_stage->properties().set("marker_ns", "lift2_" + std::to_string(4 * k + 2 * j + i));
        lift2_stage->properties().set("link", eef_frame);
        lift2_stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
        geometry_msgs::msg::Vector3Stamped vec_lift2;
        vec_lift2.header.frame_id = "world";
        vec_lift2.vector.z = (2 - k) * PALLET_DISTANCE + 0.02;
        lift2_stage->setDirection(vec_lift2);
        task.add(std::move(lift2_stage));
      }
    }
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
