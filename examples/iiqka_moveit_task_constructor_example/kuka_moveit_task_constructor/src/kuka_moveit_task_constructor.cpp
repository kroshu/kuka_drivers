#include "kuka_moveit_task_constructor/kuka_moveit_task_constructor.hpp"

static const rclcpp::Logger LOGGER = rclcpp::get_logger("kuka_moveit_task_constructor");

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr MTCTaskNode::getNodeBaseInterface()
{
  return node_->get_node_base_interface();
}

MTCTaskNode::MTCTaskNode(const rclcpp::NodeOptions& options)
  : node_{ std::make_shared<rclcpp::Node>("mtc_node", options) }
{
}

void MTCTaskNode::setupPlanningScene()
{
  moveit_msgs::msg::CollisionObject object;
  object.id = "object";
  object.header.frame_id = "world";
  object.primitives.resize(1);
  object.primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
  object.primitives[0].dimensions = { 0.1, 0.02 };

  geometry_msgs::msg::Pose pose;
  pose.position.x = 0.5;
  pose.position.y = -0.25;
  pose.orientation.w = 1.0;
  object.pose = pose;

  moveit::planning_interface::PlanningSceneInterface psi;
  psi.applyCollisionObject(object);
}

void MTCTaskNode::doTask()
{
  task_ = createTask();

  try
  {
    task_.init();
  }
  catch (mtc::InitStageException& e)
  {
    RCLCPP_ERROR_STREAM(LOGGER, e);
    return;
  }

  if (!task_.plan(5))
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Task planning failed");
    return;
  }
  task_.introspection().publishSolution(*task_.solutions().front());

  auto result = task_.execute(*task_.solutions().front());
  if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Task execution failed");
    return;
  }

  return;
}

mtc::Task MTCTaskNode::createTask()
{
  mtc::Task task;
  task.stages()->setName("demo task");
  task.loadRobotModel(node_);

  const auto& arm_group_name = "manipulator";
  const auto& eef_frame = "end_effector";

  // Set task properties
  task.setProperty("group", arm_group_name);
  task.setProperty("ik_frame", eef_frame);

  auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current");
  mtc::Stage* current_state_ptr = stage_state_current.get();
  task.add(std::move(stage_state_current));

  auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_);
  auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();

  auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
  cartesian_planner->setMaxVelocityScalingFactor(0.2);
  cartesian_planner->setMaxAccelerationScalingFactor(0.2);
  cartesian_planner->setStepSize(.01);
  cartesian_planner->setMinFraction(0.2);

  // Set stage1 as MoveRelative
  auto stage1 = std::make_unique<mtc::stages::MoveRelative>("move1", cartesian_planner);
  stage1->properties().set("marker_ns", "move1");
  stage1->properties().set("link", eef_frame);
  stage1->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
  stage1->setMinMaxDistance(0.0, 0.2);
  geometry_msgs::msg::Vector3Stamped vec;
  vec.header.frame_id = eef_frame;
  vec.vector.z = 1.0;
  stage1->setDirection(vec);
  task.add(std::move(stage1));

  // Set stage2 as MoveTo with cartesian goal
  auto stage2 = std::make_unique<mtc::stages::MoveTo>("move2", cartesian_planner);
  stage2->properties().set("marker_ns", "move2");
  stage2->properties().set("link", eef_frame);
  stage2->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });

  // Set the goal pose for stage2
  geometry_msgs::msg::PoseStamped goalPose;
  goalPose.header.frame_id = "world";
  goalPose.pose.position.x = 0.4;
  goalPose.pose.position.y = 0.2;
  goalPose.pose.position.z = 0.3;
  goalPose.pose.orientation.x = 1.0;
  goalPose.pose.orientation.y = 1.0;
  goalPose.pose.orientation.z = 1.0;
  goalPose.pose.orientation.w = 1.0;
  stage2->setGoal(goalPose);
  task.add(std::move(stage2));

  // Set stage3 as MoveTo with joint positions
  auto stage3 = std::make_unique<mtc::stages::MoveTo>("move3", interpolation_planner);
  stage3->properties().set("marker_ns", "move3");
  stage3->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });

  // Define the joint positions for the goal
  std::map<std::string, double> joint_positions = {
    {"joint_1", 0.0},
    {"joint_2", -0.5},
    {"joint_3", 1.0},
    {"joint_4", -1.5},
    {"joint_5", 1.0},
    {"joint_6", 0.5}
  };
  stage3->setGoal(joint_positions);
  task.add(std::move(stage3));

  return task;
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);

  auto mtc_task_node = std::make_shared<MTCTaskNode>(options);
  rclcpp::executors::MultiThreadedExecutor executor;

  auto spin_thread = std::make_unique<std::thread>([&executor, &mtc_task_node]() {
    executor.add_node(mtc_task_node->getNodeBaseInterface());
    executor.spin();
    executor.remove_node(mtc_task_node->getNodeBaseInterface());
  });

  mtc_task_node->setupPlanningScene();
  mtc_task_node->doTask();

  spin_thread->join();
  rclcpp::shutdown();
  return 0;
}
