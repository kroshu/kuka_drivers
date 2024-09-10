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
  addPalletObjects();  // Setup the pallet objects in the planning scene
}

void MTCTaskNode::addPalletObjects()
{
  moveit::planning_interface::PlanningSceneInterface psi;
  
  for (int k = 0; k < 1; k++)
  {
    for (int j = 0; j < 1; j++)
    {
      for (int i = 0; i < 1; i++)
      {
        moveit_msgs::msg::CollisionObject pallet_object;
        pallet_object.header.frame_id = "world";
        pallet_object.id = "pallet_" + std::to_string(9 * k + 3 * j + i);

        shape_msgs::msg::SolidPrimitive primitive;
        primitive.type = primitive.BOX;
        primitive.dimensions.resize(3);
        primitive.dimensions[primitive.BOX_X] = 0.097;
        primitive.dimensions[primitive.BOX_Y] = 0.097;
        primitive.dimensions[primitive.BOX_Z] = 0.097;

        geometry_msgs::msg::Pose pose;
        pose.orientation.w = 1.0;
        pose.position.x = 0.3 + i * 0.1;
        pose.position.y = -0.1 + j * 0.1;
        pose.position.z = 0.3 - 0.1 * k;

        pallet_object.primitives.push_back(primitive);
        pallet_object.primitive_poses.push_back(pose);
        pallet_object.operation = pallet_object.ADD;

        psi.applyCollisionObject(pallet_object);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));  // Delay for visualization
      }
    }
  }
}

void MTCTaskNode::attachObject(const std::string& object_id)
{
  moveit_msgs::msg::AttachedCollisionObject attached_object;
  attached_object.link_name = "flange";
  attached_object.object.id = object_id;
  attached_object.object.operation = attached_object.object.REMOVE;

  moveit::planning_interface::PlanningSceneInterface psi;
  psi.applyAttachedCollisionObject(attached_object);
}

void MTCTaskNode::detachAndRemoveObject(const std::string& object_id)
{
  moveit_msgs::msg::AttachedCollisionObject attached_object;
  attached_object.link_name = "flange";
  attached_object.object.id = object_id;
  attached_object.object.operation = attached_object.object.REMOVE;

  moveit::planning_interface::PlanningSceneInterface psi;
  psi.applyAttachedCollisionObject(attached_object);
  psi.removeCollisionObjects({object_id});
}

mtc::Task MTCTaskNode::createDepalletizingTask()
{
  mtc::Task task;
  task.stages()->setName("depalletizing task");
  task.loadRobotModel(node_);

  const auto& arm_group_name = "manipulator";
  const auto& eef_frame = "flange";

  task.setProperty("group", arm_group_name);
  task.setProperty("ik_frame", eef_frame);

  auto current_state_stage = std::make_unique<mtc::stages::CurrentState>("current state");
  task.add(std::move(current_state_stage));

  auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_);
  sampling_planner->setTimeout(30.0);
  auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();
  auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
  cartesian_planner->setMaxVelocityScalingFactor(0.2);
  cartesian_planner->setMaxAccelerationScalingFactor(0.2);
  cartesian_planner->setStepSize(.001);
  cartesian_planner->setMinFraction(.02);

  // Define start
  auto start = std::make_unique<mtc::stages::MoveTo>("start", interpolation_planner);
  start->properties().set("marker_ns", "start");
  start->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });

  // Define the joint positions for the goal
  std::map<std::string, double> start_joint_positions = {
    {"joint_1", 0.0},
    {"joint_2", -1.07},
    {"joint_3", 0.0},
    {"joint_4", 0.0},
    {"joint_5", 0.0},
    {"joint_6", 0.0}
  };
  start->setGoal(start_joint_positions);
  task.add(std::move(start));

  // Define Pick Stage
  auto pick_stage = std::make_unique<mtc::stages::MoveTo>("pick", sampling_planner);
  pick_stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
  pick_stage->properties().set("marker_ns", "pick");
  geometry_msgs::msg::PoseStamped pick_pose;
  pick_pose.header.frame_id = "world";
  pick_pose.pose.position.x = 0.3;
  pick_pose.pose.position.y = -0.1;
  pick_pose.pose.position.z = 0.4;
  pick_pose.pose.orientation.x = 0.0;
  pick_pose.pose.orientation.y = 1.0;
  pick_pose.pose.orientation.z = 0.0;
  pick_pose.pose.orientation.w = 0.0;
  pick_stage->setGoal(pick_pose);
  task.add(std::move(pick_stage));

  // Attach Object Stage
  auto attach_stage = std::make_unique<mtc::stages::ModifyPlanningScene>("attach object");
  attach_stage->attachObject("pallet_0", eef_frame);
  task.add(std::move(attach_stage));

  // Define Lift Stage
  auto lift_stage = std::make_unique<mtc::stages::MoveTo>("lift", cartesian_planner);
  lift_stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
  lift_stage->properties().set("marker_ns", "lift");
  geometry_msgs::msg::PoseStamped lift_pose;
  lift_pose.header.frame_id = "world";
  lift_pose.pose.position.x = 0.3;
  lift_pose.pose.position.y = -0.1;
  lift_pose.pose.position.z = 0.55;
  lift_pose.pose.orientation.x = 0.0;
  lift_pose.pose.orientation.y = 1.0;
  lift_pose.pose.orientation.z = 0.0;
  lift_pose.pose.orientation.w = 0.0;
  lift_stage->setGoal(lift_pose);
  task.add(std::move(lift_stage));

  // Define Place Stage
  auto place = std::make_unique<mtc::stages::MoveRelative>("place", sampling_planner);
  place->properties().set("marker_ns", "place");
  place->properties().set("link", eef_frame);
  place->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
  place->setMinMaxDistance(0.0, 0.2);
  geometry_msgs::msg::Vector3Stamped vec_place;
  vec_place.header.frame_id = "world";
  vec_place.vector.x = -0.6;
  //vec_place.vector.y = 0.6;
  place->setDirection(vec_place);
  //task.add(std::move(place));

  // Define Lower Stage
  auto lower = std::make_unique<mtc::stages::MoveRelative>("lower", cartesian_planner);
  lower->properties().set("marker_ns", "lower");
  lower->properties().set("link", eef_frame);
  lower->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
  lower->setMinMaxDistance(0.0, 0.2);
  geometry_msgs::msg::Vector3Stamped vec_lower;
  vec_lower.header.frame_id = "world";
  vec_lower.vector.z = -0.2;
  lower->setDirection(vec_lower);
  task.add(std::move(lower));

  // Detach Object Stage
  auto detach_stage = std::make_unique<mtc::stages::ModifyPlanningScene>("detach object");
  detach_stage->detachObject("pallet_0", eef_frame);
  task.add(std::move(detach_stage));

  return task;
}

bool MTCTaskNode::doTask(mtc::Task& task)
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
  catch (mtc::InitStageException& e)
  {
    RCLCPP_ERROR_STREAM(LOGGER, e);
    return false;
  }

  if (!task.plan(20))
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
  mtc::Task task = mtc_task_node->createDepalletizingTask();
  bool success = mtc_task_node->doTask(task);

  spin_thread->join();
  rclcpp::shutdown();
  return 0;
}
