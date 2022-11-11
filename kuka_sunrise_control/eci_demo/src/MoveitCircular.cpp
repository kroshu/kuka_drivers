#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/msg/collision_object.hpp>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include <math.h>

int main(int argc, char * argv[])
{
  //Setup

  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "moveit_circle",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("moveit_circle");

  //Create Planning group:
  static const std::string PLANNING_GROUP = "iisy_arm";

  // Next step goes here
  // Create the MoveIt MoveGroup Interface
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, PLANNING_GROUP);

  // Create Planning Sceen Interface, witch is for adding collision boxes
  using moveit::planning_interface::PlanningSceneInterface;
  auto planning_scene_interface = PlanningSceneInterface();

  // This is the official reason for this line, I don't understand it yet:
  // Raw pointers are frequently used to refer to the planning group for improved performance.
  // const moveit::core::JointModelGroup * joint_modell_group =
  //   move_group_interface.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  //End Setup


  // Visualization setup

  // Create MoveIt Visual Tools, whitch is for adding visual boxes to rviz.
  namespace rvt = rviz_visual_tools;
  // moveit_visual_tools::MoveItVisualTools visual_tools(node, "LBR3R760_iisy_fixedbase",
  //   "move_group_tutorial",
  //   move_group_interface.getRobotModel());
  // visual_tools.deleteAllMarkers();
/* Remote control is an introspection tool that allows users to step through a high level script */
/* via buttons and keyboard shortcuts in RViz */
  // visual_tools.trigger();

  // End Visualization setup

  // Collision Objects define

  moveit_msgs::msg::CollisionObject collision_object;
  collision_object.header.frame_id = move_group_interface.getPlanningFrame();
  collision_object.id = "box1";
  shape_msgs::msg::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[primitive.BOX_X] = 0.4;
  primitive.dimensions[primitive.BOX_Y] = 0.4;
  primitive.dimensions[primitive.BOX_Z] = 1.2;

  // Define a pose for the box (specified relative to frame_id).
  // Need to check what is frame_id, and what is the center of the box
  geometry_msgs::msg::Pose stand_pose;
  stand_pose.orientation.w = 1.0;
  stand_pose.position.x = - 0.2;
  stand_pose.position.y = - 0.2;
  stand_pose.position.z = -1.2;

  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(stand_pose);
  collision_object.operation = collision_object.ADD;


  std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
  collision_objects.push_back(collision_object);

  //RCLCPP_INFO(logger, "Add a Robotarm stand object into the world");
  planning_scene_interface.addCollisionObjects(collision_objects);



  // visual_tools.trigger();
  // End Collision Objects define


  std::vector<geometry_msgs::msg::Pose> waypoints;

  moveit_msgs::msg::RobotTrajectory trajectory;
  //Start pose;
  geometry_msgs::msg::Pose msg;
  msg.orientation.x = 0.0;
  msg.orientation.y = 0.0;
  msg.orientation.z = 0.0;
  msg.orientation.w = 1.0;
  msg.position.x = 0.6;
  msg.position.y = 0.1;
  msg.position.z = 0.4;
  waypoints.push_back(msg);


  for (int i = 1; i < 63; i++) {
    msg.position.y = 0.0 + sin(0.1 * i) / 5;
    msg.position.z = 0.2 + cos(0.1 * i) / 5;
    waypoints.push_back(msg);
  }

  double fraction = move_group_interface.computeCartesianPath(waypoints, 0.005, 0.0, trajectory);

  if (fraction < 0.1) {RCLCPP_ERROR(logger, "Planning failed!");} else {
    move_group_interface.execute(trajectory);
  }

  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}
