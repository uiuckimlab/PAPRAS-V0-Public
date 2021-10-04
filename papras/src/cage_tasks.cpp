#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

// For HTMs
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Vector3.h>

// For quaternions
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// End-effector links
#define EEF1 "robot1/end_effector_link"
#define EEF2 "robot2/end_effector_link"
#define EEF3 "robot3/end_effector_link"
#define EEF4 "robot4/end_effector_link"

// Planning parameters
#define VEL_SCALE 0.1
#define ACCEL_SCALE 0.1
#define PLANNING_TIME 1.0
#define NUM_PLANS 10

// The circle constant tau = 2*pi. One tau is one rotation in radians.
const double tau = 2 * M_PI;

//******************************************************************************
// Declare global objects
// Create move group planning interfaces
static const std::string PLANNING_GROUP_ARM1_2_3_4 = "arm1_2_3_4";

moveit::planning_interface::MoveGroupInterface* move_group_arm1_2_3_4;

// Pointers to move groups
const moveit::core::JointModelGroup* joint_model_arm1_2_3_4;

// Visualization
moveit_visual_tools::MoveItVisualTools* visual_tools;

void plan_execute_arm_move(moveit::planning_interface::MoveGroupInterface* move_group)
{
  // Create plan object
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  move_group->setMaxVelocityScalingFactor(VEL_SCALE);
  move_group->setMaxAccelerationScalingFactor(ACCEL_SCALE);
  move_group->setPlanningTime(PLANNING_TIME);
  move_group->setNumPlanningAttempts(NUM_PLANS);
  
  // Plan and execute on appropriate arm
  bool success;
  success = (move_group->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO("Visualizing plan %s", success ? "" : "FAILED");
  if (success) {
    visual_tools->prompt("Press 'next' to execute plan");
    move_group->execute(my_plan);
  }
}

int main(int argc, char** argv)
{
  // Setup node and start AsyncSpinner
  const std::string node_name = "test_multi";
  ros::init(argc, argv, node_name);
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  //****************************************************************************
  // Set up planning interface
  // Set up move group planning interfaces
  move_group_arm1_2_3_4 = new moveit::planning_interface::MoveGroupInterface(PLANNING_GROUP_ARM1_2_3_4);

  // Set up planning scene to add/remove collision objects in world
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // Create pointers to planning groups
  joint_model_arm1_2_3_4 = move_group_arm1_2_3_4->getCurrentState()->getJointModelGroup(PLANNING_GROUP_ARM1_2_3_4);

  //****************************************************************************
  // Set up visualization
  namespace rvt = rviz_visual_tools;
  visual_tools = new moveit_visual_tools::MoveItVisualTools("world");
  visual_tools->deleteAllMarkers();

  // Enable RViz remote control
  visual_tools->loadRemoteControl();

  // Batch publish RViz commands
  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().x() = 0.61;
  text_pose.translation().y() = 0.70;
  text_pose.translation().z() = 1.50;
  visual_tools->publishText(text_pose, "Cage Demo", rvt::WHITE, rvt::XLARGE);
  visual_tools->trigger();

  // Add collision object to represent dummy (mannequin)
  moveit_msgs::CollisionObject collision_object;
  collision_object.header.frame_id = move_group_arm1_2_3_4->getPlanningFrame();
  
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions[primitive.BOX_X] = 0.25;
  primitive.dimensions[primitive.BOX_Y] = 0.60;
  primitive.dimensions[primitive.BOX_Z] = 1.90;
  
  geometry_msgs::Pose box_pose;
  box_pose.orientation.w = 1.0;
  box_pose.position.x = 0.61;
  box_pose.position.y = 0.70;
  box_pose.position.z = 0.95;

  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(box_pose);
  collision_object.operation = collision_object.ADD;

  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.push_back(collision_object);
  ROS_INFO("Adding mannequin collision object");
  planning_scene_interface.addCollisionObjects(collision_objects);

  // User input to start demo
  visual_tools->prompt("Press 'next' in the RvizVisualToolsGui window to begin");
  move_group_arm1_2_3_4->setNamedTarget("rest");
  plan_execute_arm_move(move_group_arm1_2_3_4);

  // Load list of moves from ROS Parameter Server
  std::vector<std::string> moves_list;
  node_handle.getParam("/cage_moves", moves_list);

  for (std::vector<std::string>::iterator it = std::begin(moves_list); it != std::end(moves_list); ++it)
  {
    // String parsing
    std::string move_name = *it;
    std::string move_group = move_name.substr(0, move_name.find("/"));
    std::string pose_name = move_name.substr(move_name.find("/")+1);
    // ROS_INFO("Move found: %s, control_group: %s, pose_name: %s", move_name.c_str(), control_group.c_str(), pose_name.c_str());

    // Get pose data
    std::vector<double> pose_data;
    node_handle.getParam("/"+move_name, pose_data);

    // Do appropriate move
    if(move_group=="arm1_2_3_4"){
      std::vector<double> joint_group_positions = pose_data;
      move_group_arm1_2_3_4->setJointValueTarget(joint_group_positions);
      plan_execute_arm_move(move_group_arm1_2_3_4);
    }
  }

  move_group_arm1_2_3_4->setNamedTarget("rest");
  plan_execute_arm_move(move_group_arm1_2_3_4);
  ros::shutdown();
  return 0;
}