#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_state/conversions.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

// For HTMs
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Vector3.h>
#include <unordered_map>
// For quaternions
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// for cacheing
#include <unordered_map>
#include <fstream> 
#include <sstream>
#include <cstdint>
#include <yaml-cpp/yaml.h>


#define VEL_SCALE 0.15
#define ACCEL_SCALE 0.15
#define PLANNING_TIME 5
#define PLAN_ATTEMPTS 30

//******************************************************************************
// Declare global objects
// Create move group planning interfaces
static const std::string PLANNING_GROUP_ARM1 = "arm1";
static const std::string PLANNING_GROUP_GRIPPER1 = "gripper1";
static const std::string PLANNING_GROUP_ARM2 = "arm2";
static const std::string PLANNING_GROUP_GRIPPER2 = "gripper2";
static const std::string PLANNING_GROUP_ARM1_2 = "arm1_2";

moveit::planning_interface::MoveGroupInterface* move_group_arm1;
moveit::planning_interface::MoveGroupInterface* move_group_gripper1;
moveit::planning_interface::MoveGroupInterface* move_group_arm2;
moveit::planning_interface::MoveGroupInterface* move_group_gripper2;
moveit::planning_interface::MoveGroupInterface* move_group_arm1_2;
moveit::planning_interface::MoveGroupInterface* move_group;

// Pointers to move groups
const moveit::core::JointModelGroup* joint_model_arm1;
const moveit::core::JointModelGroup* joint_model_gripper1;
const moveit::core::JointModelGroup* joint_model_arm2;
const moveit::core::JointModelGroup* joint_model_gripper2;
const moveit::core::JointModelGroup* joint_model_arm1_2;

// Double pointers for current move
moveit::planning_interface::MoveGroupInterface** current_move_group;
const moveit::core::JointModelGroup** current_joint_model;

// Visualization
moveit_visual_tools::MoveItVisualTools* visual_tools;

int main(int argc, char** argv)
{
  // Setup node and start AsyncSpinner
  const std::string node_name = "tea_demo";
  ros::init(argc, argv, node_name);
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  //****************************************************************************
  // Set up planning interface
  // Set up move group planning interfaces
  move_group_arm1 = new moveit::planning_interface::MoveGroupInterface(PLANNING_GROUP_ARM1);
  move_group_gripper1 = new moveit::planning_interface::MoveGroupInterface(PLANNING_GROUP_GRIPPER1);
  move_group_arm2 = new moveit::planning_interface::MoveGroupInterface(PLANNING_GROUP_ARM2);
  move_group_gripper2 = new moveit::planning_interface::MoveGroupInterface(PLANNING_GROUP_GRIPPER2);
  move_group_arm1_2 = new moveit::planning_interface::MoveGroupInterface(PLANNING_GROUP_ARM1_2);

  // Set up planning scene to add/remove collision objects in world
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // Create pointers to planning groups
  joint_model_arm1 = move_group_arm1->getCurrentState()->getJointModelGroup(PLANNING_GROUP_ARM1);
  joint_model_gripper1 = move_group_gripper1->getCurrentState()->getJointModelGroup(PLANNING_GROUP_GRIPPER1);
  joint_model_arm2 = move_group_arm2->getCurrentState()->getJointModelGroup(PLANNING_GROUP_ARM2);
  joint_model_gripper2 = move_group_gripper2->getCurrentState()->getJointModelGroup(PLANNING_GROUP_GRIPPER2);
  joint_model_arm1_2 = move_group_arm1_2->getCurrentState()->getJointModelGroup(PLANNING_GROUP_ARM1_2);

  // Print reference information
  ROS_INFO("Arm 1 planning frame: %s", move_group_arm1->getPlanningFrame().c_str());
  ROS_INFO("Arm 1 end effector link: %s", move_group_arm1->getEndEffectorLink().c_str());
  ROS_INFO("Arm 2 planning frame: %s", move_group_arm2->getPlanningFrame().c_str());
  ROS_INFO("Arm 2 end effector link: %s", move_group_arm2->getEndEffectorLink().c_str());

  //****************************************************************************
  // Set up visualization
  namespace rvt = rviz_visual_tools;
  visual_tools = new moveit_visual_tools::MoveItVisualTools("world");
  visual_tools->deleteAllMarkers();

  // Enable RViz remote control
  visual_tools->loadRemoteControl();

  // Batch publish RViz commands
  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = 1.0;
  visual_tools->publishText(text_pose, "Tea Table Demo", rvt::WHITE, rvt::XLARGE);
  visual_tools->trigger();

  // User input to start demo
  visual_tools->prompt("Press 'next' in the RvizVisualToolsGui window to begin");
  move_group = move_group_arm1_2;
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  move_group->setPlanningPipelineId("ompl");
  move_group->setMaxVelocityScalingFactor(VEL_SCALE);
  move_group->setMaxAccelerationScalingFactor(ACCEL_SCALE);
  move_group->setPlanningTime(PLANNING_TIME);
  move_group->setNumPlanningAttempts(PLAN_ATTEMPTS);

  move_group->setStartStateToCurrentState();
  move_group->setNamedTarget("rest");
  move_group->plan(my_plan);
  move_group->execute(my_plan);

  std::vector<std::string> moves_list;
  node_handle.getParam("/coffee_demo_moves", moves_list);

  // Iterate through moves and do
  for (std::vector<std::string>::iterator it = std::begin(moves_list); it != std::end(moves_list); ++it)
  {
    // String parsing
    std::string move_name = *it;
    std::string control_group = move_name.substr(0, move_name.find("/"));
    std::string pose_name = move_name.substr(move_name.find("/")+1);
    // ROS_INFO("Move found: %s, control_group: %s, pose_name: %s", move_name.c_str(), control_group.c_str(), pose_name.c_str());

    // Get pose data
    std::vector<double> pose_data;
    node_handle.getParam("/"+move_name, pose_data);

    if (control_group == "sleep") {
      ros::Duration(pose_data.at(0)).sleep();
      continue;
    }

    if (control_group == "arm1_2"){
      move_group = move_group_arm1_2;
    } else if (control_group == "arm1") {
      move_group = move_group_arm1;
    } else if (control_group == "arm2") {
      move_group = move_group_arm2;
    } else if (control_group == "gripper1") {
      move_group = move_group_gripper1;
    } else if (control_group == "gripper2") {
      move_group = move_group_gripper2;
    } 
    
    bool success;
    move_group->setPlanningPipelineId("ompl");
    move_group->setMaxVelocityScalingFactor(VEL_SCALE);
    move_group->setMaxAccelerationScalingFactor(ACCEL_SCALE);
    move_group->setPlanningTime(PLANNING_TIME);
    move_group->setNumPlanningAttempts(PLAN_ATTEMPTS);
    move_group->setStartStateToCurrentState();
    move_group->setJointValueTarget(pose_data);
    success = (move_group->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO("Visualizing plan %s", success ? "" : "FAILED");
    visual_tools->trigger();
    if (success) {
      visual_tools->prompt("Press 'next' to execute plan");
      move_group->execute(my_plan);
    }
    // ros::Duration(5.0).sleep();
  }

  move_group = move_group_arm1_2;
  move_group->setStartStateToCurrentState();
  move_group->setNamedTarget("rest");
  move_group->plan(my_plan);
  move_group->execute(my_plan);
  

  // Write to a yaml file with helper functions.
  ros::shutdown();
  return 0;
}