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


//******************************************************************************
// Declare global objects
// Create move group planning interfaces
static const std::string PLANNING_GROUP_ARM1_2 = "both_arms";
moveit::planning_interface::MoveGroupInterface* move_group_arm1_2;
const moveit::core::JointModelGroup* joint_model_arm1_2;

// Visualization
moveit_visual_tools::MoveItVisualTools* visual_tools;

void plan_execute_arm_move(moveit::planning_interface::MoveGroupInterface* move_group)
{
  // Create plan object
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  move_group->setMaxVelocityScalingFactor(0.1);
  move_group->setMaxAccelerationScalingFactor(0.1);
  move_group->setPlanningTime(0.1);
  move_group->setNumPlanningAttempts(10);
  
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
  const std::string node_name = "stand_tasks";
  ros::init(argc, argv, node_name);
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  //****************************************************************************
  // Set up planning interface
  // Set up move group planning interfaces
  move_group_arm1_2 = new moveit::planning_interface::MoveGroupInterface(PLANNING_GROUP_ARM1_2);

  // Set up planning scene to add/remove collision objects in world
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // Create pointers to planning groups
  joint_model_arm1_2 = move_group_arm1_2->getCurrentState()->getJointModelGroup(PLANNING_GROUP_ARM1_2);

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
  visual_tools->publishText(text_pose, "Coffee Demo", rvt::WHITE, rvt::XLARGE);
  visual_tools->trigger();

  // User input to start demo
  visual_tools->prompt("Press 'next' in the RvizVisualToolsGui window to begin");
  move_group_arm1_2->setNamedTarget("rest");
  plan_execute_arm_move(move_group_arm1_2);

  // Load list of moves from ROS Parameter Server
  std::vector<std::string> moves_list;
  node_handle.getParam("/stand_moves", moves_list);

  for (std::vector<std::string>::iterator it = std::begin(moves_list); it != std::end(moves_list); ++it)
  {
    // String parsing
    std::string move_name = *it;
    std::string move_group = move_name.substr(0, move_name.find("/"));
    std::string pose_name = move_name.substr(move_name.find("/")+1);
    ROS_INFO("Move found: %s, move_group: %s, pose_name: %s", move_name.c_str(), move_group.c_str(), pose_name.c_str());

    // Get pose data
    std::vector<double> pose_data;
    node_handle.getParam("/"+move_name, pose_data);

    // Do appropriate move
    std::vector<double> joint_group_positions = pose_data;
    move_group_arm1_2->setJointValueTarget(joint_group_positions);
    plan_execute_arm_move(move_group_arm1_2);
    
  }

  move_group_arm1_2->setNamedTarget("rest");
  plan_execute_arm_move(move_group_arm1_2);
  ros::shutdown();
  return 0;
}