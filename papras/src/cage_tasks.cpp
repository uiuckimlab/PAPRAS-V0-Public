#include <algorithm>    // std::find
#include <vector>       // std::vector

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
static const std::string PLANNING_GROUP_ARM1_4 = "arm1_4";

moveit::planning_interface::MoveGroupInterface* move_group_arm1_4;

// Pointers to move groups
const moveit::core::JointModelGroup* joint_model_arm1_4;

// Double pointers for current move
moveit::planning_interface::MoveGroupInterface** current_move_group;
const moveit::core::JointModelGroup** current_joint_model;

// Visualization
moveit_visual_tools::MoveItVisualTools* visual_tools;

int main(int argc, char** argv)
{
  // Setup node and start AsyncSpinner
  const std::string node_name = "cage_tasks";
  ros::init(argc, argv, node_name);
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  //****************************************************************************
  // Set up planning interface
  // Set up move group planning interfaces
  move_group_arm1_4 = new moveit::planning_interface::MoveGroupInterface(PLANNING_GROUP_ARM1_4);

  // Set up planning scene to add/remove collision objects in world
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // Create pointers to planning groups
  joint_model_arm1_4 = move_group_arm1_4->getCurrentState()->getJointModelGroup(PLANNING_GROUP_ARM1_4);

  // Print reference information
  ROS_INFO("Arm 1_4 planning frame: %s", move_group_arm1_4->getPlanningFrame().c_str());

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
  visual_tools->publishText(text_pose, "Cage Tasks", rvt::WHITE, rvt::XLARGE);
  visual_tools->trigger();

  // User input to start demo
  visual_tools->prompt("Press 'next' in the RvizVisualToolsGui window to begin");

  // Load list of moves from ROS Parameter Server
  std::vector<std::string> moves_list;
  node_handle.getParam("/cage_moves", moves_list);

  std::vector<std::string> named_pose = {"rest", "init", "down", "home"};

  // Iterate through moves and do
  for (std::vector<std::string>::iterator it = std::begin(moves_list); it != std::end(moves_list); ++it)
  {
    // String parsing
    std::string move_name = *it;
    std::string control_group = move_name.substr(0, move_name.find("/"));
    std::string control_name = control_group.substr(0, control_group.length()-1);
    std::string control_id = control_group.substr(control_group.length()-1);
    std::string pose_name = move_name.substr(move_name.find("/")+1);
    ROS_INFO("Move found: %s, control_group: %s, pose_name: %s", move_name.c_str(), control_group.c_str(), pose_name.c_str());

    if(std::find(named_pose.begin(), named_pose.end(), pose_name) != named_pose.end()){
      

      if(control_group == "arm_1_4"){
        current_move_group = &move_group_arm1_4;
        current_joint_model = &joint_model_arm1_4;
      }
      (**current_move_group).setPlanningPipelineId("ompl");
      (**current_move_group).setPlanningTime(1.0);

      moveit::core::RobotStatePtr current_state = (**current_move_group).getCurrentState();
      std::vector<double> joint_group_positions;
      current_state->copyJointGroupPositions(*current_joint_model, joint_group_positions);
      joint_group_positions[0] =  1.57;  // -1/6 turn in radians
      (**current_move_group).setJointValueTarget(joint_group_positions);

      (**current_move_group).setMaxVelocityScalingFactor(0.05);
      (**current_move_group).setMaxAccelerationScalingFactor(0.05);

      // plan and execute
      moveit::planning_interface::MoveGroupInterface::Plan my_plan;
      
      // Plan and execute on appropriate arm
      bool success; 
      success = ((**current_move_group).plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
      ROS_INFO("Visualizing plan %s", success ? "" : "FAILED");
      visual_tools->publishTrajectoryLine(my_plan.trajectory_, (**current_joint_model).getLinkModel((**current_move_group).getEndEffectorLink()), (*current_joint_model));
      visual_tools->trigger();
      if (success) {
        visual_tools->prompt("Press 'next' to execute plan");
        (**current_move_group).execute(my_plan);
      }
    } else {
      // setting custom pose
    }

  }

  //****************************************************************************
  // Exit
  // ros::shutdown();
  ROS_INFO("Task finished");
  return 0;
}