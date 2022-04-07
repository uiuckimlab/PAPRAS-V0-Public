#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_state/conversions.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"

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

//ros node_handle for yaml parsing
// ros::NodeHandle node_handle;
ros::Publisher finished_move_pub;
ros::Subscriber move_subscriber;
std::string msg;
bool game_finished = false;
bool do_move_bool = false;

std::unordered_map<std::string,std::string>arm1_moves({
  {"a1","/arm1/a1"},
  {"a2","/arm1/a2"},
  {"a3","/arm1/a3"},
  {"a4","/arm1/a4"},
  {"a5","/arm1/a5"},
  {"a6","/arm1/a6"},
  {"a7","/arm1/a7"},
  {"a8","/arm1/a8"},
  {"b1","/arm1/b1"},
  {"b2","/arm1/b2"},
  {"b3","/arm1/b3"},
  {"b4","/arm1/b4"},
  {"b5","/arm1/b5"},
  {"b6","/arm1/b6"},
  {"b7","/arm1/b7"},
  {"b8","/arm1/b8"},
  {"c1","/arm1/c1"},
  {"c2","/arm1/c2"},
  {"c3","/arm1/c3"},
  {"c4","/arm1/c4"},
  {"c5","/arm1/c5"},
  {"c6","/arm1/c6"},
  {"c7","/arm1/c7"},
  {"c8","/arm1/c8"},
  {"d1","/arm1/d1"},
  {"d2","/arm1/d2"},
  {"d3","/arm1/d3"},
  {"d4","/arm1/d4"},
  {"d5","/arm1/d5"},
  {"d6","/arm1/d6"},
  {"d7","/arm1/d7"},
  {"d8","/arm1/d8"},
  {"e1","/arm1/e1"},
  {"e2","/arm1/e2"},
  {"e3","/arm1/e3"},
  {"e4","/arm1/e4"},
  {"e5","/arm1/e5"},
  {"e6","/arm1/e6"},
  {"e7","/arm1/e7"},
  {"e8","/arm1/e8"},
  {"f1","/arm1/f1"},
  {"f2","/arm1/f2"},
  {"f3","/arm1/f3"},
  {"f4","/arm1/f4"},
  {"f5","/arm1/f5"},
  {"f6","/arm1/f6"},
  {"f7","/arm1/f7"},
  {"f8","/arm1/f8"},
  {"g1","/arm1/g1"},
  {"g2","/arm1/g2"},
  {"g3","/arm1/g3"},
  {"g4","/arm1/g4"},
  {"g5","/arm1/g5"},
  {"g6","/arm1/g6"},
  {"g7","/arm1/g7"},
  {"g8","/arm1/g8"},
  {"h1","/arm1/h1"},
  {"h2","/arm1/h2"},
  {"h3","/arm1/h3"},
  {"h4","/arm1/h4"},
  {"h5","/arm1/h5"},
  {"h6","/arm1/h6"},
  {"h7","/arm1/h7"},
  {"h8","/arm1/h8"},
  {"bucket","/arm2/bucket"}
});

std::unordered_map<std::string,std::string>arm2_moves({
  {"a1","/arm2/h8"},
  {"a2","/arm2/h7"},
  {"a3","/arm2/h6"},
  {"a4","/arm2/h5"},
  {"a5","/arm2/h4"},
  {"a6","/arm2/h3"},
  {"a7","/arm2/h2"},
  {"a8","/arm2/h1"},
  {"b1","/arm2/g8"},
  {"b2","/arm2/g7"},
  {"b3","/arm2/g6"},
  {"b4","/arm2/g5"},
  {"b5","/arm2/g4"},
  {"b6","/arm2/g3"},
  {"b7","/arm2/g2"},
  {"b8","/arm2/g1"},
  {"c1","/arm2/f8"},
  {"c2","/arm2/f7"},
  {"c3","/arm2/f6"},
  {"c4","/arm2/f5"},
  {"c5","/arm2/f4"},
  {"c6","/arm2/f3"},
  {"c7","/arm2/f2"},
  {"c8","/arm2/f1"},
  {"d1","/arm2/e8"},
  {"d2","/arm2/e7"},
  {"d3","/arm2/e6"},
  {"d4","/arm2/e5"},
  {"d5","/arm2/e4"},
  {"d6","/arm2/e3"},
  {"d7","/arm2/e2"},
  {"d8","/arm2/e1"},
  {"e1","/arm2/d8"},
  {"e2","/arm2/d7"},
  {"e3","/arm2/d6"},
  {"e4","/arm2/d5"},
  {"e5","/arm2/d4"},
  {"e6","/arm2/d3"},
  {"e7","/arm2/d2"},
  {"e8","/arm2/d1"},
  {"f1","/arm2/c8"},
  {"f2","/arm2/c7"},
  {"f3","/arm2/c6"},
  {"f4","/arm2/c5"},
  {"f5","/arm2/c4"},
  {"f6","/arm2/c3"},
  {"f7","/arm2/c2"},
  {"f8","/arm2/c1"},
  {"g1","/arm2/b8"},
  {"g2","/arm2/b7"},
  {"g3","/arm2/b6"},
  {"g4","/arm2/b5"},
  {"g5","/arm2/b4"},
  {"g6","/arm2/b3"},
  {"g7","/arm2/b2"},
  {"g8","/arm2/b1"},
  {"h1","/arm2/a8"},
  {"h2","/arm2/a7"},
  {"h3","/arm2/a6"},
  {"h4","/arm2/a5"},
  {"h5","/arm2/a4"},
  {"h6","/arm2/a3"},
  {"h7","/arm2/a2"},
  {"h8","/arm2/a1"},
  {"bucket","/arm2/bucket"}
});



void execute_move(const ros::NodeHandle node_handle, std::string control_group, std::string square){
    // std::vector<int> comma_positions;
    
    // comma_positions.push_back(msg.find(","));
    // comma_positions.push_back(msg.substr(comma_positions[0]+1).find(",") + comma_positions[0]);

    bool success;
    // std::string control_group = msg.substr(0,4);
    // std::string from = msg.substr(5,2);
    // std::string to = msg.substr(8);
    // std::string move_name_from;
    // std::string move_name_to; 
    moveit::planning_interface::MoveGroupInterface* move_group_gripper;
    std::vector<double> pose_data;
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    std::string square_name;

    
    if (control_group == "arm1"){
      move_group = move_group_arm1;
      move_group_gripper = move_group_gripper1;
      square_name = arm1_moves[square];
    } else if (control_group == "arm2") {
      move_group = move_group_arm2;
      move_group_gripper = move_group_gripper2;
      square_name = arm2_moves[square];
    } 

    move_group->setPlanningPipelineId("ompl");
    move_group->setMaxVelocityScalingFactor(VEL_SCALE);
    move_group->setMaxAccelerationScalingFactor(ACCEL_SCALE);
    move_group->setPlanningTime(PLANNING_TIME);
    move_group->setNumPlanningAttempts(PLAN_ATTEMPTS);
    move_group_gripper->setPlanningPipelineId("ompl");
    move_group_gripper->setMaxVelocityScalingFactor(VEL_SCALE);
    move_group_gripper->setMaxAccelerationScalingFactor(ACCEL_SCALE);
    move_group_gripper->setPlanningTime(PLANNING_TIME);
    move_group_gripper->setNumPlanningAttempts(PLAN_ATTEMPTS);


    // ****************************************************************
    // move to position playing
    node_handle.getParam("arm1/playing_home", pose_data);
    move_group->setJointValueTarget(pose_data);
    success = (move_group->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO("Visualizing plan %s", success ? "" : "FAILED");
    visual_tools->trigger();
    if (success) {
    visual_tools->prompt("Press 'next' to execute plan");
    move_group->execute(my_plan);
    }

    move_group->setStartStateToCurrentState();
    // 
    // move to position chess board square
    node_handle.getParam(square_name + "_up", pose_data);
    move_group->setJointValueTarget(pose_data);
    success = (move_group->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO("Visualizing plan %s", success ? "" : "FAILED");
    visual_tools->trigger();
    if (success) {
      visual_tools->prompt("Press 'next' to execute plan");
      move_group->execute(my_plan);
    }

    move_group->setStartStateToCurrentState();
    // 
    // move to position chess board square
    node_handle.getParam(square_name + "_down", pose_data);
    move_group->setJointValueTarget(pose_data);
    success = (move_group->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO("Visualizing plan %s", success ? "" : "FAILED");
    visual_tools->trigger();
    if (success) {
      visual_tools->prompt("Press 'next' to execute plan");
      move_group->execute(my_plan);
    }

    move_group_gripper->setStartStateToCurrentState();
    // grab
    node_handle.getParam("/gripper/grab_piece", pose_data);
    move_group_gripper->setJointValueTarget(pose_data);
    success = (move_group_gripper->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO("Visualizing plan %s", success ? "" : "FAILED");
    visual_tools->trigger();
    if (success) {
      visual_tools->prompt("Press 'next' to execute plan");
      move_group_gripper->execute(my_plan);
    }

    move_group->setStartStateToCurrentState();
    // 
    // move to position chess board square
    node_handle.getParam(square_name + "_up", pose_data);
    move_group->setJointValueTarget(pose_data);
    success = (move_group->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO("Visualizing plan %s", success ? "" : "FAILED");
    visual_tools->trigger();
    if (success) {
      visual_tools->prompt("Press 'next' to execute plan");
      move_group->execute(my_plan);
    }

    // move to position not playing
    node_handle.getParam("arm1/not_playing_home", pose_data);
    move_group->setJointValueTarget(pose_data);
    success = (move_group->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO("Visualizing plan %s", success ? "" : "FAILED");
    visual_tools->trigger();
    if (success) {
    visual_tools->prompt("Press 'next' to execute plan");
    move_group->execute(my_plan);
    }
    
    // move_group_gripper->setStartStateToCurrentState();
    // // un grasp
    // node_handle.getParam("/gripper/ungrab_piece", pose_data);
    // move_group_gripper->setJointValueTarget(pose_data);
    // success = (move_group_gripper->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    // ROS_INFO("Visualizing plan %s", success ? "" : "FAILED");
    // visual_tools->trigger();
    // if (success) {
    //   visual_tools->prompt("Press 'next' to execute plan");
    //   move_group_gripper->execute(my_plan);
    // }

    // on game finish flip the boolean

    //publish topic 
    std_msgs::Bool finished;
    finished.data = true;
    finished_move_pub.publish(finished);

}

// void do_move(const std_msgs::String::ConstPtr& msgs){
//   ROS_INFO("In do move");
//   msg = msgs->data.c_str();
//   do_move_bool = true;
// }

int main(int argc, char** argv)
{
  // Setup node and start AsyncSpinner
  const std::string node_name = "tea_demo";
  ros::init(argc, argv, node_name);
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::Rate r(10); // 10 hz
  finished_move_pub = node_handle.advertise<std_msgs::Bool>("move_finished",10);
//   move_subscriber = node_handle.subscribe("chess_move",10,do_move);
  std::vector<double> pose_data;

  //****************************************************************************
  // Set up planning interface
  // Set up move group planning interfaces
  move_group_arm1 = new moveit::planning_interface::MoveGroupInterface(PLANNING_GROUP_ARM1);
  move_group_gripper1 = new moveit::planning_interface::MoveGroupInterface(PLANNING_GROUP_GRIPPER1);
  move_group_arm2 = new moveit::planning_interface::MoveGroupInterface(PLANNING_GROUP_ARM2);
  move_group_gripper2 = new moveit::planning_interface::MoveGroupInterface(PLANNING_GROUP_GRIPPER2);
  move_group_arm1_2 = new moveit::planning_interface::MoveGroupInterface(PLANNING_GROUP_ARM1_2);

  // Set up planning scene to add/remove collision objects in world
  // moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  bool success;

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
  // move_group->setNamedTarget("rest");

  node_handle.getParam("arm1_2/not_playing_home", pose_data);
  move_group->setJointValueTarget(pose_data);
  success = (move_group->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO("Visualizing plan %s", success ? "" : "FAILED");
  visual_tools->trigger();
  if (success) {
    visual_tools->prompt("Press 'next' to execute plan");
    move_group->execute(my_plan);
  }

  move_group = move_group_gripper1;
  move_group->setStartStateToCurrentState();
  // move_group->setNamedTarget("rest");

  node_handle.getParam("gripper/ungrab_piece", pose_data);
  move_group->setJointValueTarget(pose_data);
  success = (move_group->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  //ROS_INFO("Visualizing plan %s", success ? "" : "FAILED");
  //visual_tools->trigger();
  //if (success) {
    visual_tools->prompt("Press 'next' to execute plan");
    move_group->execute(my_plan);
  //}

  move_group = move_group_gripper2;
  move_group->setStartStateToCurrentState();
  // move_group->setNamedTarget("rest");

  node_handle.getParam("gripper/ungrab_piece", pose_data);
  move_group->setJointValueTarget(pose_data);
  success = (move_group->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  // ROS_INFO("Visualizing plan %s", success ? "" : "FAILED");
  // visual_tools->trigger();
  // if (success) {
  visual_tools->prompt("Press 'next' to execute plan");
  move_group->execute(my_plan);
  ///}



  // move_group->plan(my_plan);
  // move_group->execute(my_plan);
  
  ROS_INFO("BEFORE WHILE");
  // TODO CONVERT TO C++ FROM PYTHON
  while(!game_finished && ros::ok()){
    ROS_INFO("IN WHILE");
    std::string letters = "a";
    for (int i=0; i < letters.length(); i++) {
        for (int j=0; j < 8; j++) {
            std::string square = letters[i] + std::to_string(j+1);
            execute_move(node_handle, "arm1", square);
            ros::spinOnce();
            r.sleep();
        }
    }
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