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

// The circle constant tau = 2*pi. One tau is one rotation in radians.
const double tau = 2 * M_PI;

//******************************************************************************
// Set up global frame transforms
// World to Arm 1 HTM
tf2::Matrix3x3 rot_w_to_1( 0, -1,  0,
                           1,  0,  0,
                           0,  0,  1); // pi/2 around z
tf2::Vector3 tra_w_to_1(-0.3795, 0.0435, 0.686);
tf2::Transform htm_w_to_1(rot_w_to_1, tra_w_to_1);

// World to Arm 2 HTM
tf2::Matrix3x3 rot_w_to_2(-1,  0,  0,
                           0, -1,  0,
                           0,  0,  1); // pi around z
tf2::Vector3 tra_w_to_2(-0.3835, -0.5545, 0.686);
tf2::Transform htm_w_to_2(rot_w_to_2, tra_w_to_2);

// World to Arm 3 HTM
tf2::Matrix3x3 rot_w_to_3( 0,  1,  0,
                          -1,  0,  0,
                           0,  0,  1); // -pi/2 around z
tf2::Vector3 tra_w_to_3(0.3735, -0.2535, 0.686);
tf2::Transform htm_w_to_3(rot_w_to_3, tra_w_to_3);

//******************************************************************************
// Declare global objects
// Create move group planning interfaces
static const std::string PLANNING_GROUP_ARM1_2 = "arm1_2";
static const std::string PLANNING_GROUP_ARM2_3 = "arm2_3";
static const std::string PLANNING_GROUP_ARM1_3 = "arm1_3";
static const std::string PLANNING_GROUP_ARM1_2_3 = "arm1_2_3";

moveit::planning_interface::MoveGroupInterface* move_group_arm1_2;
moveit::planning_interface::MoveGroupInterface* move_group_arm2_3;
moveit::planning_interface::MoveGroupInterface* move_group_arm1_3;
moveit::planning_interface::MoveGroupInterface* move_group_arm1_2_3;

// Pointers to move groups
const moveit::core::JointModelGroup* joint_model_arm1_2;
const moveit::core::JointModelGroup* joint_model_arm2_3;
const moveit::core::JointModelGroup* joint_model_arm1_3;
const moveit::core::JointModelGroup* joint_model_arm1_2_3;

// Visualization
moveit_visual_tools::MoveItVisualTools* visual_tools;

geometry_msgs::Pose pose_transform(const tf2Scalar& x, const tf2Scalar& y, const tf2Scalar& z,
                                   const tf2Scalar& roll, const tf2Scalar& pitch, const tf2Scalar& yaw,
                                   const int arm)
{
  // Create HTM for goal in robot frame
  tf2::Vector3 tra(x, y, z);
  tf2::Quaternion quat;
  quat.setRPY(roll, pitch, yaw);
  tf2::Matrix3x3 rot(quat);
  tf2::Transform htm(rot, tra);

  // Select appropriate HTM from world frame to robot frame
  tf2::Transform world_htm;
  switch(arm)
  {
    case 1:
      world_htm = htm_w_to_1;
      break;
    case 2:
      world_htm = htm_w_to_2;
      break;
    case 3:
      world_htm = htm_w_to_3;
      break;
  }

  // Compute HTM from world frame to goal and return geometry_msgs::Pose
  tf2::Transform htm_out = world_htm * htm;
  geometry_msgs::Pose pose_out;
  tf2::toMsg(htm_out, pose_out);
  return pose_out;
}

void plan_execute_arm_move(moveit::planning_interface::MoveGroupInterface* move_group)
{
  // Create plan object
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  
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
  move_group_arm1_2 = new moveit::planning_interface::MoveGroupInterface(PLANNING_GROUP_ARM1_2);
  move_group_arm2_3 = new moveit::planning_interface::MoveGroupInterface(PLANNING_GROUP_ARM2_3);
  move_group_arm1_3 = new moveit::planning_interface::MoveGroupInterface(PLANNING_GROUP_ARM1_3);
  move_group_arm1_2_3 = new moveit::planning_interface::MoveGroupInterface(PLANNING_GROUP_ARM1_2_3);

  // Set up planning scene to add/remove collision objects in world
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // Create pointers to planning groups
  joint_model_arm1_2 = move_group_arm1_2->getCurrentState()->getJointModelGroup(PLANNING_GROUP_ARM1_2);
  joint_model_arm2_3 = move_group_arm2_3->getCurrentState()->getJointModelGroup(PLANNING_GROUP_ARM2_3);
  joint_model_arm1_3 = move_group_arm1_3->getCurrentState()->getJointModelGroup(PLANNING_GROUP_ARM1_3);
  joint_model_arm1_2_3 = move_group_arm1_2_3->getCurrentState()->getJointModelGroup(PLANNING_GROUP_ARM1_2_3);

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
  move_group_arm1_2_3->setNamedTarget("rest");
  plan_execute_arm_move(move_group_arm1_2_3);

  //****************************************************************************
  // Test 1: Send each arm to arbitrary point  
  geometry_msgs::Pose p1 = pose_transform(.4, .1, .4, 0, 0, 0, 1);
  geometry_msgs::Pose p2 = pose_transform(.5, .2, .3, 0, 0, 0, 2);
  geometry_msgs::Pose p3 = pose_transform(.6, .3, .2, 0, 0, 0, 3);

  move_group_arm1_2_3->setPoseTarget(p1, EEF1);
  move_group_arm1_2_3->setPoseTarget(p2, EEF2);
  move_group_arm1_2_3->setPoseTarget(p3, EEF3);
  plan_execute_arm_move(move_group_arm1_2_3);
  // Test PASS

  // Test 2: Use 123 group but only set targets for arms 1 and 2
  // Arm 3 should hold its position
  p1 = pose_transform(.5, .2, .3, 0, 0, 0, 1);
  p2 = pose_transform(.6, .3, .2, 0, 0, 0, 2);
  move_group_arm1_2_3->setPoseTarget(p1, EEF1);
  move_group_arm1_2_3->setPoseTarget(p2, EEF2);
  plan_execute_arm_move(move_group_arm1_2_3);
  // Test PASS

  //****************************************************************************
  // Exit
  move_group_arm1_2_3->setNamedTarget("rest");
  plan_execute_arm_move(move_group_arm1_2_3);
  ros::shutdown();
  return 0;
}