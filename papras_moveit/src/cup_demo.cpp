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

// The circle constant tau = 2*pi. One tau is one rotation in radians.
const double tau = 2 * M_PI;

void plan_execute_move(moveit::planning_interface::MoveGroupInterface* move_group_interface,
                       moveit_visual_tools::MoveItVisualTools* visual_tools,
                       const moveit::core::JointModelGroup* joint_model_group)
{
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = (move_group_interface->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO("Visualizing plan %s", success ? "" : "FAILED");
  // visual_tools->deleteAllMarkers();
  // visual_tools->publishTrajectoryLine(my_plan.trajectory_, joint_model_group->getLinkModel("robot3/end_link"), joint_model_group);
  // visual_tools->trigger();
  if (success)
  {
    visual_tools->prompt("Press 'next' to execute plan");
    move_group_interface->execute(my_plan);
  }
}

int main(int argc, char** argv)
{
  // Setup node and start AsyncSpinner
  const std::string node_name = "cup_demo";
  ros::init(argc, argv, node_name);
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  //***************************************************************************
  // Set up planning interface

  // Set planning group for arms
  static const std::string PLANNING_GROUP_ARM2 = "arm2";
  // static const std::string PLANNING_GROUP_GRIPPER2 = "gripper2";
  static const std::string PLANNING_GROUP_ARM3 = "arm3";
  // static const std::string PLANNING_GROUP_GRIPPER3 = "gripper3";

  // Set move group planning interfaces
  moveit::planning_interface::MoveGroupInterface move_group_arm2(PLANNING_GROUP_ARM2);
  // moveit::planning_interface::MoveGroupInterface move_group_grip2(PLANNING_GROUP_GRIPPER2);
  moveit::planning_interface::MoveGroupInterface move_group_arm3(PLANNING_GROUP_ARM3);
  // moveit::planning_interface::MoveGroupInterface move_group_grip3(PLANNING_GROUP_GRIPPER3);

  // Set up planning scene to add/remove collision objects in world
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // Create pointers to planning groups
  const moveit::core::JointModelGroup* joint_model_arm2 =
    move_group_arm2.getCurrentState()->getJointModelGroup(PLANNING_GROUP_ARM2);
  // const moveit::core::JointModelGroup* joint_model_grip2 =
  //   move_group_arm2.getCurrentState()->getJointModelGroup(PLANNING_GROUP_GRIPPER2);
  const moveit::core::JointModelGroup* joint_model_arm3 =
    move_group_arm3.getCurrentState()->getJointModelGroup(PLANNING_GROUP_ARM3);
  // const moveit::core::JointModelGroup* joint_model_grip3 =
  //   move_group_arm2.getCurrentState()->getJointModelGroup(PLANNING_GROUP_GRIPPER3);

  // Print reference information
  ROS_INFO("Arm 2 planning frame: %s", move_group_arm2.getPlanningFrame().c_str());
  ROS_INFO("Arm 2 end effector link: %s", move_group_arm2.getEndEffectorLink().c_str());
  ROS_INFO("Arm 3 planning frame: %s", move_group_arm3.getPlanningFrame().c_str());
  ROS_INFO("Arm 3 end effector link: %s", move_group_arm3.getEndEffectorLink().c_str());
  //***************************************************************************

  //***************************************************************************
  // Set up frame transforms
  
  // World to Arm 3 HTM
  // tf2::Matrix3x3 rot_w_to_3 = tf2::Matrix3x3( 0,  1,  0,
  //                                            -1,  0,  0,
  //                                             0,  0,  1); // -pi/2 around z
  // tf2::Vector3 tra_w_to_3 = tf2::Vector3(0.3735, -0.5545, 0.686);
  // tf2::Transform htm_w_to_3 = tf2::Transform(rot_w_to_3, tra_w_to_3);
  //***************************************************************************
  
  //***************************************************************************
  // Set up visualization
  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools("world");
  visual_tools.deleteAllMarkers();

  // Enable RViz remote control
  visual_tools.loadRemoteControl();

  // Batch publish RViz commands
  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = 1.0;
  visual_tools.publishText(text_pose, "Cup Demo", rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();

  // User input to start demo
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to begin");
  //***************************************************************************

  //***************************************************************************
  // Cup Demo
  // Initialize to rest pose
  move_group_arm2.setJointValueTarget(move_group_arm2.getNamedTargetValues("rest"));
  ROS_INFO("Move arm 2 to rest");
  plan_execute_move(&move_group_arm2, &visual_tools, joint_model_arm2);
  move_group_arm3.setJointValueTarget(move_group_arm3.getNamedTargetValues("rest"));
  ROS_INFO("Move arm 3 to rest");
  plan_execute_move(&move_group_arm3, &visual_tools, joint_model_arm3);

  // Arm 1 (cup)
  // Move to home
  move_group_arm2.setJointValueTarget(move_group_arm2.getNamedTargetValues("home"));
  ROS_INFO("Move arm 2 to home");
  plan_execute_move(&move_group_arm2, &visual_tools, joint_model_arm2);
  // Move to cup_init
  std::vector<double> cup_init{ -0.660, 0.255, 1.161, 0.083, -1.453, -0.034 };
  move_group_arm2.setJointValueTarget(cup_init);
  ROS_INFO("Move arm 2 to cup_init");
  plan_execute_move(&move_group_arm2, &visual_tools, joint_model_arm2);
  // Open gripper
  // move_group_grip2.setJointValueTarget(move_group_grip2.getNamedTargetValues("open"));
  // ROS_INFO("Open gripper 2");
  // plan_execute_move(&move_group_grip2, &visual_tools, joint_model_grip2);
  // Grab cup
  std::vector<double> grab_cup{ -0.677, 0.411, 0.482, 0.083, -0.905, -0.034 };
  move_group_arm2.setJointValueTarget(grab_cup);
  ROS_INFO("Move arm 2 to grab_cup");
  plan_execute_move(&move_group_arm2, &visual_tools, joint_model_arm2);
  // Close gripper
  // move_group_grip2.setJointValueTarget(move_group_grip2.getNamedTargetValues("close"));
  // ROS_INFO("Close gripper 2");
  // plan_execute_move(&move_group_grip2, &visual_tools, joint_model_grip2);
  // Release cup
  std::vector<double> release_cup{ -1.643, 0.326, 1.111, 0.083, -1.449, -0.033 };
  move_group_arm2.setJointValueTarget(release_cup);
  ROS_INFO("Move arm 2 to release_cup");
  plan_execute_move(&move_group_arm2, &visual_tools, joint_model_arm2);
  // Open gripper
  // move_group_grip2.setJointValueTarget(move_group_grip2.getNamedTargetValues("open"));
  // ROS_INFO("Open gripper 2");
  // plan_execute_move(&move_group_grip2, &visual_tools, joint_model_grip2);
  // Move to home
  move_group_arm2.setJointValueTarget(move_group_arm2.getNamedTargetValues("home"));
  ROS_INFO("Move arm 2 to home");
  plan_execute_move(&move_group_arm2, &visual_tools, joint_model_arm2);
  
  // Arm 3 (pot)
  // Move to home
  move_group_arm3.setJointValueTarget(move_group_arm3.getNamedTargetValues("home"));
  ROS_INFO("Move arm 3 to home");
  plan_execute_move(&move_group_arm3, &visual_tools, joint_model_arm3);
  // Move to pot_init
  std::vector<double> pot_init{ -0.291, 0.053, 1.278, 0.001, -1.299, 0.015 };
  move_group_arm3.setJointValueTarget(pot_init);
  ROS_INFO("Move arm 3 to pot_init");
  plan_execute_move(&move_group_arm3, &visual_tools, joint_model_arm3);
  // Open gripper
  // move_group_grip3.setJointValueTarget(move_group_grip3.getNamedTargetValues("open"));
  // ROS_INFO("Open gripper 3");
  // plan_execute_move(&move_group_grip3, &visual_tools, joint_model_grip3);
  // Grab pot
  std::vector<double> grab_pot{ -0.281, 0.328, 0.925, 0.001, -1.276, -0.343 };
  move_group_arm3.setJointValueTarget(grab_pot);
  ROS_INFO("Move arm 3 to grab_pot");
  plan_execute_move(&move_group_arm3, &visual_tools, joint_model_arm3);
  // Close gripper
  // move_group_grip3.setJointValueTarget(move_group_grip3.getNamedTargetValues("close"));
  // ROS_INFO("Close gripper 3");
  // plan_execute_move(&move_group_grip3, &visual_tools, joint_model_grip3);
  // Prep pot
  std::vector<double> prep_pot{ -0.849, 0.191, 0.708, 0.001, -0.955, -0.345 };
  move_group_arm3.setJointValueTarget(prep_pot);
  ROS_INFO("Move arm 3 to prep_pot");
  plan_execute_move(&move_group_arm3, &visual_tools, joint_model_arm3);
  // Pour pot
  std::vector<double> pour_pot{ -0.744, 0.061, 0.556, 0.001, -0.673, -1.138 };
  move_group_arm3.setJointValueTarget(pour_pot);
  ROS_INFO("Move arm 3 to pour_pot");
  plan_execute_move(&move_group_arm3, &visual_tools, joint_model_arm3);
  // Prep pot
  move_group_arm3.setJointValueTarget(prep_pot);
  ROS_INFO("Move arm 3 to prep_pot");
  plan_execute_move(&move_group_arm3, &visual_tools, joint_model_arm3);
  // Open gripper
  // move_group_grip3.setJointValueTarget(move_group_grip3.getNamedTargetValues("open"));
  // ROS_INFO("Open gripper 3");
  // plan_execute_move(&move_group_grip3, &visual_tools, joint_model_grip3);
  // Move to home
  move_group_arm3.setJointValueTarget(move_group_arm3.getNamedTargetValues("home"));
  ROS_INFO("Move arm 3 to home");
  plan_execute_move(&move_group_arm3, &visual_tools, joint_model_arm3);
  //***************************************************************************

  // Move to rest and exit
  visual_tools.prompt("Press 'next' to move to rest and exit");
  move_group_arm2.setJointValueTarget(move_group_arm2.getNamedTargetValues("rest"));
  ROS_INFO("Move arm 2 to rest");
  plan_execute_move(&move_group_arm2, &visual_tools, joint_model_arm2);
  move_group_arm3.setJointValueTarget(move_group_arm3.getNamedTargetValues("rest"));
  ROS_INFO("Move arm 3 to rest");
  plan_execute_move(&move_group_arm3, &visual_tools, joint_model_arm3);
  ros::shutdown();
  return 0;
}