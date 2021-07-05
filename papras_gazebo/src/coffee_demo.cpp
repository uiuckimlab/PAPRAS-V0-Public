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

//******************************************************************************
// Set up global frame transforms
// World to Arm 1 HTM
tf2::Matrix3x3 rot_w_to_1( 0, -1,  0,
                           1,  0,  0,
                           0,  0,  1); // pi/2 around z
tf2::Vector3 tra_w_to_1(-0.3795, 0.0435, 0.686);
tf2::Transform htm_w_to_1(rot_w_to_1, tra_w_to_1);

// World to Arm 2 HTM
tf2::Matrix3x3 rot_w_to_2( 0, -1,  0,
                           1,  0,  0,
                           0,  0,  1); // pi/2 around z
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
static const std::string PLANNING_GROUP_ARM1 = "arm1";
static const std::string PLANNING_GROUP_GRIPPER1 = "gripper1";
static const std::string PLANNING_GROUP_ARM2 = "arm2";
static const std::string PLANNING_GROUP_GRIPPER2 = "gripper2";
static const std::string PLANNING_GROUP_ARM3 = "arm3";
static const std::string PLANNING_GROUP_GRIPPER3 = "gripper3";

moveit::planning_interface::MoveGroupInterface* move_group_arm1;
moveit::planning_interface::MoveGroupInterface* move_group_gripper1;
moveit::planning_interface::MoveGroupInterface* move_group_arm2;
moveit::planning_interface::MoveGroupInterface* move_group_gripper2;
moveit::planning_interface::MoveGroupInterface* move_group_arm3;
moveit::planning_interface::MoveGroupInterface* move_group_gripper3;

// Pointers to move groups
const moveit::core::JointModelGroup* joint_model_arm1;
const moveit::core::JointModelGroup* joint_model_gripper1;
const moveit::core::JointModelGroup* joint_model_arm2;
const moveit::core::JointModelGroup* joint_model_gripper2;
const moveit::core::JointModelGroup* joint_model_arm3;
const moveit::core::JointModelGroup* joint_model_gripper3;

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

void plan_execute_arm_move(const int arm)
{
  // Create plan object
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  
  // Plan and execute on appropriate arm
  bool success;
  switch (arm)
  {
  case 1:
    success = (move_group_arm1->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO("Visualizing plan %s", success ? "" : "FAILED");
    visual_tools->publishTrajectoryLine(my_plan.trajectory_, joint_model_arm1->getLinkModel("robot1/end_effector_link"), joint_model_arm1);
    visual_tools->trigger();
    if (success)
    {
      visual_tools->prompt("Press 'next' to execute plan");
      move_group_arm1->execute(my_plan);
    }
    break;
  case 2:
    success = (move_group_arm2->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO("Visualizing plan %s", success ? "" : "FAILED");
    visual_tools->publishTrajectoryLine(my_plan.trajectory_, joint_model_arm2->getLinkModel("robot2/end_effector_link"), joint_model_arm2);
    visual_tools->trigger();
    if (success)
    {
      visual_tools->prompt("Press 'next' to execute plan");
      move_group_arm2->execute(my_plan);
    }
    break;
  case 3:
    success = (move_group_arm3->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO("Visualizing plan %s", success ? "" : "FAILED");
    visual_tools->publishTrajectoryLine(my_plan.trajectory_, joint_model_arm3->getLinkModel("robot3/end_effector_link"), joint_model_arm3);
    visual_tools->trigger();
    if (success)
    {
      visual_tools->prompt("Press 'next' to execute plan");
      move_group_arm3->execute(my_plan);
    }
    break;
  }
}

void do_arm_pose_move(const tf2Scalar& x, const tf2Scalar& y, const tf2Scalar& z,
                      const tf2Scalar& roll, const tf2Scalar& pitch, const tf2Scalar& yaw,
                      const int arm, const std::string pose_name)
{
  // Create goal pose in world frame
  geometry_msgs::Pose goal_pose = pose_transform(x, y, z, roll, pitch, yaw, arm);

  // Set pose target for appropriate move group
  switch (arm)
  {
  case 1:
    move_group_arm1->setPoseTarget(goal_pose);
    break;
  case 2:
    move_group_arm2->setPoseTarget(goal_pose);
    break;
  case 3:
    move_group_arm3->setPoseTarget(goal_pose);
    break;
  }

  // Terminal info and pose in Rviz
  ROS_INFO("Move arm %d to pose %s", arm, pose_name.c_str());
  visual_tools->deleteAllMarkers();
  visual_tools->publishAxisLabeled(goal_pose, pose_name);

  // Plan and execute move
  plan_execute_arm_move(arm);
}

void do_arm_named_move(const int arm, const std::string pose_name)
{
  // Set joint target for appropriate move group
  switch (arm)
  {
  case 1:
    move_group_arm1->setJointValueTarget(move_group_arm1->getNamedTargetValues(pose_name));
    break;
  case 2:
    move_group_arm2->setJointValueTarget(move_group_arm2->getNamedTargetValues(pose_name));
    break;
  case 3:
    move_group_arm3->setJointValueTarget(move_group_arm3->getNamedTargetValues(pose_name));
    break;
  }

  // Terminal info
  ROS_INFO("Move arm %d to pose %s", arm, pose_name.c_str());
  visual_tools->deleteAllMarkers();

  // Plan and execute move
  plan_execute_arm_move(arm);
}

void plan_execute_gripper_move(const int gripper)
{
  // Create plan object
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  
  // Plan and execute on appropriate gripper
  bool success;
  switch (gripper)
  {
  case 1:
    success = (move_group_gripper1->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO("Visualizing plan %s", success ? "" : "FAILED");
    visual_tools->trigger();
    if (success)
    {
      visual_tools->prompt("Press 'next' to execute plan");
      move_group_gripper1->execute(my_plan);
    }
    break;
  case 2:
    success = (move_group_gripper2->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO("Visualizing plan %s", success ? "" : "FAILED");
    visual_tools->trigger();
    if (success)
    {
      visual_tools->prompt("Press 'next' to execute plan");
      move_group_gripper2->execute(my_plan);
    }
    break;
  case 3:
    success = (move_group_gripper3->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO("Visualizing plan %s", success ? "" : "FAILED");
    visual_tools->trigger();
    if (success)
    {
      visual_tools->prompt("Press 'next' to execute plan");
      move_group_gripper3->execute(my_plan);
    }
    break;
  }
}

void do_gripper_angle_move(const tf2Scalar& angle, const int gripper)
{
  // Create vector for target angle
  std::vector<double> target_angle{ angle, angle };

  // Set pose target for appropriate move group
  switch (gripper)
  {
  case 1:
    move_group_gripper1->setJointValueTarget(target_angle);
    break;
  case 2:
    move_group_gripper2->setJointValueTarget(target_angle);
    break;
  case 3:
    move_group_gripper3->setJointValueTarget(target_angle);
    break;
  }

  // Terminal info
  ROS_INFO("Move gripper %d to angle %f", gripper, angle);
  visual_tools->deleteAllMarkers();

  // Plan and execute move
  plan_execute_gripper_move(gripper);
}

void do_gripper_named_move(const int gripper, const std::string pose_name)
{
  // Set joint target for appropriate move group
  switch (gripper)
  {
  case 1:
    move_group_gripper1->setJointValueTarget(move_group_gripper1->getNamedTargetValues(pose_name));
    break;
  case 2:
    move_group_gripper2->setJointValueTarget(move_group_gripper2->getNamedTargetValues(pose_name));
    break;
  case 3:
    move_group_gripper3->setJointValueTarget(move_group_gripper3->getNamedTargetValues(pose_name));
    break;
  }

  // Terminal info
  ROS_INFO("Move gripper %d to pose %s", gripper, pose_name.c_str());
  visual_tools->deleteAllMarkers();

  // Plan and execute move
  plan_execute_gripper_move(gripper);
}

int main(int argc, char** argv)
{
  // Setup node and start AsyncSpinner
  const std::string node_name = "coffee_demo";
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
  move_group_arm3 = new moveit::planning_interface::MoveGroupInterface(PLANNING_GROUP_ARM3);
  move_group_gripper3 = new moveit::planning_interface::MoveGroupInterface(PLANNING_GROUP_GRIPPER3);

  // Set up planning scene to add/remove collision objects in world
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // Create pointers to planning groups
  joint_model_arm1 = move_group_arm1->getCurrentState()->getJointModelGroup(PLANNING_GROUP_ARM1);
  joint_model_gripper1 = move_group_gripper1->getCurrentState()->getJointModelGroup(PLANNING_GROUP_GRIPPER1);
  joint_model_arm2 = move_group_arm2->getCurrentState()->getJointModelGroup(PLANNING_GROUP_ARM2);
  joint_model_gripper2 = move_group_gripper2->getCurrentState()->getJointModelGroup(PLANNING_GROUP_GRIPPER2);
  joint_model_arm3 = move_group_arm3->getCurrentState()->getJointModelGroup(PLANNING_GROUP_ARM3);
  joint_model_gripper3 = move_group_gripper2->getCurrentState()->getJointModelGroup(PLANNING_GROUP_GRIPPER3);

  // Print reference information
  ROS_INFO("Arm 1 planning frame: %s", move_group_arm1->getPlanningFrame().c_str());
  ROS_INFO("Arm 1 end effector link: %s", move_group_arm1->getEndEffectorLink().c_str());
  ROS_INFO("Arm 2 planning frame: %s", move_group_arm2->getPlanningFrame().c_str());
  ROS_INFO("Arm 2 end effector link: %s", move_group_arm2->getEndEffectorLink().c_str());
  ROS_INFO("Arm 3 planning frame: %s", move_group_arm3->getPlanningFrame().c_str());
  ROS_INFO("Arm 3 end effector link: %s", move_group_arm3->getEndEffectorLink().c_str());
  
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
  do_arm_named_move(3, "rest");
  do_arm_named_move(1, "rest");

  //****************************************************************************
  /* Coffee Demo
  *
  *  Arm move to generic task space pose in robot frame:
  *    do_arm_pose_move(x, y, z, roll, pitch, yaw, arm_number, "pose_name")
  *
  *  Arm move to named joint pose (init, home, rest):
  *    do_arm_named_move(arm_number, "pose_name")
  * 
  *  Gripper move to generic angle:
  *    do_gripper_angle_move(angle, gripper_number)
  * 
  *  Gripper move to named joint pose (open, close):
  *    do_gripper_named_move(gripper_number, "pose_name")
  */
  // Load list of moves from ROS Parameter Server
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

    // Do appropriate move
    if (control_group == "arm1") {
      do_arm_pose_move(pose_data.at(0), pose_data.at(1), pose_data.at(2), pose_data.at(3), pose_data.at(4), pose_data.at(5), 1, pose_name);
    } else if (control_group == "arm2") {
      do_arm_pose_move(pose_data.at(0), pose_data.at(1), pose_data.at(2), pose_data.at(3), pose_data.at(4), pose_data.at(5), 2, pose_name);
    } else if (control_group == "arm3") {
      do_arm_pose_move(pose_data.at(0), pose_data.at(1), pose_data.at(2), pose_data.at(3), pose_data.at(4), pose_data.at(5), 3, pose_name);
    } else if (control_group == "gripper1") {
      do_gripper_angle_move(pose_data.at(0), 1);
    } else if (control_group == "gripper2") {
      do_gripper_angle_move(pose_data.at(0), 2);
    } else if (control_group == "gripper3") {
      do_gripper_angle_move(pose_data.at(0), 3);
    } else {} // Do nothing
  }

  //****************************************************************************
  // Exit
  do_arm_named_move(1, "rest");
  do_arm_named_move(3, "rest");
  ros::shutdown();
  return 0;
}