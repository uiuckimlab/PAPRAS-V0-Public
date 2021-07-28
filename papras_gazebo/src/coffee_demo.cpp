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

// Double pointers for current move
moveit::planning_interface::MoveGroupInterface** current_move_group;
const moveit::core::JointModelGroup** current_joint_model;

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

void plan_execute_arm_move(const int arm,
                           moveit::planning_interface::MoveGroupInterface** move_group,
                           const moveit::core::JointModelGroup** joint_model)
{
  // Create plan object
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  
  // Plan and execute on appropriate arm
  bool success;
  success = ((**move_group).plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO("Visualizing plan %s", success ? "" : "FAILED");
  visual_tools->publishTrajectoryLine(my_plan.trajectory_, (**joint_model).getLinkModel((**move_group).getEndEffectorLink()), *joint_model);
  visual_tools->trigger();
  if (success) {
    visual_tools->prompt("Press 'next' to execute plan");
    (**move_group).execute(my_plan);
  }
}

void do_arm_pose_move(const tf2Scalar& x, const tf2Scalar& y, const tf2Scalar& z,
                      const tf2Scalar& roll, const tf2Scalar& pitch, const tf2Scalar& yaw,
                      const int arm, const std::string pose_name)
{
  // Assign double pointers for current move group
  switch (arm)
  {
    case 1:
      current_move_group = &move_group_arm1;
      current_joint_model = &joint_model_arm1;
      break;
    case 2:
      current_move_group = &move_group_arm2;
      current_joint_model = &joint_model_arm2;
      break;
    case 3:
      current_move_group = &move_group_arm3;
      current_joint_model = &joint_model_arm3;
  }
  // Set planning pipeline id to chomp
  (**current_move_group).setPlanningPipelineId("ompl");
  (**current_move_group).setPlanningTime(1.0);
  
  // Create goal pose in world frame
  geometry_msgs::Pose goal_pose = pose_transform(x, y, z, roll, pitch, yaw, arm);

  // Height constraint on end-effector of 0.6m
  // shape_msgs::SolidPrimitive box;
  // box.type = box.BOX;
  // box.dimensions.resize(3);
  // box.dimensions[box.BOX_X] = 1.0;
  // box.dimensions[box.BOX_Y] = 2.0;
  // box.dimensions[box.BOX_Z] = 0.6;
  // geometry_msgs::Pose box_pose;
  // box_pose.position.x = 0.0;
  // box_pose.position.y = 0.0;
  // box_pose.position.z = 0.95;
  // box_pose.orientation.w = 1.0;
  // moveit_msgs::BoundingVolume bounding_box;
  // bounding_box.primitives.push_back(box);
  // bounding_box.primitive_poses.push_back(box_pose);
  // moveit_msgs::PositionConstraint eef_height;
  // eef_height.header.frame_id = "world";
  // eef_height.link_name = (**current_move_group).getEndEffectorLink();
  // eef_height.constraint_region = bounding_box;
  // eef_height.weight = 1.0;
  // moveit_msgs::Constraints constraints;
  // constraints.position_constraints.push_back(eef_height);
  // (**current_move_group).setPathConstraints(constraints);


  // Set joint target from IK for current move group
  bool success;
  success = (**current_move_group).setJointValueTarget(goal_pose, (**current_move_group).getEndEffectorLink());
  ROS_INFO("IK Solver %s", success ? "PASSED" : "FAILED");
  // if (!success) {
  //   success = (**current_move_group).setApproximateJointValueTarget(goal_pose, (**current_move_group).getEndEffectorLink());
  //   ROS_INFO("Approx IK Solver %s", success ? "PASSED" : "FAILED");
  // }
  int ik_attempts = 1;
  while (!success) {
    if (ik_attempts > 10) break;
    success = (**current_move_group).setJointValueTarget(goal_pose, (**current_move_group).getEndEffectorLink());
    ROS_INFO("IK Solver %s", success ? "PASSED" : "FAILED");
    ik_attempts++;
  }

  // Terminal info and pose in Rviz
  ROS_INFO("Move arm %d to pose %s", arm, pose_name.c_str());
  visual_tools->deleteAllMarkers();
  visual_tools->publishAxisLabeled(goal_pose, pose_name);

  // Plan and execute move
  plan_execute_arm_move(arm, current_move_group, current_joint_model);

  // Clear constraints
  // (**current_move_group).clearPathConstraints();
}

void do_arm_named_move(const int arm, const std::string pose_name)
{
  // Assign double pointers for current move group
  switch (arm)
  {
    case 1:
      current_move_group = &move_group_arm1;
      current_joint_model = &joint_model_arm1;
      break;
    case 2:
      current_move_group = &move_group_arm2;
      current_joint_model = &joint_model_arm2;
      break;
    case 3:
      current_move_group = &move_group_arm3;
      current_joint_model = &joint_model_arm3;
  }
  // Set planning pipeline id to chomp
  (**current_move_group).setPlanningPipelineId("ompl");
  (**current_move_group).setPlanningTime(1.0);

  // Set joint target for the current move group
  (**current_move_group).setNamedTarget(pose_name);

  // Terminal info
  ROS_INFO("Move arm %d to pose %s", arm, pose_name.c_str());
  visual_tools->deleteAllMarkers();

  // Plan and execute move
  plan_execute_arm_move(arm, current_move_group, current_joint_model);
}

void plan_execute_gripper_move(const int gripper,
                               moveit::planning_interface::MoveGroupInterface** move_group,
                               const moveit::core::JointModelGroup** joint_model)
{
  // Create plan object
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  
  // Plan and execute on appropriate gripper
  bool success;
  success = ((**move_group).plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO("Visualizing plan %s", success ? "" : "FAILED");
  visual_tools->trigger();
  if (success) {
    visual_tools->prompt("Press 'next' to execute plan");
    (**move_group).execute(my_plan);
  }
}

void do_gripper_angle_move(const tf2Scalar& angle, const int gripper)
{
  // Assign double pointers for current move group
  switch (gripper)
  {
    case 1:
      current_move_group = &move_group_gripper1;
      current_joint_model = &joint_model_gripper1;
      break;
    case 2:
      current_move_group = &move_group_gripper2;
      current_joint_model = &joint_model_gripper2;
      break;
    case 3:
      current_move_group = &move_group_gripper3;
      current_joint_model = &joint_model_gripper3;
  }
  // Set planning pipeline id to ompl
  (**current_move_group).setPlanningPipelineId("ompl");
  
  // Create vector for target angle
  std::vector<double> target_angle{ angle, angle };

  // Set pose target for current move group
  (**current_move_group).setJointValueTarget(target_angle);

  // Terminal info
  ROS_INFO("Move gripper %d to angle %f", gripper, angle);
  visual_tools->deleteAllMarkers();

  // Plan and execute move
  plan_execute_gripper_move(gripper, current_move_group, current_joint_model);
}

void do_gripper_named_move(const int gripper, const std::string pose_name)
{
  // Assign double pointers for current move group
  switch (gripper)
  {
    case 1:
      current_move_group = &move_group_gripper1;
      current_joint_model = &joint_model_gripper1;
      break;
    case 2:
      current_move_group = &move_group_gripper2;
      current_joint_model = &joint_model_gripper2;
      break;
    case 3:
      current_move_group = &move_group_gripper3;
      current_joint_model = &joint_model_gripper3;
  }
  // Set planning pipeline id to ompl
  (**current_move_group).setPlanningPipelineId("ompl");
  
  // Set joint target for current move group
  (**current_move_group).setNamedTarget(pose_name);

  // Terminal info
  ROS_INFO("Move gripper %d to pose %s", gripper, pose_name.c_str());
  visual_tools->deleteAllMarkers();

  // Plan and execute move
  plan_execute_gripper_move(gripper, current_move_group, current_joint_model);
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
  do_arm_named_move(1, "rest");
  do_arm_named_move(3, "rest");

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
    std::string control_name = control_group.substr(0, control_group.length()-1);
    std::string control_id = control_group.substr(control_group.length()-1);
    std::string pose_name = move_name.substr(move_name.find("/")+1);
    // ROS_INFO("Move found: %s, control_group: %s, pose_name: %s", move_name.c_str(), control_group.c_str(), pose_name.c_str());

    // Get pose data
    std::vector<double> pose_data;
    node_handle.getParam("/"+move_name, pose_data);

    // Do appropriate move
    if (control_name == "arm") {
      do_arm_pose_move(pose_data.at(0), pose_data.at(1), pose_data.at(2), pose_data.at(3), pose_data.at(4), pose_data.at(5), std::stoi(control_id), pose_name);
    } else if (control_name == "gripper") {
      do_gripper_angle_move(pose_data.at(0), std::stoi(control_id));
    } else {} // Do nothing
  }

  //****************************************************************************
  // Exit
  do_arm_named_move(1, "rest");
  do_arm_named_move(3, "rest");
  ros::shutdown();
  return 0;
}