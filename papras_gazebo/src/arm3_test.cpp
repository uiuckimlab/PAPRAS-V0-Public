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
  visual_tools->publishTrajectoryLine(my_plan.trajectory_, joint_model_group->getLinkModel("robot3/end_link"), joint_model_group);
  visual_tools->trigger();
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
  static const std::string PLANNING_GROUP_ARM3 = "arm3";

  // Set move group planning interfaces
  moveit::planning_interface::MoveGroupInterface move_group_arm3(PLANNING_GROUP_ARM3);

  // Set up planning scene to add/remove collision objects in world
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // Create pointers to planning groups
  const moveit::core::JointModelGroup* joint_model_arm3 =
    move_group_arm3.getCurrentState()->getJointModelGroup(PLANNING_GROUP_ARM3);

  // Print reference information
  ROS_INFO("Arm 3 planning frame: %s", move_group_arm3.getPlanningFrame().c_str());
  ROS_INFO("Arm 3 end effector link: %s", move_group_arm3.getEndEffectorLink().c_str());
  //***************************************************************************

  //***************************************************************************
  // Set up frame transforms
  
  // World to Arm 3 HTM
  tf2::Matrix3x3 rot_w_to_3( 0,  1,  0,
                            -1,  0,  0,
                             0,  0,  1); // -pi/2 around z
  tf2::Vector3 tra_w_to_3(0.3735, -0.2535, 0.686);
  tf2::Transform htm_w_to_3(rot_w_to_3, tra_w_to_3);
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
  // move_group_arm3.setJointValueTarget(move_group_arm3.getNamedTargetValues("rest"));
  // ROS_INFO("Move arm 3 to rest");
  // plan_execute_move(&move_group_arm3, &visual_tools, joint_model_arm3);

  // Move to init
  // geometry_msgs::Pose init;
  // tf2::Quaternion init_quat;
  // init_quat.setRPY(0.0, 0.0, -M_PI_2);
  // init.orientation = tf2::toMsg(init_quat);
  // init.position.x = 0.3735;
  // init.position.y = -0.7535;
  // init.position.z = 1.0;
  tf2::Vector3 tra_init_3(0.5, 0.1, 0.5);
  tf2::Transform htm_init_3(tf2::Matrix3x3::getIdentity(), tra_init_3);
  tf2::Transform htm_w_to_init = htm_w_to_3 * htm_init_3;
  geometry_msgs::Pose init;
  tf2::toMsg(htm_w_to_init, init);
  visual_tools.publishAxis(init);
  visual_tools.trigger();
  move_group_arm3.setPoseTarget(init);
  ROS_INFO("Move arm 3 to init (pose target)");
  plan_execute_move(&move_group_arm3, &visual_tools, joint_model_arm3);
  //***************************************************************************

  // Move to rest and exit
  // move_group_arm3.setJointValueTarget(move_group_arm3.getNamedTargetValues("rest"));
  // ROS_INFO("Move arm 3 to rest");
  // plan_execute_move(&move_group_arm3, &visual_tools, joint_model_arm3);
  ros::shutdown();
  return 0;
}