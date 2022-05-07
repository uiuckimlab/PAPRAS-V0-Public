/* Authors: Dhruv Mathur */

// ROS
#include <ros/ros.h>

// TF2
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <aruco_msgs/MarkerArray.h>
#include <aruco_msgs/Marker.h>

// eigen_conversions
#include <eigen_conversions/eigen_msg.h>

// MoveIt!
#include <geometric_shapes/solid_primitive_dims.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

// Other
#include <sstream>
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <rosparam_shortcuts/rosparam_shortcuts.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <cmath>        // std::abs
#include <vector>
#include <unordered_map>

std::vector<double> deg_to_rad(std::vector<double> &degs) {
  std::vector<double> rads;
  for (auto deg : degs) {
    rads.push_back(deg * M_PI / 180.0);
  }
  return rads;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "wall_demo");
  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::Rate r(10); 
  ros::NodeHandle nh;

  moveit::planning_interface::MoveGroupInterface group("arm1");
  moveit::planning_interface::MoveGroupInterface hand_group("gripper1");
  group.setPlanningTime(5.0);
  group.setPlannerId("RRTConnect");
  group.setMaxAccelerationScalingFactor(0.10);
  group.setMaxVelocityScalingFactor(0.10);
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  moveit_visual_tools::MoveItVisualTools visual_tools("world");

  std::vector<double> arm_goal; // 6 radians for joint angles
  std::vector<double> gripper_goal; // 2 radians for gripper angles
  std::vector<double> goal_in_degrees; // n degrees to be converted to radian goal

  group.setStartStateToCurrentState();
  group.setNamedTarget("rest");
  moveit::core::MoveItErrorCode error_code = group.plan(plan);
  error_code = group.execute(plan);

  hand_group.setStartStateToCurrentState();
  group.setNamedTarget("close");
  error_code = hand_group.plan(plan);
  error_code = hand_group.execute(plan);

  // Lean back to grab jacket
  goal_in_degrees = {0, -85, 85, 0, -90, 0};
  arm_goal = deg_to_rad(goal_in_degrees);
  group.setStartStateToCurrentState();
  group.setJointValueTarget(arm_goal);
  error_code = group.plan(plan);
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");
  error_code = group.execute(plan);

  hand_group.setStartStateToCurrentState();
  group.setNamedTarget("open");
  error_code = hand_group.plan(plan);
  error_code = hand_group.execute(plan);

  // ... finish script moves

  return 0;
}