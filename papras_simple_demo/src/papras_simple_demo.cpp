/* Author: Kazuki Shin */

// ROS
#include <ros/ros.h>

// MoveIt!
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

// TF2
#include <eigen_conversions/eigen_msg.h>

std::string tf_prefix_ = "robot1/";

moveit::core::MoveItErrorCode moveToCartesianPose(moveit::planning_interface::MoveGroupInterface &group,
                                                           geometry_msgs::Pose target_pose)
{
    group.setStartStateToCurrentState();
    group.setPoseTarget(target_pose);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    auto error_code = group.plan(my_plan);
    bool success = (error_code == moveit::core::MoveItErrorCode::SUCCESS);

    moveit_visual_tools::MoveItVisualTools visual_tools("world");
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

    ROS_INFO("Move planning (cartesian pose goal) %s", success ? "SUCCESS" : "FAILED");
    if (success)
    {
        error_code = group.execute(my_plan);
    }
    return error_code;
}

moveit::core::MoveItErrorCode moveToNamedPose(moveit::planning_interface::MoveGroupInterface &group,
                                                           std::string named_pose)
{
    robot_state::RobotState start_state(*group.getCurrentState());
    group.setStartState(start_state);
    group.setNamedTarget(named_pose);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    auto error_code = group.plan(my_plan);
    bool success = (error_code == moveit::core::MoveItErrorCode::SUCCESS);

    ROS_INFO("Move planning (named pose goal) %s", success ? "SUCCESS" : "FAILED");
    if (success)
    {
        error_code = group.execute(my_plan);
    }
    return error_code;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "soft_gripper_ft_experiment");
  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::NodeHandle nh;

  ROS_INFO_STREAM("Setting up MoveIt.");

  moveit::planning_interface::MoveGroupInterface arm_group("arm1");
  moveit::planning_interface::MoveGroupInterface gripper_group("gripper1");
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  arm_group.setPlanningTime(45.0);
  arm_group.setPlannerId("RRTConnect");
  arm_group.setMaxAccelerationScalingFactor(0.30);
  arm_group.setMaxVelocityScalingFactor(0.30);
  gripper_group.setMaxAccelerationScalingFactor(0.30);
  gripper_group.setMaxVelocityScalingFactor(0.30);

  ROS_INFO_STREAM("Starting papras_simple_demo");

  moveToNamedPose(arm_group, "init");
  moveToNamedPose(gripper_group, "close");
  moveToNamedPose(arm_group, "rest");
  moveToNamedPose(gripper_group, "open");
  moveToNamedPose(arm_group, "observe_table_left");
  moveToNamedPose(arm_group, "rest");

  ROS_INFO_STREAM("Finished.");
  return 0;
}