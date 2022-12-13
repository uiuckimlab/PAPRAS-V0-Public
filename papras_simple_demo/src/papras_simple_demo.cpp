/* Author: Kazuki Shin */

// ROS
#include <ros/ros.h>

// MoveIt!
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

// import geometric_shapes SolidPrimitiveDimCount
#include <geometric_shapes/solid_primitive_dims.h>
#include <moveit_msgs/GetPositionIK.h>
#include <moveit_msgs/PositionIKRequest.h>

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

moveit::core::MoveItErrorCode moveToJointPose(moveit::planning_interface::MoveGroupInterface &group,
                                                           std::vector<double> joint_pose)
{
    robot_state::RobotState start_state(*group.getCurrentState());
    group.setStartState(start_state);
    group.setJointValueTarget(joint_pose);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    auto error_code = group.plan(my_plan);
    bool success = (error_code == moveit::core::MoveItErrorCode::SUCCESS);

    ROS_INFO("Move planning (joint pose goal) %s", success ? "SUCCESS" : "FAILED");
    if (success)
    {
        error_code = group.execute(my_plan);
    }
    return error_code;
}

// find IK solutions for a target pose without planning or using setPoseTarget() or kinematic_state->setFromIK()
moveit::core::MoveItErrorCode findIKSolutions(moveit::planning_interface::MoveGroupInterface &group,
                                                           geometry_msgs::Pose target_pose,
                                                           std::vector<std::vector<double>> &ik_solutions)
{
    // get the current state
    robot_state::RobotState start_state(*group.getCurrentState());

    // create a robot state
    robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(start_state));

    // get the joint model group
    const robot_state::JointModelGroup *joint_model_group = kinematic_state->getJointModelGroup(group.getName());

    // get the current joint values
    std::vector<double> joint_values;
    kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);

    // find IK solutions using setFromIK()
    bool found_ik = kinematic_state->setFromIK(joint_model_group, target_pose, 10, 0.1);

    if (found_ik)
    {
        kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
        ik_solutions.push_back(joint_values);
    }
    else
    {
        ROS_INFO("Did not find IK solution");
        return moveit::core::MoveItErrorCode::FAILURE;
    }


    return moveit::core::MoveItErrorCode::SUCCESS;
}

int test_findIKSolutions_moveToJointPose(moveit::planning_interface::MoveGroupInterface &group)
{
    geometry_msgs::Pose target_pose;
    target_pose.orientation.w = 1.0;
    target_pose.position.x = 0.28;
    target_pose.position.y = -0.2;
    target_pose.position.z = 0.5;

    std::vector<std::vector<double>> ik_solutions;
    auto error_code = findIKSolutions(group, target_pose, ik_solutions);
    bool success = (error_code == moveit::core::MoveItErrorCode::SUCCESS);

    ROS_INFO("Find IK solutions %s", success ? "SUCCESS" : "FAILED");
    if (success)
    {
        for (auto &ik_solution : ik_solutions)
        {
            for (auto &joint_value : ik_solution)
            {
                ROS_INFO("Joint value: %f", joint_value);
            }
            error_code = moveToJointPose(group, ik_solution);
            success = (error_code == moveit::core::MoveItErrorCode::SUCCESS);
            ROS_INFO("Move to joint pose %s", success ? "SUCCESS" : "FAILED");
        }  
    }

    return 0;
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

  moveToNamedPose(gripper_group, "close");
  moveToNamedPose(arm_group, "rest");
  
  test_findIKSolutions_moveToJointPose(arm_group);

  ROS_INFO_STREAM("Finished.");
  return 0;

}