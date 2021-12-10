#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_state/conversions.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

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



// The circle constant tau = 2*pi. One tau is one rotation in radians.
const double tau = 2 * M_PI;

#define VEL_SCALE 0.13
#define ACCEL_SCALE 0.13
#define PLANNING_TIME 5
#define PLAN_ATTEMPTS 30
#define USE_CACHE 1

// End-effector links
#define EEF1 "robot1/end_effector_link"
#define EEF3 "robot3/end_effector_link"

#define CACHE_FILE "/home/kimlab/catkin_ws/src/PAPRAS/papras/config/_coffee_demo_cache.yaml" /* CHANGE */ 

namespace YAML {
  template<>
  struct convert<ros::Duration> {
    static bool decode(const Node & node, ros::Duration & result){
      	std::int32_t seconds;
	      std::int32_t nanoseconds;
	      if (!convert<std::int32_t>::decode(node["secs"],  seconds    )) return false;
	      if (!convert<std::int32_t>::decode(node["nsecs"], nanoseconds)) return false;
	        result = ros::Duration(seconds, nanoseconds);
	      return true;
    }
  };

  template<>
  struct convert<ros::Time> {
    static bool decode(const Node & node, ros::Time & result){
      std::int32_t seconds;
      std::int32_t nanoseconds;
      if (!convert<std::int32_t>::decode(node["secs"],  seconds)) return false;
      if (!convert<std::int32_t>::decode(node["nsecs"], nanoseconds)) return false;
      result = ros::Time(seconds, nanoseconds);
      return true;
    }
  };

  template<>
  struct convert<ros::Header> {
    static bool decode(const Node & node, std_msgs::Header & result){
      if (!convert<unsigned int>::decode(node["seq"], result.seq)) return false;
      if (!convert<std::string >::decode(node["frame_id"], result.frame_id)) return false;
      if (!convert<ros::Time>::decode(node["stamp"], result.stamp)) return false;
      return true;
    }
  };



  template<>
  struct convert<trajectory_msgs::JointTrajectoryPoint> {
    static bool decode(const Node & node, trajectory_msgs::JointTrajectoryPoint & result){
      	if (!convert<std::vector<double>>::decode(node["positions"], result.positions)) return false;
        if (!convert<std::vector<double>>::decode(node["velocities"], result.velocities)) return false;
        if (!convert<std::vector<double>>::decode(node["accelerations"], result.accelerations)) return false;
        if (!convert<std::vector<double>>::decode(node["effort"], result.effort)) return false;
        if (!convert<ros::Duration>::decode(node["time_from_start"], result.time_from_start)) return false;
        return true;
    }
  };

  template<>
  struct convert<trajectory_msgs::JointTrajectory> {
    
    static bool decode(const Node& node, trajectory_msgs::JointTrajectory & result) {
	    using TrajectoryPoints = std::vector<trajectory_msgs::JointTrajectoryPoint>;
      using JointNames       = std::vector<std::string>;

      if (!convert<ros::Header>::decode(node["header"], result.header)) return false;
      if (!convert<JointNames>::decode(node["joint_names"], result.joint_names)) return false;
      if (!convert<TrajectoryPoints>::decode(node["points"], result.points)) return false;
      return true;
    }
  };
}

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
// static const std::string PLANNING_GROUP_ARM2 = "arm2";
// static const std::string PLANNING_GROUP_GRIPPER2 = "gripper2";
static const std::string PLANNING_GROUP_ARM3 = "arm3";
static const std::string PLANNING_GROUP_GRIPPER3 = "gripper3";
static const std::string PLANNING_GROUP_ARM1_3 = "arm1_3";


moveit::planning_interface::MoveGroupInterface* move_group_arm1;
moveit::planning_interface::MoveGroupInterface* move_group_gripper1;
// moveit::planning_interface::MoveGroupInterface* move_group_arm2;
// moveit::planning_interface::MoveGroupInterface* move_group_gripper2;
moveit::planning_interface::MoveGroupInterface* move_group_arm3;
moveit::planning_interface::MoveGroupInterface* move_group_gripper3;
moveit::planning_interface::MoveGroupInterface* move_group_arm1_3;

// Pointers to move groups
const moveit::core::JointModelGroup* joint_model_arm1;
const moveit::core::JointModelGroup* joint_model_gripper1;
// const moveit::core::JointModelGroup* joint_model_arm2;
// const moveit::core::JointModelGroup* joint_model_gripper2;
const moveit::core::JointModelGroup* joint_model_arm3;
const moveit::core::JointModelGroup* joint_model_gripper3;
const moveit::core::JointModelGroup* joint_model_arm1_3;

// Double pointers for current move
moveit::planning_interface::MoveGroupInterface** current_move_group;
const moveit::core::JointModelGroup** current_joint_model;

// Visualization
moveit_visual_tools::MoveItVisualTools* visual_tools;

std::stringstream convert_plan_to_cache(moveit::planning_interface::MoveGroupInterface::Plan plan){
    std::stringstream ss;
    ss << "  plan_time: " << plan.planning_time_ << " \n";
    trajectory_msgs::JointTrajectory trajectory = plan.trajectory_.joint_trajectory;
    ss << "  joint_trajectory: \n    header: \n      seq: 0\n      stamp: \n        secs: 0\n        nsecs: 0\n      frame_id: /world\n";
    ss << "    joint_names: ['";
    std::vector<std::string> jns = trajectory.joint_names;
    for (int ii = 0; ii < jns.size()-1; ii++) ss << jns[ii] << "', '";
    ss << jns.back() << "']\n";
 
    ss << "    points: \n";

  for (int i = 0; i < trajectory.points.size(); i++){
    trajectory_msgs::JointTrajectoryPoint jtp = trajectory.points[i];

    ss << "      - \n";
    ss << "        positions: [";
    for (int ii = 0; ii < jtp.positions.size()-1; ii++) ss << jtp.positions[ii] << ", ";
    ss << jtp.positions.back() << "]\n";

    ss << "        velocities: [";
    for (int ii = 0; ii < jtp.velocities.size()-1; ii++) ss << jtp.velocities[ii] << ", ";
    ss << jtp.velocities.back() << "]\n";

    ss << "        accelerations: [";
    for (int ii = 0; ii < jtp.accelerations.size()-1; ii++) ss << jtp.accelerations[ii] << ", ";
    ss << jtp.accelerations.back() << "]\n";

    ss << "        effort: []\n";
    ss << "        time_from_start: \n";
    ss << "          secs: " << jtp.time_from_start.sec << "\n";
    ss << "          nsecs: " << jtp.time_from_start.nsec << "\n";
  }
  
  sensor_msgs::JointState js = plan.start_state_.joint_state;
  ss << "  joint_state: \n    header: \n      seq: 0\n      stamp: \n        secs: 0\n        nsecs: 0\n      frame_id: /world\n";
  ss << "    name: ['";
  std::vector<std::string> js_name = js.name;
  for(int i =0; i < js_name.size()-1; i++){
    ss << js_name[i] <<"', '"; 
  }
  ss << js_name.back() << "']\n";
  ss << "    position: [";
  std::vector<double> pos_js = js.position;
  for(int i =0; i < pos_js.size()-1; i++){
    ss << pos_js[i] << ", ";
  }
  ss << pos_js.back() << "]\n";
  ss << "    velocity: [";
  std::vector<double> vel_js = js.velocity;
  for(int i =0; i < vel_js.size()-1; i++){
    ss << vel_js[i] << ", ";
  }
  ss << vel_js.back() << "]\n";
  

  return ss;
}




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

// unordered_map can be x to y


void plan_execute_arm_move(moveit::planning_interface::MoveGroupInterface** move_group,
                           const moveit::core::JointModelGroup** joint_model,
                           std::string from_to, 
                           std::unordered_map<std::string,moveit::planning_interface::MoveGroupInterface::Plan> &trajectory_cache
                           )
{
  // Create plan object
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  
  // Plan and execute on appropriate arm
  bool success;
  if(!USE_CACHE || trajectory_cache.find(from_to) == std::end(trajectory_cache) || from_to == "ignore"){
    success = ((**move_group).plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if(success){
      trajectory_cache[from_to] = my_plan;
    }
    ROS_INFO("Visualizing plan %s", success ? "" : "FAILED");
    // visual_tools->publishTrajectoryLine(my_plan.trajectory_, (**joint_model).getLinkModel((**move_group).getEndEffectorLink()), *joint_model);
    // visual_tools->trigger();
  }else{
    my_plan = trajectory_cache[from_to];
    moveit::core::robotStateToRobotStateMsg( *((**move_group).getCurrentState()), my_plan.start_state_ );

    ROS_INFO("%s\n","INSIDE CACHE");
    success = true;
  }
  
  if (success) {
    visual_tools->prompt("Press 'next' to execute plan");
    (**move_group).execute(my_plan);
  }
}

void do_multi_arm_pose_move( const std::vector<double> pose_data, const std::string pose_name, std::string from_to, 
                                std::unordered_map<std::string,moveit::planning_interface::MoveGroupInterface::Plan> & trajectory_cache)
{
  current_move_group = &move_group_arm1_3;
  current_joint_model = &joint_model_arm1_3;

  (**current_move_group).setPlanningPipelineId("ompl");
  (**current_move_group).setMaxVelocityScalingFactor(VEL_SCALE);
  (**current_move_group).setMaxAccelerationScalingFactor(ACCEL_SCALE);
  (**current_move_group).setPlanningTime(PLANNING_TIME);
  (**current_move_group).setNumPlanningAttempts(PLAN_ATTEMPTS);

  tf2Scalar arm1_x = pose_data[0];
  tf2Scalar arm1_y = pose_data[1];
  tf2Scalar arm1_z = pose_data[2];
  tf2Scalar arm1_roll = pose_data[3];
  tf2Scalar arm1_pitch = pose_data[4];
  tf2Scalar arm1_yaw = pose_data[5];

  tf2Scalar arm3_x = pose_data[6];
  tf2Scalar arm3_y = pose_data[7];
  tf2Scalar arm3_z = pose_data[8];
  tf2Scalar arm3_roll = pose_data[9];
  tf2Scalar arm3_pitch = pose_data[10];
  tf2Scalar arm3_yaw = pose_data[11];

  // Create goal pose in world frame
  geometry_msgs::Pose arm1_goal_pose = pose_transform(arm1_x, arm1_y, arm1_z, arm1_roll, arm1_pitch, arm1_yaw, 1);
  geometry_msgs::Pose arm3_goal_pose = pose_transform(arm3_x, arm3_y, arm3_z, arm3_roll, arm3_pitch, arm3_yaw, 3);

  move_group_arm1_3->setPoseTarget(arm1_goal_pose, EEF1);
  move_group_arm1_3->setPoseTarget(arm3_goal_pose, EEF3);

  plan_execute_arm_move(current_move_group, current_joint_model,from_to, trajectory_cache);

  move_group_arm1_3->clearPoseTargets();
}

void do_multi_arm_named_move( const std::string pose_name, std::string from_to, 
                                std::unordered_map<std::string,moveit::planning_interface::MoveGroupInterface::Plan> & trajectory_cache)
{
  current_move_group = &move_group_arm1_3;
  current_joint_model = &joint_model_arm1_3;

  (**current_move_group).setPlanningPipelineId("ompl");
  (**current_move_group).setMaxVelocityScalingFactor(VEL_SCALE);
  (**current_move_group).setMaxAccelerationScalingFactor(ACCEL_SCALE);
  (**current_move_group).setPlanningTime(PLANNING_TIME);
  (**current_move_group).setNumPlanningAttempts(PLAN_ATTEMPTS);

  move_group_arm1_3->setNamedTarget(pose_name);

  plan_execute_arm_move(current_move_group, current_joint_model,from_to, trajectory_cache);

}

void do_arm_pose_move(const tf2Scalar& x, const tf2Scalar& y, const tf2Scalar& z,
                      const tf2Scalar& roll, const tf2Scalar& pitch, const tf2Scalar& yaw,
                      const int arm, const std::string pose_name, std::string from_to, std::unordered_map<std::string,moveit::planning_interface::MoveGroupInterface::Plan> & trajectory_cache)
{
  // Assign double pointers for current move group
  switch (arm)
  {
    case 1:
      current_move_group = &move_group_arm1;
      current_joint_model = &joint_model_arm1;
      break;
    // case 2:
    //   current_move_group = &move_group_arm2;
    //   current_joint_model = &joint_model_arm2;
    //   break;
    case 3:
      current_move_group = &move_group_arm3;
      current_joint_model = &joint_model_arm3;
  }
  // Set planning pipeline id to chomp
  (**current_move_group).setPlanningPipelineId("ompl");
  (**current_move_group).setMaxVelocityScalingFactor(VEL_SCALE);
  (**current_move_group).setMaxAccelerationScalingFactor(ACCEL_SCALE);
  (**current_move_group).setPlanningTime(PLANNING_TIME);
  (**current_move_group).setNumPlanningAttempts(PLAN_ATTEMPTS);
  
  // Create goal pose in world frame
  geometry_msgs::Pose goal_pose = pose_transform(x, y, z, roll, pitch, yaw, arm);

  if(USE_CACHE && trajectory_cache.find(from_to) != std::end(trajectory_cache)){
      plan_execute_arm_move(current_move_group, current_joint_model,from_to, trajectory_cache);
  } else {
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
      if (ik_attempts > 100) break;
      success = (**current_move_group).setJointValueTarget(goal_pose, (**current_move_group).getEndEffectorLink());
      ROS_INFO("IK Solver %s", success ? "PASSED" : "FAILED");
      ik_attempts++;
    }

    // Terminal info and pose in Rviz
    ROS_INFO("Move arm %d to pose %s", arm, pose_name.c_str());
    visual_tools->deleteAllMarkers();
    visual_tools->publishAxisLabeled(goal_pose, pose_name);

    // Plan and execute move
    plan_execute_arm_move(current_move_group, current_joint_model,from_to, trajectory_cache);
  }
}

void do_arm_named_move(const int arm, const std::string pose_name, std::string from_to, std::unordered_map<std::string,moveit::planning_interface::MoveGroupInterface::Plan> & trajectory_cache)
{

  // Assign double pointers for current move group
  switch (arm)
  {
    case 1:
      current_move_group = &move_group_arm1;
      current_joint_model = &joint_model_arm1;
      break;
    // case 2:
    //   current_move_group = &move_group_arm2;
    //   current_joint_model = &joint_model_arm2;
    //   break;
    case 3:
      current_move_group = &move_group_arm3;
      current_joint_model = &joint_model_arm3;
  }
  // Set planning pipeline id to chomp
  (**current_move_group).setPlanningPipelineId("ompl");
  (**current_move_group).setMaxVelocityScalingFactor(VEL_SCALE);
  (**current_move_group).setMaxAccelerationScalingFactor(ACCEL_SCALE);
  (**current_move_group).setPlanningTime(PLANNING_TIME);
  (**current_move_group).setNumPlanningAttempts(PLAN_ATTEMPTS);

  // Set joint target for the current move group
  (**current_move_group).setNamedTarget(pose_name);

  // Terminal info
  ROS_INFO("Move arm %d to pose %s", arm, pose_name.c_str());
  visual_tools->deleteAllMarkers();

  // Plan and execute move
  plan_execute_arm_move(current_move_group, current_joint_model,from_to,trajectory_cache);
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
    //   current_move_group = &move_group_gripper2;
    //   current_joint_model = &joint_model_gripper2;
    //   break;
    case 3:
      current_move_group = &move_group_gripper3;
      current_joint_model = &joint_model_gripper3;
  }
  // Set planning pipeline id to ompl
  (**current_move_group).setPlanningPipelineId("ompl");
  (**current_move_group).setMaxVelocityScalingFactor(VEL_SCALE);
  (**current_move_group).setMaxAccelerationScalingFactor(ACCEL_SCALE);
  (**current_move_group).setPlanningTime(PLANNING_TIME);
  (**current_move_group).setNumPlanningAttempts(PLAN_ATTEMPTS);
  
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
    // case 2:
    //   current_move_group = &move_group_gripper2;
    //   current_joint_model = &joint_model_gripper2;
    //   break;
    case 3:
      current_move_group = &move_group_gripper3;
      current_joint_model = &joint_model_gripper3;
  }
  // Set planning pipeline id to ompl
  (**current_move_group).setPlanningPipelineId("ompl");
  (**current_move_group).setMaxVelocityScalingFactor(VEL_SCALE);
  (**current_move_group).setMaxAccelerationScalingFactor(ACCEL_SCALE);
  (**current_move_group).setPlanningTime(PLANNING_TIME);
  (**current_move_group).setNumPlanningAttempts(PLAN_ATTEMPTS);
  
  
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
  std::unordered_map<std::string,moveit::planning_interface::MoveGroupInterface::Plan> trajectory_cache;
  const std::string node_name = "coffee_demo";
  ros::init(argc, argv, node_name);
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  if(USE_CACHE){
    YAML::Node root = YAML::LoadFile(CACHE_FILE);
    for (YAML::const_iterator it=root.begin();it!=root.end();++it) {
      std::string name = it->first.as<std::string>();
      moveit::planning_interface::MoveGroupInterface::Plan my_cached_plan;
      YAML::Node node = it->second;
      my_cached_plan.planning_time_ = node["plan_time"].as<double>();
      
      trajectory_msgs::JointTrajectory trajectory = node["joint_trajectory"].as<trajectory_msgs::JointTrajectory>();
      my_cached_plan.trajectory_.joint_trajectory = trajectory;

      trajectory_cache[name] = my_cached_plan;
  
     // ROS_INFO("%s\n",name.c_str());
    }
    //ROS_INFO("HERE");
  }

  //****************************************************************************
  // Set up planning interface
  // Set up move group planning interfaces
  move_group_arm1 = new moveit::planning_interface::MoveGroupInterface(PLANNING_GROUP_ARM1);
  move_group_gripper1 = new moveit::planning_interface::MoveGroupInterface(PLANNING_GROUP_GRIPPER1);
  // move_group_arm2 = new moveit::planning_interface::MoveGroupInterface(PLANNING_GROUP_ARM2);
  // move_group_gripper2 = new moveit::planning_interface::MoveGroupInterface(PLANNING_GROUP_GRIPPER2);
  move_group_arm3 = new moveit::planning_interface::MoveGroupInterface(PLANNING_GROUP_ARM3);
  move_group_gripper3 = new moveit::planning_interface::MoveGroupInterface(PLANNING_GROUP_GRIPPER3);
  move_group_arm1_3 = new moveit::planning_interface::MoveGroupInterface(PLANNING_GROUP_ARM1_3);

  // Set up planning scene to add/remove collision objects in world
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // Create pointers to planning groups
  joint_model_arm1 = move_group_arm1->getCurrentState()->getJointModelGroup(PLANNING_GROUP_ARM1);
  joint_model_gripper1 = move_group_gripper1->getCurrentState()->getJointModelGroup(PLANNING_GROUP_GRIPPER1);
  // joint_model_arm2 = move_group_arm2->getCurrentState()->getJointModelGroup(PLANNING_GROUP_ARM2);
  // joint_model_gripper2 = move_group_gripper2->getCurrentState()->getJointModelGroup(PLANNING_GROUP_GRIPPER2);
  joint_model_arm3 = move_group_arm3->getCurrentState()->getJointModelGroup(PLANNING_GROUP_ARM3);
  joint_model_gripper3 = move_group_gripper3->getCurrentState()->getJointModelGroup(PLANNING_GROUP_GRIPPER3);
  joint_model_arm1_3 = move_group_arm1_3->getCurrentState()->getJointModelGroup(PLANNING_GROUP_ARM1_3);

  // Print reference information
  ROS_INFO("Arm 1 planning frame: %s", move_group_arm1->getPlanningFrame().c_str());
  ROS_INFO("Arm 1 end effector link: %s", move_group_arm1->getEndEffectorLink().c_str());
  // ROS_INFO("Arm 2 planning frame: %s", move_group_arm2->getPlanningFrame().c_str());
  // ROS_INFO("Arm 2 end effector link: %s", move_group_arm2->getEndEffectorLink().c_str());
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
  do_multi_arm_named_move("rest","ignore",trajectory_cache);

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

    std::string from_to = move_name + "_to_";
    if(it +1 != std::end(moves_list)){
      from_to += (*(it+1));
    }else {
      from_to += "end";
    }
    // ROS_INFO("Move found: %s, control_group: %s, pose_name: %s", move_name.c_str(), control_group.c_str(), pose_name.c_str());

    // Get pose data
    std::vector<double> pose_data;
    node_handle.getParam("/"+move_name, pose_data);

    // Do appropriate move
    if (control_group == "arm1_3"){
      do_multi_arm_pose_move(pose_data, pose_name, from_to, trajectory_cache);
    }
    else if (control_name == "arm") {
      do_arm_pose_move(pose_data.at(0), pose_data.at(1), pose_data.at(2), pose_data.at(3), pose_data.at(4), pose_data.at(5), std::stoi(control_id), pose_name, from_to, trajectory_cache);
    } else if (control_name == "gripper") {
      do_gripper_angle_move(pose_data.at(0), std::stoi(control_id));
    } else if (control_group == "sleep") {
      ros::Duration(pose_data.at(0)).sleep();
    }
    else {} // Do nothing
  }

  //****************************************************************************
  // Exit
  do_multi_arm_named_move("rest","rest1_3",trajectory_cache);

  if(!USE_CACHE){
    std::ofstream os(CACHE_FILE);
    // YAML::Node node;
    for(std::unordered_map<std::string,moveit::planning_interface::MoveGroupInterface::Plan>::iterator it = std::begin(trajectory_cache); it != std::end(trajectory_cache); it++){
      // chatter_pub.publish((it->second));
      // ROS_INFO((it->second));
      os << it->first <<":\n";
      std::stringstream ss = convert_plan_to_cache(it->second);
      std::string value = ss.str();
      os << value;
      // os << ss.rdbuf;
    }
  }

  // Write to a yaml file with helper functions.
  ros::shutdown();
  return 0;
}