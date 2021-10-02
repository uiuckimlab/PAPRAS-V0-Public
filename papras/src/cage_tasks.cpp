#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/RobotTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>
#include <std_msgs/String.h>

// For HTMs
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Vector3.h>

// For quaternions
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <unordered_map>
#include <fstream> 
#include <sstream>
#include <cstdint>
#include <yaml-cpp/yaml.h>

// End-effector links
#define EEF1 "robot1/end_effector_link"
#define EEF2 "robot2/end_effector_link"
#define EEF3 "robot3/end_effector_link"
#define EEF4 "robot4/end_effector_link"
#define USE_CACHE 1


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
static const std::string PLANNING_GROUP_ARM1_2_3_4 = "arm1_2_3_4";
static const std::string PLANNING_GROUP_ARM1_4 = "arm1_4";

moveit::planning_interface::MoveGroupInterface* move_group_arm1_2;
moveit::planning_interface::MoveGroupInterface* move_group_arm2_3;
moveit::planning_interface::MoveGroupInterface* move_group_arm1_3;
moveit::planning_interface::MoveGroupInterface* move_group_arm1_2_3;
moveit::planning_interface::MoveGroupInterface* move_group_arm1_2_3_4;
moveit::planning_interface::MoveGroupInterface* move_group_arm1_4;

// Pointers to move groups
const moveit::core::JointModelGroup* joint_model_arm1_2;
const moveit::core::JointModelGroup* joint_model_arm2_3;
const moveit::core::JointModelGroup* joint_model_arm1_3;
const moveit::core::JointModelGroup* joint_model_arm1_2_3;
const moveit::core::JointModelGroup* joint_model_arm1_2_3_4;
const moveit::core::JointModelGroup* joint_model_arm1_4;

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

void plan_execute_arm_move(moveit::planning_interface::MoveGroupInterface* move_group, std::string from_to,  std::unordered_map<std::string,moveit::planning_interface::MoveGroupInterface::Plan> &trajectory_cache
)
{
  // Create plan object
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  move_group->setMaxVelocityScalingFactor(0.07);
  move_group->setMaxAccelerationScalingFactor(0.07);
  move_group->setPlanningTime(1);
  move_group->setNumPlanningAttempts(10);
  
  // Plan and execute on appropriate arm
  bool success;
  if(!USE_CACHE && trajectory_cache.find(from_to) != std::end(trajectory_cache)){
      success = (move_group->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  }else{
    my_plan = trajectory_cache[from_to];
    ROS_INFO("%s\n","INSIDE CACHE");
  }
  // moveit_msgs::DisplayTrajectory display_trajectory;
  // display_trajectory.trajectory_start = my_plan.start_state_;
  // display_trajectory.trajectory = my_plan.trajectory_;
  if(success){
    trajectory_cache[from_to] = my_plan;
  }
  ROS_INFO("Visualizing plan %s", success ? "" : "FAILED");
  if (success) {
    visual_tools->prompt("Press 'next' to execute plan");
    move_group->execute(my_plan);
  }
}

int main(int argc, char** argv)
{
  // Setup node and start AsyncSpinner
  std::ofstream fout("test.yaml");

  std::unordered_map<std::string,moveit::planning_interface::MoveGroupInterface::Plan> trajectory_cache;
  const std::string node_name = "test_multi";
  ros::init(argc, argv, node_name);
  ros::NodeHandle node_handle;
  // ros::Publisher chatter_pub = node_handle.advertise<moveit::planning_interface::MoveGroupInterface::Plan>("chatter", 1000);
  ros::AsyncSpinner spinner(1);
  spinner.start();

    if(USE_CACHE){
      YAML::Node root = YAML::LoadFile("/home/sankalp/manipulator_p_workspace/catkin_ws/test.yaml");
      for (YAML::const_iterator it=root.begin();it!=root.end();++it) {
        std::string name = it->first.as<std::string>();
        moveit::planning_interface::MoveGroupInterface::Plan my_cached_plan;
        YAML::Node node = it->second;
        my_cached_plan.planning_time_ = node["plan_time"].as<double>();
        
        trajectory_msgs::JointTrajectory trajectory = node["joint_trajectory"].as<trajectory_msgs::JointTrajectory>();
        my_cached_plan.trajectory_.joint_trajectory = trajectory;
        
        // sensor_msgs::JointState js = node["joint_state"].as<sensor_msgs::JointState>();
        // my_cached_plan.start_state_.joint_state = js;

        trajectory_cache[name] = my_cached_plan;
        // for(YAML::const_iterator it2=node.begin();it2!=node.end();++it2){
        //    std::string name_2 = it2->first.as<std::string>();
        //     ROS_INFO("I: %s\n",name_2.c_str());
        // } 
        // my_cached_plan.start_state_ = inner["joint_state"];
        ROS_INFO("%s\n",name.c_str());
      }
      ROS_INFO("HERE");
      // return 0;
    }

  
  //****************************************************************************
  // Set up planning interface
  // Set up move group planning interfaces
  move_group_arm1_2 = new moveit::planning_interface::MoveGroupInterface(PLANNING_GROUP_ARM1_2);
  move_group_arm2_3 = new moveit::planning_interface::MoveGroupInterface(PLANNING_GROUP_ARM2_3);
  move_group_arm1_3 = new moveit::planning_interface::MoveGroupInterface(PLANNING_GROUP_ARM1_3);
  move_group_arm1_2_3 = new moveit::planning_interface::MoveGroupInterface(PLANNING_GROUP_ARM1_2_3);
  move_group_arm1_2_3_4 = new moveit::planning_interface::MoveGroupInterface(PLANNING_GROUP_ARM1_2_3_4);
  move_group_arm1_4 = new moveit::planning_interface::MoveGroupInterface(PLANNING_GROUP_ARM1_4);

  // Set up planning scene to add/remove collision objects in world
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // Create pointers to planning groups
  joint_model_arm1_2 = move_group_arm1_2->getCurrentState()->getJointModelGroup(PLANNING_GROUP_ARM1_2);
  joint_model_arm2_3 = move_group_arm2_3->getCurrentState()->getJointModelGroup(PLANNING_GROUP_ARM2_3);
  joint_model_arm1_3 = move_group_arm1_3->getCurrentState()->getJointModelGroup(PLANNING_GROUP_ARM1_3);
  joint_model_arm1_2_3 = move_group_arm1_2_3->getCurrentState()->getJointModelGroup(PLANNING_GROUP_ARM1_2_3);
  joint_model_arm1_2_3 = move_group_arm1_2_3_4->getCurrentState()->getJointModelGroup(PLANNING_GROUP_ARM1_2_3_4);
  joint_model_arm1_2_3 = move_group_arm1_4->getCurrentState()->getJointModelGroup(PLANNING_GROUP_ARM1_4);

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
  move_group_arm1_2_3_4->setNamedTarget("rest");
  plan_execute_arm_move(move_group_arm1_2_3_4,"begin",trajectory_cache);

  // Load list of moves from ROS Parameter Server
  std::vector<std::string> moves_list;
  node_handle.getParam("/cage_moves", moves_list);

  for (std::vector<std::string>::iterator it = std::begin(moves_list); it != std::end(moves_list); ++it)
  {
    // String parsing
    std::string move_name = *it;
    std::string move_group = move_name.substr(0, move_name.find("/"));
    std::string pose_name = move_name.substr(move_name.find("/")+1);
    // ROS_INFO("Move found: %s, control_group: %s, pose_name: %s", move_name.c_str(), control_group.c_str(), pose_name.c_str());

    std::string from_to = move_name + "_to_";
    if(it +1 != std::end(moves_list)){
      from_to += (*(it+1));
    }else {
      from_to += "end";
    }

    // Get pose data
    std::vector<double> pose_data;
    node_handle.getParam("/"+move_name, pose_data);

    // Do appropriate move
    if(move_group=="arm1_2_3_4"){
      std::vector<double> joint_group_positions = pose_data;
      move_group_arm1_2_3_4->setJointValueTarget(joint_group_positions);
      plan_execute_arm_move(move_group_arm1_2_3_4, from_to,trajectory_cache);
    }
    if(move_group=="arm1_4"){
      std::vector<double> joint_group_positions = pose_data;
      move_group_arm1_4->setJointValueTarget(joint_group_positions);
      plan_execute_arm_move(move_group_arm1_4, from_to,trajectory_cache);
    }
  }
  if(!USE_CACHE){
    std::ofstream os("/home/sankalp/manipulator_p_workspace/catkin_ws/test.yaml");
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

  // if(os.is_open()){
  //   ROS_INFO("HI");
  // }
  // os << node;

  move_group_arm1_2_3_4->setNamedTarget("rest");
  plan_execute_arm_move(move_group_arm1_2_3_4, "end",trajectory_cache);
  ros::shutdown();
  return 0;
}