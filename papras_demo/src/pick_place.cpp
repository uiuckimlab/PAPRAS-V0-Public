/* Authors: Kazuki Shin, Sankalp Yamsani */

// ROS
#include <ros/ros.h>

// MoveIt!
#include <geometric_shapes/solid_primitive_dims.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

// TF2
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>

// eigen_conversions
#include <eigen_conversions/eigen_msg.h>

// Other
#include <sstream>
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>
#include <std_msgs/Float32.h>
#include <vision_msgs/Detection3DArray.h>
#include <rosparam_shortcuts/rosparam_shortcuts.h>

// Gazebo Object Spawn
#include "gazebo_msgs/SpawnModel.h"
#include "geometry_msgs/Pose.h"
#include <tinyxml.h>
#include <sstream>

// Grasp 
// #include <moveit_grasps/two_finger_grasp_generator.h>
// #include <moveit_grasps/two_finger_grasp_data.h>
// #include <moveit_grasps/two_finger_grasp_filter.h>
// #include <moveit_grasps/grasp_planner.h>
// #include <moveit_grasps/grasp_generator.h>

// Aruco
#include <aruco_msgs/Marker.h>
#include <aruco_msgs/MarkerArray.h>

//Action Client
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

std::string tf_prefix_ = "robot1/";
constexpr char LOGNAME[] = "moveit_task_constructor_papras";
// The circle constant tau = 2*pi. One tau is one rotation in radians.
const double tau = 2 * M_PI;

#define SPAWN_URDF_OBJECT_TOPIC "gazebo/spawn_urdf_model"
#define SPAWN_SDF_OBJECT_TOPIC "gazebo/spawn_sdf_model"

enum state
{
    ST_INIT,
    ST_PAUSED,
    ST_ARM_TO_REST_START,
    ST_INIT_SIM_OBJ,
    ST_CAPTURE_OBJ,
    ST_PICK_OBJ,
    ST_ARM_TO_TRANSPORT,
    ST_PLACE_OBJ,
    ST_ARM_TO_HOME_END,
    ST_DONE
};

enum ObjectID
{
    CUP = 0,
    SPOON = 11,
    BOWL = 2,
    SPRITE = 3,
    LACROIX = 4,
    OJ = 5,
    BUTTER = 6,
    PINK_CUP = 7,
    PLATE = 12,
    BOTTLE = 13,
    ABETSOUP = 14,
    TABLE = 16,
};

auto OBJECT_TO_GRASP = ObjectID::ABETSOUP;

std::string id_to_string(int id)
{
    switch (id)
    {
    case ObjectID::CUP:
        return "cup";
    case ObjectID::SPOON:
        return "spoon";
    case ObjectID::BOWL:
        return "bowl";
    case ObjectID::SPRITE:
        return "sprite";
    case ObjectID::LACROIX:
        return "lacroix";
    case ObjectID::OJ:
        return "oJ";
    case ObjectID::BUTTER:
        return "butter";
    case ObjectID::PINK_CUP:
        return "pinkCup";
    case ObjectID::ABETSOUP:
        return "abetSoup";
    case ObjectID::PLATE:
        return "plate";
    case ObjectID::BOTTLE:
        return "bottle";
    case ObjectID::TABLE:
        return "table";
    default:
        std::cerr << "No such ObjectID: " << id << std::endl;
        return "";
    }
}

struct GrapsPoseDefine
{
  Eigen::Isometry3d grasp_pose;
  std::float_t gripper_width;
};

bool paused = false;
bool failed = false;

void initCollisionObject(moveit::planning_interface::PlanningSceneInterface &planning_scene_interface){
  std::vector<moveit_msgs::CollisionObject> collision_objects;

  // Add table 
  moveit_msgs::CollisionObject co;
  co.header.frame_id = "/world";
  co.id = id_to_string(ObjectID::TABLE);
  co.operation = moveit_msgs::CollisionObject::ADD;
  co.primitives.resize(1);
  co.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
  co.primitives[0].dimensions.resize(geometric_shapes::solidPrimitiveDimCount<shape_msgs::SolidPrimitive::BOX>());
  co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = 1;
  co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = 2;
  co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = 0.78;
  co.primitive_poses.resize(1);
  co.primitive_poses[0].position.x = 0; 
  co.primitive_poses[0].position.y = 0;
  co.primitive_poses[0].position.z = 0.38;
  co.primitive_poses[0].orientation.z = 0;
  co.primitive_poses[0].orientation.w = 1.0;

  collision_objects.push_back(co);

  planning_scene_interface.applyCollisionObjects(collision_objects);
}

void openGripper(trajectory_msgs::JointTrajectory& posture)
{
  posture.joint_names.resize(1);
  posture.joint_names[0] = tf_prefix_ + "gripper";
  posture.points.resize(1);
  posture.points[0].positions.resize(1);
  posture.points[0].positions[0] = 0;
  posture.points[0].time_from_start.fromSec(5.0);
}

void closedGripper(trajectory_msgs::JointTrajectory& posture, std::float_t gripper_width = 0.1)
{
  posture.joint_names.resize(1);
  posture.joint_names[0] = tf_prefix_ + "gripper";
  posture.points.resize(1);
  posture.points[0].positions.resize(1);
  posture.points[0].positions[0] = 1;
  posture.points[0].time_from_start.fromSec(5.0);
}


int shake(float theta, float duration, bool isPitch, actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> &ac){

  ROS_INFO("IN SHAKE");

  control_msgs::FollowJointTrajectoryGoal goal;

  goal.trajectory.joint_names = {tf_prefix_+"joint1", tf_prefix_+"joint2", tf_prefix_+"joint3",tf_prefix_+"joint4", tf_prefix_+"joint5", tf_prefix_+"joint6"};

  // shake 1
  goal.trajectory.points.resize(1);
  if (isPitch)
    goal.trajectory.points[0].positions =  {0.0, 0.0, 0.0, 0.0, theta / 2.0, 0.0};
  else 
    goal.trajectory.points[0].positions = {0.0, 0.0, 0.0, 0.0, 0.0, theta / 2.0};
  goal.trajectory.points[0].time_from_start = ros::Duration(duration / 2.0);

  ac.sendGoal(goal);
  ac.waitForResult(ros::Duration(0.1));
  ros::Duration(duration / 2.0 + 0.1).sleep();


  // shake 2
  if (isPitch)
    goal.trajectory.points[0].positions =  {0.0, 0.0, 0.0, 0.0, -theta / 2.0, 0.0};
  else 
    goal.trajectory.points[0].positions = {0.0, 0.0, 0.0, 0.0, 0.0, -theta / 2.0};
  goal.trajectory.points[0].time_from_start = ros::Duration(duration / 2.0);

  ac.sendGoal(goal);
  ac.waitForResult(ros::Duration(0.1));
  ros::Duration(duration / 2.0 + 0.1).sleep();
    
  return 0;

}

int pick_place_object(moveit::planning_interface::MoveGroupInterface& group, moveit::planning_interface::MoveGroupInterface& hand_group, ObjectID object_id){
  // get pose of detected object
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  group.setPlanningTime(10.0);
  group.setNumPlanningAttempts(10.0);
  group.setMaxAccelerationScalingFactor(0.05);
  group.setMaxVelocityScalingFactor(0.05);
  group.setPlannerId("RRTConnect");
  std::string frame_id = tf_prefix_ + "camera_link";
  std::string object_name = id_to_string(object_id);

  geometry_msgs::Pose objPose;
  objPose = planning_scene_interface.getObjectPoses({object_name}).at(object_name);
  geometry_msgs::PoseStamped pre_grasp_pose;

  // <<< ----- PRE GRASP  ----- >>>
  pre_grasp_pose = group.getCurrentPose();
  switch(object_id){
    case ObjectID::BOWL :
      pre_grasp_pose.pose.position.x = objPose.position.x + 0.05; 
      pre_grasp_pose.pose.position.y = objPose.position.y + 0.20;
      pre_grasp_pose.pose.position.z = objPose.position.z + 0.01;
      pre_grasp_pose.pose.orientation.x = 0.2347;
      pre_grasp_pose.pose.orientation.y = 0.2274;
      pre_grasp_pose.pose.orientation.z = -0.6677;
      pre_grasp_pose.pose.orientation.w = 0.6689;
      break;
    case ObjectID::BOTTLE:
      pre_grasp_pose.pose.position.x = objPose.position.x + 0.07;
      pre_grasp_pose.pose.position.y = objPose.position.y + 0.02;
      pre_grasp_pose.pose.position.z = objPose.position.z + 0.02;
      pre_grasp_pose.pose.orientation.x = -0.35939;
      pre_grasp_pose.pose.orientation.y = 0.61076;
      pre_grasp_pose.pose.orientation.z = 0.61211;
      pre_grasp_pose.pose.orientation.w = -0.35090;
      break;
    case ObjectID::ABETSOUP:
    case ObjectID::OJ:
    case ObjectID::SPRITE:
    case ObjectID::CUP :
      pre_grasp_pose.pose.position.x = objPose.position.x + 0.07;
      pre_grasp_pose.pose.position.y = objPose.position.y + 0.02;
      pre_grasp_pose.pose.position.z = objPose.position.z + 0.04;
      break;
    case ObjectID::SPOON :
      pre_grasp_pose.pose.position.x = objPose.position.x + 0.02; 
      pre_grasp_pose.pose.position.y = objPose.position.y - 0.10;
      pre_grasp_pose.pose.position.z = objPose.position.z + 0.03;
      pre_grasp_pose.pose.orientation.x = -0.70711;
      pre_grasp_pose.pose.orientation.y = -0.00162;
      pre_grasp_pose.pose.orientation.z = 0.70711;
      pre_grasp_pose.pose.orientation.w = -0.00161;
      break;
    case ObjectID::BUTTER :
      pre_grasp_pose.pose.position.x = objPose.position.x + 0.03;
      pre_grasp_pose.pose.position.y = objPose.position.y;
      pre_grasp_pose.pose.position.z = objPose.position.z + 0.03;
      pre_grasp_pose.pose.orientation.x = -0.70711;
      pre_grasp_pose.pose.orientation.y = -0.00162;
      pre_grasp_pose.pose.orientation.z = 0.70711;
      pre_grasp_pose.pose.orientation.w = -0.00161;
      break;
    case ObjectID::PLATE :
      pre_grasp_pose.pose.position.x = objPose.position.x + 0.10; 
      pre_grasp_pose.pose.position.y = objPose.position.y + 0.30;
      pre_grasp_pose.pose.position.z = objPose.position.z - 0.03;
      pre_grasp_pose.pose.orientation.x = -0.00545;
      pre_grasp_pose.pose.orientation.y = -0.00506;
      pre_grasp_pose.pose.orientation.z = -0.86334;
      pre_grasp_pose.pose.orientation.w = 0.50457;
      break;
    default:
      ROS_INFO("object not in list");
      return 0;
  }
  
  group.setStartState(*group.getCurrentState());
  group.setPoseTarget(pre_grasp_pose.pose);
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;


  hand_group.setNamedTarget("open");
  hand_group.plan(my_plan);
  hand_group.execute(my_plan);
  ros::Duration(3).sleep();

  bool success = (group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("tutorial", "Successful pregrasp plan %s", success ? "" : "FAILED");
  moveit_visual_tools::MoveItVisualTools visual_tools("world");
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");
  group.execute(my_plan);
  group.attachObject(object_name, frame_id);
  ros::Duration(0.5).sleep();

  // <<< ----- GRASP  ----- >>>
  geometry_msgs::PoseStamped grasp_pose;
  grasp_pose = group.getCurrentPose();
  switch(object_id){
    case ObjectID::BOWL :
      grasp_pose.pose.position.x = pre_grasp_pose.pose.position.x;
      grasp_pose.pose.position.y = pre_grasp_pose.pose.position.y - 0.05;
      grasp_pose.pose.position.z = pre_grasp_pose.pose.position.z - 0.015;
      break;
    case ObjectID::OJ:
      grasp_pose.pose.position.x = pre_grasp_pose.pose.position.x;
      grasp_pose.pose.position.y = pre_grasp_pose.pose.position.y - 0.05;
      grasp_pose.pose.position.z = pre_grasp_pose.pose.position.z - 0.02;
      break;
    case ObjectID::BOTTLE:
      grasp_pose.pose.position.x = pre_grasp_pose.pose.position.x - 0.02;
      grasp_pose.pose.position.y = pre_grasp_pose.pose.position.y - 0.07;
      grasp_pose.pose.position.z = pre_grasp_pose.pose.position.z;
      break;
    case ObjectID::ABETSOUP:
    case ObjectID::SPRITE:
    case ObjectID::CUP :
      grasp_pose.pose.position.x = pre_grasp_pose.pose.position.x;
      grasp_pose.pose.position.y = pre_grasp_pose.pose.position.y - 0.07;
      grasp_pose.pose.position.z = pre_grasp_pose.pose.position.z - 0.02;
      break;
    case ObjectID::SPOON :
      grasp_pose.pose.position.x = pre_grasp_pose.pose.position.x;
      grasp_pose.pose.position.y = pre_grasp_pose.pose.position.y;
      grasp_pose.pose.position.z = pre_grasp_pose.pose.position.z - 0.03;
      break;
    case ObjectID::BUTTER :
      grasp_pose.pose.position.x = pre_grasp_pose.pose.position.x;
      grasp_pose.pose.position.y = pre_grasp_pose.pose.position.y;
      grasp_pose.pose.position.z = pre_grasp_pose.pose.position.z - 0.03;
      break;
    case ObjectID::PLATE : 
      grasp_pose.pose.position.x = pre_grasp_pose.pose.position.x - 0.05;
      grasp_pose.pose.position.y = pre_grasp_pose.pose.position.y - 0.08;
      grasp_pose.pose.position.z = pre_grasp_pose.pose.position.z;
      break;
    default:
      ROS_INFO("object not in list");
      return 0;
  }

  group.setStartState(*group.getCurrentState());
  group.setPoseTarget(grasp_pose);
  success = (group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("tutorial", "Successful grasp plan %s", success ? "" : "FAILED");
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");
  group.execute(my_plan);

  hand_group.setNamedTarget("close_tight");
  hand_group.plan(my_plan);
  hand_group.execute(my_plan);

  group.setStartState(*group.getCurrentState());
  group.setNamedTarget("init");
  group.plan(my_plan);
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui");
  group.execute(my_plan);

  actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ac("/arm1_controller/follow_joint_trajectory",true);
  ac.waitForServer();

  shake(0.2, 0.05, true, ac);
  shake(0.2, 0.05, true, ac);
  shake(0.2, 0.05, true, ac);

  shake(0.6, 0.4, false, ac);
  shake(0.6, 0.4, false, ac);
  shake(0.6, 0.4, false, ac);

  group.setStartState(*group.getCurrentState());
  group.setPoseTarget(grasp_pose);
  success = (group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("tutorial", "Successful grasp plan %s", success ? "" : "FAILED");
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");
  group.execute(my_plan);

  hand_group.setStartStateToCurrentState();
  hand_group.setNamedTarget("open");
  hand_group.plan(my_plan);
  hand_group.execute(my_plan);

  group.setStartState(*group.getCurrentState());
  group.setPoseTarget(pre_grasp_pose);
  success = (group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("tutorial", "Successful grasp plan %s", success ? "" : "FAILED");
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");
  group.execute(my_plan);

  group.setStartState(*group.getCurrentState());
  group.setNamedTarget("rest");
  group.plan(my_plan);
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui");
  group.execute(my_plan);

  // detach all objects
  auto attached_objects = planning_scene_interface.getAttachedObjects();
  for (auto &&object : attached_objects)
  {
      group.detachObject(object.first);
  }
  return 0;
}


moveit::core::MoveItErrorCode pick(moveit::planning_interface::MoveGroupInterface& group)
{
  std::vector<moveit_msgs::Grasp> grasps;

  // --- calculate grasps
  // this is using standard frame orientation: x forward, y left, z up, relative to object bounding box center

  std::vector<GrapsPoseDefine> grasp_poses;


  for(double x = -0.03; x <= 0.03; x+= 0.01){
    for(double y = -0.02; y <= 0; y+= 0.01){
      for(double z = -0.05; z <= 0.05; z+= 0.01){

        {
          GrapsPoseDefine grasp_pose_define;
          grasp_pose_define.grasp_pose = Eigen::Isometry3d::Identity();
          grasp_pose_define.grasp_pose.translate(Eigen::Vector3d(x, z, 0.0));
          grasp_pose_define.grasp_pose.rotate(Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d(0.5d, 0.0d, 0.5d)));
          grasp_pose_define.gripper_width = 0.0;
          grasp_poses.push_back(grasp_pose_define);
        }


      }
    }
  }
  

  for (auto&& grasp_pose : grasp_poses)
  {
    // rotate grasp pose from CAD model orientation to standard orientation (x forward, y left, z up)
    // Eigen quaternion = wxyz, not xyzw
    // Eigen::Isometry3d bbox_center_rotated = Eigen::Isometry3d::Identity();
    // bbox_center_rotated.rotate(Eigen::AngleAxisd(M_PI, Eigen::Vector3d(0.0d, 1.0d, 0.0d)));
    // bbox_center_rotated.rotate(Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d(1.0d, 0.0d, 0.0d)));

    // --- calculate desired pose of gripper_tcp (in cracker frame) when grasping
    geometry_msgs::PoseStamped p;
    p.header.frame_id = "abetSoup";
    tf::poseEigenToMsg(grasp_pose.grasp_pose, p.pose);
    ROS_DEBUG_STREAM("Grasp pose:\n" << p.pose);
    // p.
    moveit_msgs::Grasp g;

    g.grasp_pose = p;
    g.grasp_quality = 1.0;
    ROS_INFO_STREAM("Grasp pose:\n" << p.pose);

    g.pre_grasp_approach.direction.vector.x = 1.0;
    g.pre_grasp_approach.direction.header.frame_id = tf_prefix_ + "/end_effector_link";
    g.pre_grasp_approach.min_distance = 0.01;
    g.pre_grasp_approach.desired_distance = 0.05;

    g.post_grasp_retreat.direction.header.frame_id = "world";
    g.post_grasp_retreat.direction.vector.z = 1.0;
    g.post_grasp_retreat.min_distance = 0.1;
    g.post_grasp_retreat.desired_distance = 0.15;

    openGripper(g.pre_grasp_posture);

    closedGripper(g.grasp_posture, grasp_pose.gripper_width);

    grasps.push_back(g);
  }

  group.setSupportSurfaceName("table");
  group.setGoalTolerance(0.01);
  return group.pick("abetSoup", grasps);
}

moveit::core::MoveItErrorCode place(moveit::planning_interface::MoveGroupInterface& group)
{
  std::vector<moveit_msgs::PlaceLocation> loc;

  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // get table height
  std::vector<std::string> object_ids;
  object_ids.push_back("table");
  auto table = planning_scene_interface.getObjects(object_ids).at("table");
  double table_height = table.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z];
  ROS_INFO("Table height: %f", table_height);

  // --- calculate desired pose of cracker (in base_link frame) when placing
  geometry_msgs::PoseStamped p;

  Eigen::Isometry3d place_pose = Eigen::Isometry3d::Identity();
  place_pose.translate(Eigen::Vector3d(0.0d, 0.0d, 0.05d));
  place_pose.rotate(Eigen::AngleAxisd(-M_PI_2, Eigen::Vector3d(0.0d, 1.0d, 0.0d)));
  p.header.frame_id = tf_prefix_ + "end_effector_link";
  tf::poseEigenToMsg(place_pose, p.pose);

  moveit_msgs::PlaceLocation g;
  g.place_pose = p;
  g.allowed_touch_objects.push_back("table");

  g.pre_place_approach.direction.header.frame_id = tf_prefix_ + "end_effector_link";
  g.pre_place_approach.desired_distance = 0.10;
  g.pre_place_approach.direction.vector.y = -1.0;
  g.pre_place_approach.direction.vector.z = -1.0;
  g.pre_place_approach.min_distance = 0.01;

  g.post_place_retreat.direction.header.frame_id = "world";
  g.post_place_retreat.direction.vector.z = 1.0;
  g.post_place_retreat.desired_distance = 0.10;
  g.post_place_retreat.min_distance = 0.01;

  loc.push_back(g);
  group.setSupportSurfaceName("table");

  ROS_INFO_STREAM("Place at " << g.place_pose);
  // auto error_code = group.place("cracker");
  auto error_code = moveit::core::MoveItErrorCode::SUCCESS;
  group.clearPathConstraints();
  return error_code;
}

int spawnGazeboModel(std::string objName, geometry_msgs::Pose pose, ros::ServiceClient gazebo_spawn_sdf_obj){
  ROS_INFO_STREAM("SPAWN SDF MODEL");

  gazebo_spawn_sdf_obj.waitForExistence();

  gazebo_msgs::SpawnModel spawn_model;
  spawn_model.request.model_name = "abetSoup"; //TODO: model name as arg, case for file path

  // load urdf file
  std::string sdf_filename = std::string("/home/kimlab/model_editor_models/abetSoup_resized/model.sdf"); // TODO: Make path relative
  

  ROS_INFO("loading file: %s", sdf_filename.c_str());
  // read sdf / gazebo model xml from file
  TiXmlDocument xml_in(sdf_filename);
  xml_in.LoadFile();
  std::ostringstream stream;
  stream << xml_in;
  spawn_model.request.model_xml = stream.str(); // load xml file
  // ROS_INFO("XML string: %s", stream.str().c_str());

  spawn_model.request.robot_namespace = "";
  spawn_model.request.initial_pose = pose;
  spawn_model.request.reference_frame = "world";

  if (gazebo_spawn_sdf_obj.call(spawn_model))
  {
      ROS_INFO("Spawn in simulation SUCCESSFUL");
      return 0;
  }
  else
  {
      ROS_ERROR("Failed to spawn model in sim! Error msg:%s", spawn_model.response.status_message.c_str());
      return 1;
  }
}

int spawnGazeboModel(vision_msgs::Detection3D detected_obj, ros::ServiceClient gazebo_spawn_sdf_obj){
    ROS_INFO_STREAM("SPAWN SDF MODEL");

    gazebo_spawn_sdf_obj.waitForExistence();

    gazebo_msgs::SpawnModel spawn_model;
    spawn_model.request.model_name = "cracker_det"; //TODO: model name as arg, case for file path

    // load urdf file
    std::string sdf_filename = std::string("/home/kazuki/model_editor_models/cracker_box_resized/model.sdf"); // TODO: Make path relative
    

    ROS_INFO("loading file: %s", sdf_filename.c_str());
    // read sdf / gazebo model xml from file
    TiXmlDocument xml_in(sdf_filename);
    xml_in.LoadFile();
    std::ostringstream stream;
    stream << xml_in;
    spawn_model.request.model_xml = stream.str(); // load xml file
    ROS_INFO("XML string: %s", stream.str().c_str());

    spawn_model.request.robot_namespace = "";
    geometry_msgs::Pose pose = detected_obj.bbox.center;
    spawn_model.request.initial_pose = pose;
    spawn_model.request.reference_frame = "robot1/end_link";

    if (gazebo_spawn_sdf_obj.call(spawn_model))
    {
        ROS_INFO("Spawn in simulation SUCCESSFUL");
        return 0;
    }
    else
    {
        ROS_ERROR("Failed to spawn model in sim! Error msg:%s", spawn_model.response.status_message.c_str());
        return 1;
    }
}

// Updates the simulation and planning scenes
int updateScene(moveit::planning_interface::PlanningSceneInterface &planning_scene_interface,
                        ros::NodeHandle &nh, moveit::planning_interface::MoveGroupInterface &group,
                        ros::ServiceClient gazebo_spawn_sdf_obj)
{
    // get objects from object detection
    bool found_obj = false;
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf2_listener(tf_buffer);
    geometry_msgs::TransformStamped world_to_camera_link; // My frames are named "base_link" and "leap_motion"

    
    while (!found_obj)
    {
        if (!ros::ok())
            return 0;

        vision_msgs::Detection3DArrayConstPtr detections =
            ros::topic::waitForMessage<vision_msgs::Detection3DArray>("/dope/detected_objects", nh, ros::Duration(30.0));
        if (!detections)
        {
            ROS_ERROR("Timed out while waiting for a message on topic detected_objects!");
            return 1;
        }

        // add objects to planning scene
        std::vector<moveit_msgs::CollisionObject> collision_objects;
        for (auto &&det3d : detections->detections)
        {
            if (det3d.results.empty())
            {
                ROS_ERROR("Detections3D message has empty results!");
                return 1;
            }

            if (det3d.results[0].id == ObjectID::ABETSOUP)
                found_obj = true;
            
            geometry_msgs::PoseStamped to_world_frame;
            to_world_frame.header = detections->header;
            to_world_frame.header.frame_id = "robot1/camera_link";
            to_world_frame.pose = det3d.bbox.center;
            geometry_msgs::PoseStamped in_world;
            world_to_camera_link = tf_buffer.lookupTransform("world", "robot1/camera_link", ros::Time(0), ros::Duration(10.0));
            tf2::doTransform(to_world_frame, in_world, world_to_camera_link);
            // spawnGazeboModel(det3d, gazebo_spawn_sdf_obj);
            ROS_INFO("Adding to planning scene");
            // add collision object to planning scene 
            moveit_msgs::CollisionObject co;
            co.header = detections->header;
            co.header.frame_id = "world";
            co.id = id_to_string(det3d.results[0].id);
            co.operation = moveit_msgs::CollisionObject::ADD;
            co.primitives.resize(1);
            co.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
            co.primitives[0].dimensions.resize(geometric_shapes::solidPrimitiveDimCount<shape_msgs::SolidPrimitive::BOX>());
            co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = det3d.bbox.size.x + 0.01;
            co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = det3d.bbox.size.y + 0.01;
            co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = det3d.bbox.size.z + 0.01;
            co.primitive_poses.resize(1);
            co.primitive_poses[0] = in_world.pose;
            // co.pose = t.pose; 
            collision_objects.push_back(co);

        }

        if (!found_obj)
            ROS_INFO_THROTTLE(1.0, "Still waiting for obj...");

        planning_scene_interface.addCollisionObjects(collision_objects);
    }

    // detach all objects
    auto attached_objects = planning_scene_interface.getAttachedObjects();
    for (auto &&object : attached_objects)
    {
        group.detachObject(object.first);
    }
    return 0;
}

moveit::core::MoveItErrorCode moveToCartesianPose(moveit::planning_interface::MoveGroupInterface &group,
                                                           Eigen::Isometry3d cartesian_pose,
                                                           std::string base_frame = tf_prefix_ + "link1",
                                                           std::string target_frame = tf_prefix_ + "end_effector_link")
{
    robot_state::RobotState start_state(*group.getCurrentState());
    group.setStartState(start_state);

    geometry_msgs::PoseStamped target_pose;
    target_pose.header.frame_id = base_frame;
    tf::poseEigenToMsg(cartesian_pose, target_pose.pose);

    group.setPoseTarget(target_pose, target_frame);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    auto error_code = group.plan(my_plan);
    bool success = (error_code == moveit::core::MoveItErrorCode::SUCCESS);

    ROS_INFO("Move planning (pose goal) %s", success ? "" : "FAILED");
    if (success)
    {
        error_code = group.execute(my_plan);
    }
    return error_code;
}

bool pause_service(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
    if (!paused)
    {
        ROS_INFO_STREAM("Pause statemachine after current state is completed");
        paused = true;
    }
    return true;
}

bool continue_service(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
    if (paused || failed)
    {
        paused = false;
        failed = false;
        ROS_INFO_STREAM("Continue statemachine");
    }
    return true;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "papras_pick_n_place");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::NodeHandle nh;
  state task_state = ST_INIT;
  state paused_state = ST_INIT;

  geometry_msgs::Pose base_pick_pose;
  geometry_msgs::Pose base_handover_pose;
  geometry_msgs::Pose base_place_pose;
  geometry_msgs::Pose base_home_pose;
  std::string world_name;
  bool handover_planned;

  // Load rosparams
  ros::NodeHandle nh_priv("~");
  std::string param_path;
  if (nh_priv.searchParam("tf_prefix", param_path))
    nh_priv.getParam(param_path, tf_prefix_);
  nh_priv.param<std::string>("tf_prefix", tf_prefix_, "robot1/");

  // ensure tf_prefix_ ends with exactly 1 '/' if nonempty, or "" if empty
  tf_prefix_ = tf_prefix_.erase(tf_prefix_.find_last_not_of('/') + 1) + "/";
  if (tf_prefix_.length() == 1)
    tf_prefix_ = "";

  ROS_INFO_STREAM("Current world name: PAPRAS Table");

  // pause service
  ros::ServiceServer pause_state = nh.advertiseService("pause_statemachine", pause_service);

  // pause service
  ros::ServiceServer continue_state = nh.advertiseService("continue_statemachine", continue_service);

  // GAZEBO SERVICE
  ros::ServiceClient gazebo_spawn_sdf_obj = nh.serviceClient<gazebo_msgs::SpawnModel>(SPAWN_SDF_OBJECT_TOPIC);

  moveit::planning_interface::MoveGroupInterface group("arm1");
  moveit::planning_interface::MoveGroupInterface hand_group("gripper1");

  // MOVE IT
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  // PLANNING INTERFACE
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  while (ros::ok())
  {
    if ((paused || failed) && !(task_state == ST_PAUSED))
    {
      ROS_INFO_STREAM("PAUSED in state " << task_state);
      ROS_INFO_STREAM("Call service continue_statemachine to resume");
      paused_state = task_state;
      task_state = ST_PAUSED;
    }
    switch (task_state)
    {
      case ST_INIT:
      {
        ROS_INFO_STREAM("ST_INIT");
        group.setPlanningTime(45.0);
        group.setPlannerId("RRTConnect");
        group.setMaxAccelerationScalingFactor(0.05);
        group.setMaxVelocityScalingFactor(0.05);
        hand_group.setMaxAccelerationScalingFactor(0.05);
        hand_group.setMaxVelocityScalingFactor(0.05);

        hand_group.setNamedTarget("close_loose");
        hand_group.plan(plan);
        hand_group.execute(plan);
        // initCollisionObject(planning_scene_interface);
        // detach all objects
        auto attached_objects = planning_scene_interface.getAttachedObjects();
        for (auto &&object : attached_objects)
        {
            group.detachObject(object.first);
        }

        task_state = ST_ARM_TO_REST_START;
        break;
      }
      case ST_PAUSED:
      {
        if (!paused && !failed)
        {
          task_state = paused_state;
          ROS_INFO_STREAM("Next state: " << task_state);
        }
        ros::Duration(0.1).sleep();
        break;
      }
      case ST_ARM_TO_REST_START:
      {
        ROS_INFO_STREAM("ST_ARM_TO_REST_START");

        /* ******************* MOVE ARM TO HOME ****************************** */
        // plan
        group.setPlannerId("RRTConnect");
        group.setNamedTarget("rest");
        moveit::core::MoveItErrorCode error_code = group.plan(plan);
        if (error_code == moveit::core::MoveItErrorCode::SUCCESS)
        {
          ROS_INFO("Planning to rest pose SUCCESSFUL");
        }
        else
        {
          ROS_ERROR("Planning to rest pose FAILED");
          failed = true;
          break;
        }

        // move
        error_code = group.execute(plan);
        if (error_code == moveit::core::MoveItErrorCode::SUCCESS)
        {
          ROS_INFO("Moving to rest pose SUCCESSFUL");
          task_state = ST_INIT_SIM_OBJ;
        }
        else
        {
          ROS_ERROR("Moving to rest pose FAILED");
          failed = true;
        }
        break;
      }
      case ST_INIT_SIM_OBJ:
      {
        geometry_msgs::Pose pose;
        pose.position.x = -0.20;
        pose.position.y = 0.55;
        pose.position.z = 0.90;
        double theta = 180 * (M_PI / 180);
        pose.orientation.x = pose.orientation.y = 0.0;
        pose.orientation.z = sin(theta / 2);
        pose.orientation.w = cos(theta / 2);

        // spawnGazeboModel("cracker", pose, gazebo_spawn_sdf_obj);
        task_state = ST_CAPTURE_OBJ;
        
      }
      case ST_CAPTURE_OBJ:
      {
        ROS_INFO_STREAM("ST_CAPTURE_OBJ");

        /* ********************* PLAN AND EXECUTE MOVES ********************* */

        // plan to observe the table
        if (OBJECT_TO_GRASP == ObjectID::SPOON || OBJECT_TO_GRASP == ObjectID::BUTTER)
          group.setNamedTarget("observe_spoon");
        else
          group.setNamedTarget("observe_table_left");
        // group.setNamedTarget("observe_table_waypoint1");
        moveit::core::MoveItErrorCode error_code = group.plan(plan);
        if (error_code == moveit::core::MoveItErrorCode::SUCCESS)
        {
          ROS_INFO("Planning to observation pose SUCCESSFUL");
        }
        else
        {
          ROS_ERROR("Planning to observation pose FAILED");
          failed = true;
          break;
        }

        // move to observation pose
        error_code = group.execute(plan);
        if (error_code == moveit::core::MoveItErrorCode::SUCCESS)
        {
          ROS_INFO("Moving to observation pose SUCCESSFUL");
          task_state = ST_PICK_OBJ;
        }
        else
        {
          ROS_ERROR("Moving to observation pose FAILED");
          failed = true;
        }
        ros::WallDuration(5.0).sleep();
        break;
      }
      case ST_PICK_OBJ:
      {
        ROS_INFO_STREAM("ST_PICK_OBJ");

        /* ********************* PICK ********************* */

        // clear octomap
        ros::ServiceClient clear_octomap = nh.serviceClient<std_srvs::Empty>("clear_octomap");
        std_srvs::Empty srv;
        group.clearPathConstraints();
        // pick
        moveit::core::MoveItErrorCode error_code;
        uint pickPlanAttempts = 0;
        do
        {
          clear_octomap.call(srv);

          int result = updateScene(planning_scene_interface, nh, group, gazebo_spawn_sdf_obj);
          if (result != 0)
          {
            ROS_ERROR("Updating planning scene FAILED");
            failed = true;
            break;
          }
          return 0;

          // error_code = group.planGraspsAndPick("bowl");
          // error_code = pick(group);
          // error_code = pick_place_object(group, hand_group, OBJECT_TO_GRASP);
          ++pickPlanAttempts;

          if (error_code == moveit::core::MoveItErrorCode::SUCCESS)
          {
            ROS_INFO("Picking SUCCESSFUL");
            task_state = ST_ARM_TO_TRANSPORT;
          }
          else if ((error_code == moveit::core::MoveItErrorCode::PLANNING_FAILED ||
                    error_code == moveit::core::MoveItErrorCode::INVALID_MOTION_PLAN) &&
                   pickPlanAttempts < 10)
          {
            ROS_INFO("Planning for Picking FAILED");
          }
          else
          {
            ROS_ERROR("Picking FAILED");
            failed = true;
          }

        } while ((error_code == moveit::core::MoveItErrorCode::PLANNING_FAILED ||
                  error_code == moveit::core::MoveItErrorCode::INVALID_MOTION_PLAN) &&
                 pickPlanAttempts < 10);
        break;
      }
      case ST_ARM_TO_TRANSPORT:
      {
        ROS_INFO_STREAM("ST_ARM_TO_TRANSPORT");

        /* ********************* PLAN AND EXECUTE TO TRANSPORT POSE ********************* */
        group.setStartStateToCurrentState();
        group.setNamedTarget("observe_table_right");
        moveit::core::MoveItErrorCode error_code = group.plan(plan);

        if (error_code == moveit::core::MoveItErrorCode::SUCCESS)
        {
          ROS_INFO("Planning to transport pose SUCCESSFUL");
        }
        else
        {
          ROS_ERROR("Planning to transport pose FAILED");
          failed = true;
          break;
        }

        // move to transport pose

        error_code = group.execute(plan);
        if (error_code == moveit::core::MoveItErrorCode::SUCCESS)
        {
          ROS_INFO("Moving to transport pose SUCCESSFUL");
          task_state = ST_PLACE_OBJ;
        }
        else
        {
          ROS_INFO_STREAM("Move to TRANSPORT failed");
          failed = true;
        }
        break;
      }
      case ST_PLACE_OBJ:
      {
        ROS_INFO_STREAM("ST_PLACE_OBJ");

        /* ********************* PLACE ********************* */

        // move(group, -0.05, -0.05, 0.2);
        ros::WallDuration(1.0).sleep();
        group.setPlannerId("RRTConnect");
        ROS_INFO("Start Placing");
        // place
        uint placePlanAttempts = 0;
        moveit::core::MoveItErrorCode error_code;
        do
        {
          group.setPlanningTime(40 + 10 * placePlanAttempts);
          error_code = place(group);
          ++placePlanAttempts;
          if (error_code == moveit::core::MoveItErrorCode::SUCCESS)
          {
            ROS_INFO("Placing SUCCESSFUL");
            task_state = ST_ARM_TO_HOME_END;
          }
          else if ((error_code == moveit::core::MoveItErrorCode::PLANNING_FAILED ||
                    error_code == moveit::core::MoveItErrorCode::INVALID_MOTION_PLAN ||
                    error_code == moveit::core::MoveItErrorCode::TIMED_OUT) &&
                   placePlanAttempts < 10)
          {
            ROS_INFO("Planning for Placing FAILED");
            ros::WallDuration(1.0).sleep();
          }
          else
          {
            ROS_ERROR("Placing FAILED");
            failed = true;
          }
        } while ((error_code == moveit::core::MoveItErrorCode::PLANNING_FAILED ||
                  error_code == moveit::core::MoveItErrorCode::INVALID_MOTION_PLAN ||
                  error_code == moveit::core::MoveItErrorCode::TIMED_OUT) &&
                 placePlanAttempts < 10);
        break;
      }
      case ST_ARM_TO_HOME_END:
      {
        ROS_INFO_STREAM("ST_ARM_TO_HOME_END");
        // plan to go home

        group.setPlannerId("RRTConnect");
        group.setStartStateToCurrentState();  // not sure why this is necessary after placing
        group.setNamedTarget("rest");
        moveit::core::MoveItErrorCode error_code = group.plan(plan);
        if (error_code == moveit::core::MoveItErrorCode::SUCCESS)
        {
          ROS_INFO("Planning to home pose SUCCESSFUL");
        }
        else
        {
          ROS_ERROR("Planning to home pose FAILED");
          failed = true;
          break;
        }

        // move to home
        error_code = group.execute(plan);
        if (error_code == moveit::core::MoveItErrorCode::SUCCESS)
        {
          ROS_INFO("Moving to home pose SUCCESSFUL");
          task_state = ST_DONE;
        }
        else
        {
          ROS_ERROR("Moving to home pose FAILED");
          failed = true;
        }
        break;
      }
      case ST_DONE:
      {
        ROS_INFO_STREAM("ST_DONE");
        return 0;
      }
      default:
      {
        ROS_INFO_STREAM("Unknown state");
        return 1;
      }
    }
  }
  return 1;
}