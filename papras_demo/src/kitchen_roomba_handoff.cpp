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

//Action Client
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <cmath>

std::string tf_prefix_ = "kitchen/robot1/";
const double tau = 2 * M_PI;

enum state
{
    ST_INIT,
    ST_ARM_TO_REST_START,
    ST_SPAWN_TRAY,
    ST_OBSERVE_OBJS,
    ST_PICK_PLACE_OBJS,
    ST_ARM_TO_HOME_END,
    ST_DONE,
    ST_PAUSED
};

enum ObjectID
{
    CRACKER = 1,
    GELATIN = 2,
    MEAT = 3,
    MUSTARD = 4,
    SOUP = 5,
    SUGAR = 6,
    BLEACH = 7,
    ABETSOUP = 9, 
    KETCHUP = 10, 
    PINEAPPLE = 11,
    BBQSAUCE = 12, 
    MACANDCHEESE = 13, 
    POPCORN = 14,
    BUTTER = 15, 
    MAYO = 16, 
    RAISINS = 17,
    CHERRIES = 18, 
    MILK = 19, 
    SALADDRESSING = 20,
    CHOCOLATEPUDDING = 21, 
    MUSHROOMS = 22, 
    SPAGHETTI = 23,
    COOKIES = 24, 
    TOMATOSAUCE = 26,
    CORN = 27, 
    OJ = 28, 
    TUNA = 29,
    CREAMCHEESE = 20, 
    PARMESAN = 31, 
    YOGURT =  32,
    GRANOLABARS = 33, 
    PEACHES = 34,
    GREENBEANS = 35, 
    PEASANDCARROTS = 36,
    BOWL = 37,
    SPOON = 38,
    CUP = 39,
    TABLE = 0
};

auto OBJECT_TO_GRASP = ObjectID::MILK;


std::string id_to_string(int id)
{
    switch (id)
    {
    case ObjectID::MILK:
        return "milk";
    case ObjectID::CUP:
        return "cup";
    case ObjectID::SPOON:
        return "spoon";
    case ObjectID::BOWL:
        return "bowl";
    case ObjectID::OJ:
        return "oJ";
    case ObjectID::BUTTER:
        return "butter";
    case ObjectID::ABETSOUP:
        return "abetSoup";
    case ObjectID::TABLE:
        return "table";
    case ObjectID::TOMATOSAUCE:
        return "tomatoSauce";
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

int pick_place_object(moveit::planning_interface::MoveGroupInterface& group, moveit::planning_interface::MoveGroupInterface& hand_group, ObjectID object_id, geometry_msgs::Pose objPose, int i){
  // get pose of detected object
  group.setPlanningTime(10.0);
  group.setNumPlanningAttempts(30.0);
  group.setMaxAccelerationScalingFactor(0.1);
  group.setMaxVelocityScalingFactor(0.1);
  group.setPlannerId("RRTConnect");

  geometry_msgs::PoseStamped pre_grasp_pose;

  // <<< ----- PRE GRASP  ----- >>>
  pre_grasp_pose = group.getCurrentPose();
  switch(object_id){
    case ObjectID::OJ:
    case ObjectID::MILK:
      pre_grasp_pose.pose.position.x = objPose.position.x;
      pre_grasp_pose.pose.position.y = objPose.position.y;
      pre_grasp_pose.pose.position.z = objPose.position.z + 0.15;
      break;
    case ObjectID::PEACHES:
    case ObjectID::ABETSOUP:
    case ObjectID::TOMATOSAUCE:
    case ObjectID::CUP :
      pre_grasp_pose.pose.position.x = objPose.position.x;
      pre_grasp_pose.pose.position.y = objPose.position.y;
      // pre_grasp_pose.pose.position.z = objPose.position.z + 0.1;
      pre_grasp_pose.pose.position.z = 0.895;
      break;
    default:
      ROS_INFO("object not in list");
      return 0;
  }
  // calculate orientation: parallel with xy plane, pointing from arm base to object
  // kitchen link 1 (x,y) ~= (0,0)
  double yaw = atan2(objPose.position.y, objPose.position.x);
  tf::Quaternion q;
  q.setRPY(0.0, 0.0, yaw);
  geometry_msgs::Quaternion q_msg;
  tf::quaternionTFToMsg(q, q_msg);
  pre_grasp_pose.pose.orientation = q_msg;
  
  group.setStartState(*group.getCurrentState());
  // group.setPoseTarget(pre_grasp_pose.pose);
  group.setJointValueTarget(pre_grasp_pose.pose);
  // check if target pose == start pose
  std::vector<double> start_angles = group.getCurrentJointValues();
  std::vector<double> goal_angles;
  group.getJointValueTarget(goal_angles);
  bool same_ik = true;

  for (int i = 0; i < 6; i++) {
    // std::cerr << "joint " << i << ": start=" << start_angles[i] << "\t goal=" << goal_angles[i] << std::endl;
    if (std::abs(start_angles[i] - goal_angles[i]) > 0.0001){
      same_ik = false;
      break;
    }
  }

  if (same_ik) {
    ROS_ERROR("OBJECT CANNOT BE REACHED");
    return -1;
  }

  // if ( *group.getCurrentState() == (moveit::core::RobotState) group.getJointValueTarget()) {
  //     ROS_INFO("CANT PICK");
  //     return -1;
  // }
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;


  hand_group.setNamedTarget("open");
  hand_group.plan(my_plan);
  hand_group.execute(my_plan);
  ros::Duration(3).sleep();
  
  
  
  std::cerr << "x: " << pre_grasp_pose.pose.position.x  << std::endl;
  std::cerr << "y: " << pre_grasp_pose.pose.position.y  << std::endl;
  std::cerr << "z: " << pre_grasp_pose.pose.position.z  << std::endl;
  // 
  bool success = (group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("tutorial", "Successful pregrasp plan %s", success ? "" : "FAILED");
  moveit_visual_tools::MoveItVisualTools visual_tools("world");
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");
  group.execute(my_plan);
  ros::Duration(0.5).sleep();


  // // <<< ----- GRASP  ----- >>>
  geometry_msgs::PoseStamped grasp_pose;
  grasp_pose = group.getCurrentPose();
  switch(object_id){
    case ObjectID::BOWL :
      grasp_pose.pose.position.x = pre_grasp_pose.pose.position.x;
      grasp_pose.pose.position.y = pre_grasp_pose.pose.position.y - 0.05;
      grasp_pose.pose.position.z = pre_grasp_pose.pose.position.z - 0.075;
      break;
    case ObjectID::OJ:
    case ObjectID::MILK:
      grasp_pose.pose.position.x = pre_grasp_pose.pose.position.x;
      grasp_pose.pose.position.y = pre_grasp_pose.pose.position.y;
      grasp_pose.pose.position.z = pre_grasp_pose.pose.position.z - 0.1;
      break;
    case ObjectID::PEACHES:
    case ObjectID::ABETSOUP:
    case ObjectID::TOMATOSAUCE:
    case ObjectID::CUP :
      grasp_pose.pose.position.x = pre_grasp_pose.pose.position.x;
      grasp_pose.pose.position.y = pre_grasp_pose.pose.position.y;
      grasp_pose.pose.position.z = pre_grasp_pose.pose.position.z - 0.04;
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
    default:
      ROS_INFO("object not in list");
      return 0;
  }

  group.setStartState(*group.getCurrentState());
  // group.setPoseTarget(grasp_pose);
  group.setJointValueTarget(grasp_pose.pose);
  success = (group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("tutorial", "Successful grasp plan %s", success ? "" : "FAILED");
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");
  group.execute(my_plan);

  hand_group.setNamedTarget("close");
  hand_group.plan(my_plan);
  hand_group.execute(my_plan);


  group.setStartState(*group.getCurrentState());
  // group.setPoseTarget(pre_grasp_pose.pose);
  group.setJointValueTarget(pre_grasp_pose.pose);

  success = (group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("tutorial", "Successful pregrasp plan %s", success ? "" : "FAILED");
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");
  group.execute(my_plan);
  ros::Duration(0.5).sleep();

  group.setStartStateToCurrentState();  // not sure why this is necessary after placing
  group.setNamedTarget("place_counter");
  success = (group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("tutorial", "Successful grasp plan %s", success ? "" : "FAILED");
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");
  group.execute(my_plan);

  hand_group.setNamedTarget("open");
  hand_group.plan(my_plan);
  hand_group.execute(my_plan);

  group.setStartStateToCurrentState();  // not sure why this is necessary after placing
  group.setNamedTarget("observe_roomba");
  success = (group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("tutorial", "Successful grasp plan %s", success ? "" : "FAILED");
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");
  group.execute(my_plan);

  return 0;
}

// Updates the simulation and planning scenes
std::map<ObjectID, geometry_msgs::PoseStamped> updateScene(ros::NodeHandle &nh)
{
    // get objects from object detection
    bool found_obj = false;
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf2_listener(tf_buffer);
    geometry_msgs::TransformStamped world_to_camera_link; // My frames are named "base_link" and "leap_motion"
    std::map<ObjectID, geometry_msgs::PoseStamped> collision_objects_found;
    
    while (!found_obj)
    {
        if (!ros::ok())
            return collision_objects_found;

        vision_msgs::Detection3DArrayConstPtr detections =
            ros::topic::waitForMessage<vision_msgs::Detection3DArray>("/dope/detected_objects", nh, ros::Duration(30.0));
        if (detections->detections.size() == 0)
        {
            ROS_ERROR("Timed out while waiting for a message on topic detected_objects!");
            return collision_objects_found;
        }

        // add objects to planning scene
        for (auto &&det3d : detections->detections)
        {
            if (det3d.results.empty())
            {
                ROS_ERROR("Detections3D message has empty results!");
                return collision_objects_found;
            }

            found_obj = true;  
            
            geometry_msgs::PoseStamped obj_camera_frame;
            obj_camera_frame.header = detections->header;
            obj_camera_frame.header.frame_id = "kitchen/robot1/camera_link";
            obj_camera_frame.pose = det3d.bbox.center;
            geometry_msgs::PoseStamped obj_world_frame;
            world_to_camera_link = tf_buffer.lookupTransform("kitchen/world", "kitchen/robot1/camera_link", ros::Time(0), ros::Duration(10.0));
            tf2::doTransform(obj_camera_frame, obj_world_frame, world_to_camera_link);
            // add collision object to map
            collision_objects_found.emplace((ObjectID) det3d.results[0].id, obj_world_frame);
            
        }

        if (!found_obj)
            ROS_INFO_THROTTLE(1.0, "Still waiting for obj...");

    }

    return collision_objects_found;

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
  ros::init(argc, argv, "handoff");
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
  std::map<ObjectID, geometry_msgs::PoseStamped> object_ids;
  bool handover_planned;

  // Load rosparams
  ros::NodeHandle nh_priv("~");
  std::string param_path;
  if (nh_priv.searchParam("tf_prefix", param_path))
    nh_priv.getParam(param_path, tf_prefix_);
  nh_priv.param<std::string>("tf_prefix", tf_prefix_, "kitchen/robot1/");

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

        hand_group.setNamedTarget("open");
        hand_group.plan(plan);
        hand_group.execute(plan);

        hand_group.setNamedTarget("close");
        hand_group.plan(plan);
        hand_group.execute(plan);
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
        moveit_visual_tools::MoveItVisualTools visual_tools("world");
        visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

        // move
        error_code = group.execute(plan);
        if (error_code == moveit::core::MoveItErrorCode::SUCCESS)
        {
          ROS_INFO("Moving to rest pose SUCCESSFUL");
          task_state = ST_OBSERVE_OBJS;
        }
        else
        {
          ROS_ERROR("Moving to rest pose FAILED");
          failed = true;
        }
        break;
      }
      case ST_OBSERVE_OBJS:
      {
        ROS_INFO_STREAM("ST_CAPTURE_OBJS");
        group.setNamedTarget("observe_roomba"); 
        
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
        moveit_visual_tools::MoveItVisualTools visual_tools("world");
        visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

        error_code = group.execute(plan);
        if (error_code == moveit::core::MoveItErrorCode::SUCCESS)
        {
          ROS_INFO("Moving to observation pose SUCCESSFUL");
         
        }
        else
        {
          ROS_ERROR("Moving to observation pose FAILED");
          failed = true;
          break;
        }
        ros::WallDuration(5.0).sleep();

        group.clearPathConstraints();

        object_ids = updateScene(nh);  

        if (object_ids.size() == 0) {
          task_state = ST_ARM_TO_HOME_END;
        } else {
          task_state = ST_PICK_PLACE_OBJS;
        }
        break;
      }
      case ST_PICK_PLACE_OBJS:
      {
        ROS_INFO_STREAM("ST_PICK_PLACE_OBJECT");

        /* ********************* PLAN AND EXECUTE MOVES ********************* */

        moveit::core::MoveItErrorCode error_code;
        uint pickPlanAttempts = 0;

        // sort object_ids by distance
        std::vector<std::pair<double, ObjectID>> objs_dists;
        for (auto obj_it : object_ids) {
          double dist = obj_it.second.pose.position.x*obj_it.second.pose.position.x + obj_it.second.pose.position.y*obj_it.second.pose.position.y; // kitchen mount (x,y) ~= (0,0)
          objs_dists.push_back({dist, obj_it.first});
        }
        std::sort(objs_dists.begin(), objs_dists.end());
        
        for(int i = 0; i < object_ids.size(); i++){
            pickPlanAttempts = 0;
            do{
                error_code = pick_place_object(group, hand_group, objs_dists[i].second, object_ids.at(objs_dists[i].second).pose, i);
                ++pickPlanAttempts;

                if ((error_code == moveit::core::MoveItErrorCode::PLANNING_FAILED ||
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

            }while((error_code == moveit::core::MoveItErrorCode::PLANNING_FAILED ||
                  error_code == moveit::core::MoveItErrorCode::INVALID_MOTION_PLAN) &&
                 pickPlanAttempts < 10);

        }   
        if(!failed){
            task_state = ST_OBSERVE_OBJS;
        }
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
        moveit_visual_tools::MoveItVisualTools visual_tools("world");
        visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

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